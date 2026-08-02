// ═══════════════════════════════════════════════════════════════
//  lenz_lan.h  —  Lenz LAN TCP server (core1)
//
//  Implements the Lenz LI-USB-Ethernet protocol (LAN v3.6) on
//  TCP port 5550. Supports up to LENZ_LAN_MAX_CLIENTS simultaneous
//  client connections.
//
//  Framing protocol:
//    PC → station:  FF FE [XN bytes] [XOR]   (command)
//    station → PC:  FF FE [XN bytes] [XOR]   (direct reply)
//    station → PC:  FF FD [XN bytes] [XOR]   (broadcast to all)
//
//  Special non-XpressNet messages:
//    F1 01 F0      = keep-alive request (sent by PC every ~30s)
//    F2 01 01 F2   = keep-alive response (sent by station)
//
//  On new connection, the station sends:
//    1. Welcome message: FF FD 61 01 60 (normal operation broadcast)
//    2. Initial RS-Bus dump: all non-zero feedback module states
//       as FF FD 46 packets (max 3 pairs per packet)
// ═══════════════════════════════════════════════════════════════
#pragma once
#include "module_arch.h"
#include "command_bus.h"
#include "event_bus.h"
#include "xpressnet_handler.h"
#include "xn_interface_layer.h"
#include "eeprom_store.h"
#include <Ethernet.h>

#define LENZ_LAN_MAX_CLIENTS   8        // Max simultaneous TCP connections
#define LENZ_LAN_RX_BUF       128      // Receive buffer per client (bytes)
#define LENZ_LAN_HDR1         0xFF     // First header byte (always)
#define LENZ_LAN_HDR_CMD      0xFE    // Second byte: direct reply
#define LENZ_LAN_HDR_BC       0xFD    // Second byte: broadcast to all clients
#define LENZ_LAN_TIMEOUT_MS   30000   // Inactivity timeout (30s, prevented by keep-alive)
#define LENZ_LAN_PROG_TIMEOUT 90000   // Timeout in programming mode (90s)
#define LENZ_LAN_FRAME_TIMEOUT_MS 250 // Max time allowed for an incomplete frame (spec: "01/01/00")

// ─────────────────────────────────────────────────────────────
//  LenzClient — state of a single TCP client connection
// ─────────────────────────────────────────────────────────────
struct LenzClient {
    EthernetClient tcp;                      // TCP socket
    uint8_t        rxBuf[LENZ_LAN_RX_BUF];  // Receive buffer
    uint8_t        rxLen;                    // Bytes currently in rxBuf
    bool           progMode;                 // True if client is in programming mode
    uint32_t       lastActMs;               // Timestamp of last activity (millis)
    uint32_t       frameStartMs;            // Timestamp at which the current incomplete frame started (0 = none)
    bool           active;                   // True if this slot is in use

    LenzClient() : rxLen(0), progMode(false), lastActMs(0), frameStartMs(0), active(false) {}
};

// ─────────────────────────────────────────────────────────────
//  LenzLan — TCP server for the Lenz LAN protocol
// ─────────────────────────────────────────────────────────────
class LenzLan : public ProtocolModule {
public:
    LenzLan() : ProtocolModule("LenzLan", ModuleId::LENZ_LAN,
                                ModuleCore::CORE1) {}

    // Starts the TCP server on port 5550, registers EventBus handler
    void begin()  override;

    // Accepts new connections, processes incoming data, checks timeouts
    void loop()   override;

    // Converts hardware events to LAN broadcasts:
    // POWER_ON/OFF → 0x61 broadcast
    // LOCO_STATE   → 0xE4 broadcast
    // FEEDBACK_BULK → 0x46 RS-Bus packets
    void onEvent(const Event& ev) override;

    uint8_t connectionCount() const override { return _connCount; }
    uint8_t maxConnections()  const override { return LENZ_LAN_MAX_CLIENTS; }

// Feedback queue — buffers RS-Bus events that arrive during client processing
#define LENZ_LAN_FB_QUEUE_SIZE  56   // Maximum queued addr/data pairs (28 pairs × 2)

private:
    EthernetServer  _server{5550};
    LenzClient      _clients[LENZ_LAN_MAX_CLIENTS];
    uint8_t         _connCount = 0;

    // TX serialisation: prevents RS-Bus broadcasts from interleaving with
    // response packets while _processClient() is writing to the TCP buffer.
    bool    _txBusy = false;

    // Pending feedback queue (filled when _txBusy, drained by _flushPendingFeedback)
    uint8_t _fbQueueAddr[LENZ_LAN_FB_QUEUE_SIZE];
    uint8_t _fbQueueData[LENZ_LAN_FB_QUEUE_SIZE];
    uint8_t _fbQueueCount = 0;

    // Queue RS-Bus feedback pairs for deferred sending
    void _queueFeedback(const uint8_t* addr, const uint8_t* data, uint8_t count);

    // Send all queued RS-Bus feedback packets (called after _txBusy cleared)
    void _flushPendingFeedback();

    // Sends RS-Bus feedback as 0x46 packets (max 3 pairs per packet)
    void _sendFeedbackBulk(const uint8_t* addr, const uint8_t* data, uint8_t count);

    // Accepts a new TCP connection and sends welcome message + RS-Bus dump
    void _acceptNew();

    // Reads available data from one client, assembles complete frames,
    // calls _processFrame for each complete frame
    void _processClient(LenzClient& c);

    // Parses one complete LAN frame and dispatches to XpressNet handler
    void _processFrame(LenzClient& c, const uint8_t* buf, uint8_t len);

    // Returns the length of the next complete frame in the buffer,
    // or 0 if no complete frame is available yet
    uint8_t _frameLen(const uint8_t* buf, uint8_t available);

    // Low-level primitive: writes raw bytes to one client and flushes
    // immediately (one write = one TCP segment). All other send paths
    // (_sendFrame, the _acceptNew welcome/RS-Bus dump, _sendFeedbackBulk)
    // go through this function, so the flush behaviour is defined in
    // exactly one place.
    void _sendRaw(LenzClient& c, const uint8_t* buf, uint8_t len);

    // Sends an XpressNet frame to a single client with LAN header
    void _sendFrame(LenzClient& c, const XnFrame& f);

    // Interface-local layer (see xn_interface_layer.h): intercepts
    // 0xF0/0xF1.../0xF2/0x01 and buffers broadcasts so they are sent
    // in the correct order relative to the 01/04/05 acknowledgement.
    XnInterfaceLayer _ifLayer;

    // Returns the actual number of free W5100/W5500 hardware sockets
    // (spec "Verfügbare Freie Verbindungen") — see the .cpp file for
    // exactly which Ethernet-library symbol this uses; verify this
    // against the Ethernet library version that is actually installed.
    static uint8_t _freeConnections();

    // Sends an XpressNet frame to all active clients as a broadcast
    void _sendBroadcast(const XnFrame& f);

    // Disconnects clients that have exceeded LENZ_LAN_TIMEOUT_MS
    void _checkTimeouts();
};

extern LenzLan gLenzLan;
