#pragma once
#include "module_arch.h"
#include "command_bus.h"
#include "event_bus.h"
#include "xpressnet_handler.h"
#include "xn_interface_layer.h"

// ═══════════════════════════════════════════════════════════════
//  LenzUsb — Lenz LI-USB via USB CDC serial (core1)
//
//  Implements the same framing protocol as LenzLan but over
//  the USB CDC serial port (Serial). This allows a PC connected
//  via USB to use Lenz-compatible software (e.g. iTrain, Traincontroller).
//
//  Differences from LenzLan:
//  - No TCP connection management (single implicit connection)
//  - No keep-alive timeout (USB is always connected)
//  - Serial.flush() is called after each write for immediate delivery
//  - No hardware handshake (no DTR detection)
//
//  Serial.begin() is called in begin() on core1.
// ═══════════════════════════════════════════════════════════════

#define LENZ_USB_RX_BUF      128     // Receive buffer size (bytes)
#define LENZ_USB_HDR1        0xFF    // First header byte (always)
#define LENZ_USB_HDR_CMD     0xFE   // Second byte: direct reply
#define LENZ_USB_HDR_BC      0xFD   // Second byte: broadcast
#define LENZ_USB_TIMEOUT_MS  5000   // Inactivity timeout (5s)
#define LENZ_USB_PROG_TO     90000  // Programming mode timeout (90s)
#define LENZ_USB_BAUD        57600  // Serial baud rate
#define LENZ_USB_FRAME_TIMEOUT_MS 250 // Max time allowed for an incomplete frame (spec: "01/01/00")

class LenzUsb : public ProtocolModule {
public:
    LenzUsb() : ProtocolModule("LenzUsb", ModuleId::LENZ_USB,
                                ModuleCore::CORE1) {}

    // Initialises USB CDC serial at LENZ_USB_BAUD, registers EventBus handler
    void begin()  override;

    // Reads available serial data, assembles frames, processes complete frames
    void loop()   override;

    // Converts hardware events to USB broadcasts (same format as LenzLan)
    void onEvent(const Event& ev) override;

    // Returns 1 if USB activity has been detected, 0 otherwise
    uint8_t connectionCount() const override { return _hasActivity ? 1 : 0; }
    uint8_t maxConnections()  const override { return 1; }

private:
    uint8_t  _rxBuf[LENZ_USB_RX_BUF];
    uint8_t  _rxLen       = 0;
    bool     _progMode    = false;
    bool     _hasActivity = false;
    uint32_t _lastActMs   = 0;
    uint32_t _frameStartMs = 0;  // start of the current incomplete frame (0 = none)

    // Interface-local layer (see xn_interface_layer.h)
    XnInterfaceLayer _ifLayer;

    // USB is a single serial link — there is no socket pool like
    // Ethernet has. Always reports 1 "free connection" when nothing
    // is currently active, 0 when that one link is already in use.
    // (Deliberately kept simple: USB has no socket pool to query.)
    static uint8_t _freeConnections();

    // Parses and dispatches one complete LAN frame
    void _processFrame(const uint8_t* buf, uint8_t len);

    // Low-level primitive: writes raw bytes to Serial and flushes
    // immediately afterwards. All other send paths (_sendFrame, the
    // keep-alive reply, the RS-Bus dump, onEvent feedback) go through
    // this function, so the flush behaviour is defined in exactly one
    // place.
    void _sendRaw(const uint8_t* buf, uint8_t len);

    // Sends an XpressNet frame with LAN header over USB serial
    void _sendFrame(const XnFrame& f);

    // Sends an XpressNet frame as a broadcast over USB serial
    void _sendBroadcast(const XnFrame& f);

    // Returns the length of the next complete frame, or 0 if incomplete
    static uint8_t _frameLen(const uint8_t* buf, uint8_t available);
};

extern LenzUsb gLenzUsb;
