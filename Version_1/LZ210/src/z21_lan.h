#pragma once
#include "module_arch.h"
#include "command_bus.h"
#include "event_bus.h"
#include "xpressnet_handler.h"
#include "loco_repository.h"
#include <Ethernet.h>
#include <EthernetUdp.h>

// ═══════════════════════════════════════════════════════════════
//  Z21Lan — Roco Z21 LAN UDP server (core1)
//
//  Implements the Z21 LAN protocol v1.10 over UDP port 21105.
//  XpressNet packets embedded in Z21 frames (header 0x40xx) are
//  processed via the shared XpressNetHandler.
//
//  Client tracking:
//    Up to Z21_MAX_CLIENTS unique IP+port combinations are tracked.
//    Each client has a set of broadcast flags that determine which
//    events it receives. Clients are removed after Z21_CLIENT_TIMEOUT
//    milliseconds of inactivity.
//
//  Z21 frame format:
//    [DataLen Lo][DataLen Hi][Header Lo][Header Hi][Data...]
//    DataLen = total packet length including the 4-byte header
//
//  Supported commands: loco drive, functions, turnouts, CV read/write,
//  POM read/write, system state, broadcast flags, fast clock,
//  LocoNet dispatch, RS-Bus (RMBus) data.
// ═══════════════════════════════════════════════════════════════

#define Z21_PORT           21105          // UDP port number
#define Z21_MAX_CLIENTS    16             // Max tracked clients
#define Z21_CLIENT_TIMEOUT 30000          // Client inactivity timeout (ms)
#define Z21_UDP_BUF        256            // UDP transmit buffer size

// ─────────────────────────────────────────────────────────────
//  Z21BcFlag — broadcast flag bitmask values
//
//  Each client subscribes to events by OR-ing flags via
//  LAN_SET_BROADCASTFLAGS. The station only sends broadcasts
//  to clients that have the corresponding flag set.
// ─────────────────────────────────────────────────────────────
namespace Z21BcFlag {
    constexpr uint32_t DRIVING_LOCO  = 0x00000001;  // Loco driven by this client
    constexpr uint32_t ALL_LOCOS     = 0x00000002;  // All loco state changes
    constexpr uint32_t SWITCH_INFO   = 0x00000004;  // Turnout state changes
    constexpr uint32_t RMT_SENS_INFO = 0x00000010;  // RS-Bus feedback changes
    constexpr uint32_t SYSTEM_STATE  = 0x00000100;  // Power/estop state changes
    constexpr uint32_t RAILCOM       = 0x00040000;  // RailCom data
    constexpr uint32_t LOCO_NET      = 0x01000000;  // LocoNet messages
    constexpr uint32_t LOCONET_LOCO  = 0x02000000;  // LocoNet loco messages
    constexpr uint32_t LOCONET_SWITCH= 0x04000000;  // LocoNet switch messages
    constexpr uint32_t LOCONET_DETECT= 0x08000000;  // LocoNet detection messages
    constexpr uint32_t RMBUS         = 0x00000080;  // RS-Bus (RMBus) data
}

// ─────────────────────────────────────────────────────────────
//  Z21Client — state of a single Z21 UDP client
// ─────────────────────────────────────────────────────────────
struct Z21Client {
    IPAddress ip;          // Client IP address
    uint16_t  port;        // Client UDP port
    uint32_t  bcFlags;     // Active broadcast flag subscriptions
    uint32_t  lastSeenMs;  // Timestamp of last received packet
    bool      active;      // True if this slot is in use
};

// ─────────────────────────────────────────────────────────────
//  Z21Lan — Z21 LAN UDP server
// ─────────────────────────────────────────────────────────────
class Z21Lan : public ProtocolModule {
public:
    Z21Lan() : ProtocolModule("Z21Lan", ModuleId::Z21_LAN,
                               ModuleCore::CORE1) {}

    // Starts UDP listener on Z21_PORT, registers EventBus handler
    void begin()  override;

    // Receives UDP packets, dispatches to appropriate handlers,
    // checks client timeouts
    void loop()   override;

    // Converts hardware events to Z21 broadcasts:
    // POWER_ON/OFF  → LAN_X_BC_TRACK_POWER_OFF/ON
    // LOCO_STATE    → LAN_X_LOCO_INFO (to subscribed clients)
    // FEEDBACK_BULK → LAN_RMBUS_DATACHANGED
    void onEvent(const Event& ev) override;

    uint8_t connectionCount() const override { return _clientCount; }
    uint8_t maxConnections()  const override { return Z21_MAX_CLIENTS; }

private:
    EthernetUDP _udp;
    Z21Client   _clients[Z21_MAX_CLIENTS];
    uint8_t     _clientCount = 0;
    uint8_t     _txBuf[Z21_UDP_BUF];

    // Dispatches one received UDP packet to the appropriate handler.
    // Validates the packet's stated length against the actual number
    // of bytes received, looks up (or creates) the client entry for
    // the sender's IP+port, refreshes its last-seen timestamp, and
    // switches on the 16-bit Z21 header value to call the matching
    // _handle*() method with the remaining payload bytes.
    void _processPacket(const uint8_t* buf, uint16_t len,
                        const IPAddress& ip, uint16_t port);

    // ── Z21 command handlers ──────────────────────────────

    // LAN_GET_SERIAL_NUMBER — replies with a fixed, fake serial number.
    void _handleLanGetSerialNumber   (Z21Client& c);
    // LAN_GET_VERSION — replies with a fixed hardware/firmware version.
    void _handleLanGetVersion        (Z21Client& c);
    // LAN_GET_STATUS — replies with the current system state packet.
    void _handleLanGetStatus         (Z21Client& c);
    // LAN_SET_BROADCASTFLAGS — stores the requested broadcast flag
    // bitmask for this client, controlling which future events it
    // will receive.
    void _handleSetBcFlags           (Z21Client& c, const uint8_t* data, uint8_t len);
    // LAN_LOGON — sends the current system state followed by a full
    // dump of the known RS-Bus feedback state (per group of 10
    // modules), so a newly connecting client is immediately up to
    // date without having to poll for it.
    void _handleLogon                (Z21Client& c);
    // LAN_LOGOFF — deactivates this client's slot immediately (rather
    // than waiting for the inactivity timeout) and frees it up for
    // reuse.
    void _handleLogoff               (Z21Client& c);
    // LAN_X_BUS — handles an XpressNet command tunnelled inside a Z21
    // packet. Contains a Z21-specific shortcut for the MultiMaus
    // extended loco-info query (E3 F0), a Z21-specific address/data
    // remapping for LAN_X_SET_TURNOUT (X-header 0x53) before handing
    // off to the shared XpressNet handler, and translates the
    // handler's result into either a direct LAN_X reply or a
    // LAN_X_TURNOUT_INFO broadcast.
    void _handleXBus                 (Z21Client& c, const uint8_t* data, uint8_t len);
    // LAN_X_GET_LOCO_INFO — replies with the current state of the
    // requested loco address as a LAN_X_LOCO_INFO packet.
    void _handleGetLocoInfo          (Z21Client& c, const uint8_t* data, uint8_t len);
    // LAN_LOCONET_DISPATCH_ADDR — dispatches an LN_DISPATCH command
    // for the given LocoNet address onto the command bus.
    void _handleLocoNetDispatch      (Z21Client& c, const uint8_t* data, uint8_t len);
    // LAN_RMBUS_GETDATA — replies with the current RS-Bus feedback
    // state for one group of 10 modules.
    void _handleRmbusGetData         (Z21Client& c, const uint8_t* data, uint8_t len);
    // LAN_FAST_CLOCK_DATA (as a query, or as the common reply used by
    // the various fast-clock control sub-commands) — replies with the
    // current fast-clock time/state.
    void _handleFastClockGet         (Z21Client& c, const uint8_t* data, uint8_t len);
    // LAN_FAST_CLOCK_DATA (as a control command) — central dispatcher
    // for the fast clock's GET/SET/START/STOP sub-commands, identified
    // by their first two payload bytes.
    void _handleFastClockSet         (Z21Client& c, const uint8_t* data, uint8_t len);
    // LAN_FAST_CLOCK_SETTINGS — replies with the persistent fast-clock
    // settings (enable flags, rate, start time).
    void _handleFastClockSettingsGet (Z21Client& c);
    // LAN_FAST_CLOCK_SETTINGS_SET — applies new persistent fast-clock
    // settings and replies with the resulting settings as confirmation.
    void _handleFastClockSettingsSet (Z21Client& c, const uint8_t* data, uint8_t len);
    // LAN_SYSTEMSTATE_GETDATA — replies with the current system state
    // packet (same payload as sent as a broadcast on power changes).
    void _handleSystemStateGet       (Z21Client& c);
    // LAN_GET_BROADCASTFLAGS — replies with this client's currently
    // active broadcast flag bitmask.
    void _handleGetBcFlags           (Z21Client& c);
    // LAN_GET_LOCOMODE — replies with a fixed "DCC mode" answer for
    // the requested loco address (Motorola mode is not implemented).
    void _handleGetLocoMode          (Z21Client& c, const uint8_t* data, uint8_t len);
    // LAN_GET_TURNOUTMODE — replies with the stored DCC/Motorola mode
    // flag for the requested turnout address.
    void _handleGetTurnoutMode       (Z21Client& c, const uint8_t* data, uint8_t len);
    // LAN_SET_TURNOUTMODE — stores the DCC/Motorola mode flag for the
    // given turnout address. No reply is sent, per spec.
    void _handleSetTurnoutMode       (Z21Client& c, const uint8_t* data, uint8_t len);
    // LAN_X_SET_LOCO_DRIVE (128-step Z21 loco speed command) —
    // dispatches a LOCO_SPEED command onto the command bus.
    void _handleLocoSpeed            (Z21Client& c, const uint8_t* data, uint8_t len);
    // LAN_X_SET_LOCO_FUNCTION (Z21 loco function command) —
    // dispatches a LOCO_FUNCTION command onto the command bus.
    void _handleLocoFunction         (Z21Client& c, const uint8_t* data, uint8_t len);
    // LAN_CV_READ — dispatches a CV_READ command (service-mode CV
    // read) onto the command bus.
    void _handleCvRead               (Z21Client& c, const uint8_t* data, uint8_t len);
    // LAN_CV_WRITE — dispatches a CV_WRITE command (service-mode CV
    // write) onto the command bus.
    void _handleCvWrite              (Z21Client& c, const uint8_t* data, uint8_t len);
    // LAN_CV_POM_READ — dispatches a POM_READ command (programming on
    // main, for a specific loco address) onto the command bus.
    void _handleCvPomRead            (Z21Client& c, const uint8_t* data, uint8_t len);
    // LAN_CV_POM_WRITE — dispatches a POM_WRITE command onto the
    // command bus.
    void _handleCvPomWrite           (Z21Client& c, const uint8_t* data, uint8_t len);

    // ── Transmit helpers ──────────────────────────────────

    // Sends a UDP packet to one client
    void _send(Z21Client& c, const uint8_t* buf, uint16_t len);

    // Sends a Z21 header-only response (no data payload)
    void _sendHeader(Z21Client& c, uint16_t hdr, uint16_t dataLen);

    // Sends a UDP packet to all clients with matching broadcast flags
    void _broadcast(const uint8_t* buf, uint16_t len, uint32_t flagMask);

    // ── Frame builders ────────────────────────────────────

    // Fills buf with a LAN_SYSTEMSTATE_DATACHANGED packet
    void _buildSystemState(uint8_t* buf, uint16_t& len);

    // Fills buf with a LAN_X_LOCO_INFO packet for a given address
    void _buildLocoInfo   (uint8_t* buf, uint16_t& len, uint16_t addr);

    // Wraps an XpressNet reply frame in a Z21 LAN_X packet
    void _buildXBusReply  (uint8_t* buf, uint16_t& len, const XnFrame& f);

    // ── Client management ─────────────────────────────────

    // Finds an existing client slot or allocates a new one
    Z21Client* _findOrAddClient(const IPAddress& ip, uint16_t port);

    // Removes clients that have exceeded Z21_CLIENT_TIMEOUT
    void       _checkTimeouts();

    // ── Utility ───────────────────────────────────────────

    // Read a little-endian uint16 from a byte buffer
    static uint16_t _u16le(const uint8_t* b) {
        return (uint16_t)b[0] | ((uint16_t)b[1] << 8);
    }
    // Write a little-endian uint16 to a byte buffer
    static void _putU16le(uint8_t* b, uint16_t v) {
        b[0]=(uint8_t)v; b[1]=(uint8_t)(v>>8);
    }
};

extern Z21Lan gZ21Lan;
