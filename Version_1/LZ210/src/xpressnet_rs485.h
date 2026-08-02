#pragma once
#include "module_arch.h"
#include "command_bus.h"
#include "event_bus.h"
#include "xpressnet_handler.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"

// ─────────────────────────────────────────────────────────────
//  XNSlaveType — identified handheld type on the RS-485 bus
//
//  Tracked per slave address so the shared XpressNetHandler can
//  adapt certain protocol-dependent responses to the specific type
//  of device that is currently being polled (e.g. a Roco MultiMaus
//  behaves slightly differently on some queries than a standard
//  Lenz-style handheld such as an LH100).
// ─────────────────────────────────────────────────────────────
enum class XNSlaveType : uint8_t {
    UNKNOWN   = 0,  // Not yet identified
    STANDARD  = 1,  // Standard XpressNet handheld (LH100 etc.)
    MULTIMAUS = 2,  // Roco MultiMaus / LokMouse2
};

// ═══════════════════════════════════════════════════════════════
//  XpressNetRs485 — XpressNet RS-485 bus master via PIO2 (core1)
//
//  Polls all 31 slave addresses cyclically with Normale Anfrage
//  (normal inquiry) rufbytes. Flushes the broadcast queue after
//  each poll cycle. Frames are decoded via XpressNetHandler.
//
//  Discovery strategy:
//    Known (present) slaves are polled on every cycle.
//    Every 8th cycle, one unknown address is probed for new devices.
//    Once a device responds, it enters the fast poll cycle.
//
//  PIO2 SM0 = TX (9-bit UART, program origin 0)
//  PIO2 SM1 = RX (9-bit UART, program origin 4)
//
//  Pin assignments:
//    GP10 = TX
//    GP11 = RX
//    GP12 = DE (HIGH=transmit, LOW=receive)
// ═══════════════════════════════════════════════════════════════

#include "hardware_config.h"

#define XN_RS485_TX_PIN       HW_XN_TX_PIN
#define XN_RS485_RX_PIN       HW_XN_RX_PIN
#define XN_RS485_DE_PIN       HW_XN_DE_PIN
#define XN_RS485_BAUD         62500
#define XN_RS485_MAX_SLAVES   31
#define XN_RS485_REPLY_US     500  // Reply timeout in µs (was 1000 — further lowered, towards DCC-EX's ~500µs)
#define XN_RS485_MIN_US       40
#define XN_INTER_POLL_US      150  // Inter-poll delay in µs (was 500 — further lowered)
#define XN_RS485_TX_SETTLE    10
#define XN_RS485_BC_REPEAT    1
#define XN_RS485_RX_BUF       32
#define XN_BIT_US             16
#define XN_BYTE_US            176
#define XN_FRAME_GAP_US       352
#define XN_TX_ORIGIN          0
#define XN_RX_ORIGIN          4

// XpressNet rufbyte type field values (bits 6-5 of the rufbyte)
#define XN_RUF_NORMALE_ANFRAGE  0x40
#define XN_RUF_QUITTIERUNG      0x00
#define XN_RUF_BROADCAST        0x60

// ─────────────────────────────────────────────────────────────
//  XNSlave — per-address bookkeeping for one RS-485 bus position
//
//  One entry per possible slave address (1-31). Used by the polling
//  cycle to decide which addresses are worth polling on every pass
//  (present==true) versus which are still unknown/absent and should
//  only be probed occasionally during discovery.
// ─────────────────────────────────────────────────────────────
struct XNSlave {
    bool        present;           // True if this slave has responded recently
    bool        needsQuittierung;    // True if slave requested acknowledgment
    uint32_t    lastSeenMs;          // Timestamp of last successful response
    uint8_t     missCount;           // Consecutive missed polls before marking absent
    XNSlaveType type;               // Detected handheld type
};

#define XN_BC_QUEUE_SIZE  8
#define XN_BC_MAX_LEN     24

// ─────────────────────────────────────────────────────────────
//  XNBroadcast — one queued outgoing broadcast frame
//
//  Broadcasts (power state, loco info, feedback, etc.) are queued
//  here rather than sent immediately, because the RS-485 bus can
//  only be driven when it is our turn (between polls) — see
//  enqueueBroadcast()/_flushBroadcasts().
// ─────────────────────────────────────────────────────────────
struct XNBroadcast {
    uint8_t data[XN_BC_MAX_LEN];
    uint8_t len;
    bool    pending;
};

// ═══════════════════════════════════════════════════════════════
//  XpressNetRs485 — the RS-485 bus master module itself
//
//  Runs entirely on core1. Each call to loop() flushes any pending
//  broadcasts and then polls exactly one slave address (see
//  _pollSlave()), advancing to the next address afterwards. All the
//  actual byte-level protocol handling (turnouts, loco commands,
//  queries, ...) is delegated to a shared XpressNetHandler instance;
//  this class is only responsible for the RS-485 transport layer:
//  bus arbitration (who may talk when), rufbyte addressing, framing,
//  and timing.
// ═══════════════════════════════════════════════════════════════
class XpressNetRs485 : public HardwareModule {
public:
    XpressNetRs485()
        : HardwareModule("XpressNetRs485", ModuleId::XPRESSNET_RS485,
                          ModuleCore::CORE1) {}

    // Initialises the DE pin, both PIO state machines (TX and RX),
    // resets all per-slave bookkeeping and the broadcast queue, and
    // registers the EventBus handler used to receive broadcast-
    // worthy events (power state, feedback, ...) from other modules
    // running on core0. Also queues an initial "track power off"
    // broadcast so that any already-connected handheld (e.g. a
    // MultiMaus) immediately shows the correct stopped state.
    void begin()  override;

    // Called once per core1 loop iteration. Flushes any broadcasts
    // that have been queued since the last call, polls exactly one
    // slave address (the active programming-mode slave if one is
    // set, otherwise the next slave in the normal round-robin), and
    // advances the round-robin position for the next call.
    void loop()   override;

    // Reports which GPIO pins this module claims, so the module
    // registry can detect pin conflicts with other modules at boot.
    void usedPins(const uint8_t*& pins, uint8_t& count) const override {
        static const uint8_t p[] = {XN_RS485_TX_PIN, XN_RS485_RX_PIN,
                                     XN_RS485_DE_PIN};
        pins = p; count = 3;
    }
    // Reports which PIO state machines this module uses (PIO2, SM0
    // and SM1), so the module registry can detect PIO resource
    // conflicts with other modules at boot.
    uint16_t usedPioSmMask() const override { return 0b11 << 8; } // PIO2 SM0+SM1

    // Queues a broadcast frame (raw XpressNet bytes, without any
    // rufbyte/framing) for transmission on the next available slot
    // in the polling cycle. Called from the EventBus handler
    // (_onEvent) and from the shared XpressNetHandler's broadcast
    // callback (registered in _processSlaveReply()). If the queue is
    // full, the oldest pending broadcast is dropped to make room.
    void enqueueBroadcast(const uint8_t* data, uint8_t len);

    // Returns the previously detected handheld type for the given
    // slave address (UNKNOWN if the address is out of range or has
    // not been classified yet).
    XNSlaveType getSlaveType(uint8_t addr) const {
        if (addr == 0 || addr > XN_RS485_MAX_SLAVES) return XNSlaveType::UNKNOWN;
        return _slaves[addr].type;
    }

    // Immediately transmits a single broadcast frame on the bus,
    // bypassing the queue (used when a broadcast must go out right
    // away, e.g. right after handling a command that itself produced
    // a BROADCAST_ONLY result — see _processSlaveReply()).
    void sendBroadcastNow(const uint8_t* data, uint8_t len);

    // Returns true if at least one slave address currently has
    // present==true (i.e. at least one handheld has answered a poll
    // recently). Used to decide whether the bus is "in use" at all.
    bool hasSlavesPresent() const;

private:
    XNSlave     _slaves[XN_RS485_MAX_SLAVES + 1];
    uint8_t     _currentSlave = 1;
    XNBroadcast _bcQueue[XN_BC_QUEUE_SIZE];
    uint8_t     _rxBuf[XN_RS485_RX_BUF];
    bool        _progModeActive = false;
    uint8_t     _progModeSlave  = 0;
    uint        _txProgOffset;
    uint        _rxProgOffset;

    // Loads and configures the PIO2 SM0 9-bit UART TX program:
    // configures the side-set/output pin, the shift direction, and
    // the clock divider needed to reach 62500 baud (8 PIO cycles per
    // bit).
    void _initPioTx();

    // Loads and configures the PIO2 SM1 9-bit UART RX program:
    // configures the input pin (with hysteresis enabled for better
    // noise immunity on the shared RS-485 bus), shift direction, and
    // the same clock divider as the TX program so both run at
    // 62500 baud.
    void _initPioRx();

    // DE (driver-enable) direction control for the RS-485 transceiver.
    //
    // _txMode(): switches the transceiver to transmit mode (DE=HIGH)
    // and waits XN_RS485_TX_SETTLE microseconds for the line driver
    // to settle before any data is actually shifted out.
    inline void _txMode() {
        // DE high = transmit
        digitalWrite(XN_RS485_DE_PIN, HIGH);
        delayMicroseconds(XN_RS485_TX_SETTLE);
    }
    // _rxMode(): waits until the TX PIO FIFO is fully empty and the
    // last byte has finished shifting out (including its stop bit),
    // then switches the transceiver back to receive mode (DE=LOW) so
    // a slave device can answer.
    inline void _rxMode() {
        // Wait until the TX FIFO is empty
        while (!pio_sm_is_tx_fifo_empty(pio2, 0)) tight_loop_contents();
        // Wait until the shift register has finished (last byte still in the SR)
        delayMicroseconds(XN_BYTE_US + 20);
        // DE low — stop bit has been fully sent
        digitalWrite(XN_RS485_DE_PIN, LOW);
    }

    // ── PIO TX/RX primitives ──────────────────────────────

    // Pushes one 9-bit word (bit8 = address/data marker, bits7-0 =
    // payload byte) into the TX state machine's FIFO, blocking until
    // there is room if the FIFO is currently full.
    void    _send9(uint16_t word9);

    // Sends a single "address byte" (bit8=1), used for rufbytes
    // (poll/call bytes) that address a specific slave or the
    // broadcast address.
    void    _sendAddr(uint8_t byte);

    // Sends a sequence of "data bytes" (bit8=0) — the actual
    // XpressNet payload bytes that follow an address byte.
    void    _sendData(const uint8_t* data, uint8_t len);

    // Reads one raw 32-bit word from the RX state machine's FIFO, or
    // returns -1 if no word is currently available. The 9-bit
    // address/data marker + payload byte are still packed into the
    // upper bits of the returned word and must be unpacked by the
    // caller (see _receiveReply()).
    int32_t _read9();

    // Waits for and assembles one complete incoming XpressNet reply
    // frame from the RX FIFO. Waits up to timeoutUs microseconds for
    // the first (header) byte to arrive; once it has arrived, keeps
    // reading bytes (ignoring any that turn out to be address-marked,
    // i.e. bit8=1) until the frame length implied by the header's
    // low nibble has been reached, or until more than 300µs elapse
    // between two consecutive bytes (inter-byte timeout, treated as
    // "reply aborted"). Returns the number of bytes received into
    // _rxBuf (0 if nothing arrived before the initial timeout).
    uint8_t _receiveReply(uint32_t timeoutUs = XN_RS485_REPLY_US);

    // ── Poll cycle ──────────────────────────────────────────

    // Polls exactly one slave address: sends a "Normale Anfrage"
    // (normal inquiry) rufbyte for that address, switches the
    // transceiver to receive, waits for a reply, verifies its
    // checksum, and — if valid — hands it off to _processSlaveReply()
    // for further handling. Updates the slave's present/missCount
    // bookkeeping based on whether a valid reply was received. Always
    // waits XN_INTER_POLL_US after finishing, to leave a settling gap
    // before the next poll.
    void    _pollSlave(uint8_t addr);

    // Processes one validated reply frame received from a slave:
    // tracks entry/exit of programming mode based on the frame's
    // header bytes, forwards the frame to the shared XpressNetHandler
    // for protocol-level handling (turnouts, loco commands, queries,
    // ...), and — depending on the handler's result — either
    // transmits a direct reply back to the slave (addressed
    // specifically to it via the broadcast/reply rufbyte 0x60+addr)
    // or immediately flushes any queued broadcasts (for results that
    // only produce a broadcast, so time-sensitive devices such as a
    // MultiMaus see the update within their own response timeout).
    void    _processSlaveReply(uint8_t addr, const uint8_t* buf, uint8_t len);

    // ── Broadcasts ──────────────────────────────────────────

    // Sends every currently pending broadcast in the queue (in FIFO
    // order), clearing each entry's pending flag as it is sent. Called
    // at the start of every loop() iteration, and also directly after
    // handling a command whose result is BROADCAST_ONLY.
    void    _flushBroadcasts();

    // Transmits one broadcast frame on the bus: switches to transmit
    // mode, sends the broadcast rufbyte (0x60) followed by the
    // payload bytes, optionally repeating the whole frame `repeat`
    // times for reliability, then switches back to receive mode.
    void    _sendBroadcast(const uint8_t* data, uint8_t len, uint8_t repeat);

    // ── Helpers ─────────────────────────────────────────────

    // Decides which slave address to poll next, implementing the
    // discovery strategy described in the class-level comment above:
    // every 8th call performs a "discovery" poll of the next address
    // that has never answered (or has missed fewer than 3 polls),
    // looking for newly connected devices; all other calls simply
    // advance to the next address that is currently marked present.
    // If no slave is currently present at all, falls back to
    // discovery every time.
    uint8_t _nextSlave(uint8_t current) const;

    // Builds an XpressNet "rufbyte" (call byte) for the given type
    // (Normale Anfrage / Quittierung / Broadcast) and address,
    // including the even-parity bit (bit7) required by the XpressNet
    // RS-485 physical layer.
    static uint8_t _makeRufbyte(uint8_t rufType, uint8_t addr);

    // Verifies the XOR checksum of a received frame (the last byte
    // must equal the XOR of all preceding bytes). Returns false if
    // the buffer is too short to contain a checksum at all.
    static bool    _checkXor(const uint8_t* buf, uint8_t len);

    // EventBus callback: receives events published by other modules
    // (running on either core) and translates the relevant ones
    // (power on/off, emergency stop, feedback) into XpressNet
    // broadcast frames, which are then queued via enqueueBroadcast().
    // Ignores events that originated from this same module, to avoid
    // broadcasting our own commands back onto the bus.
    static void _onEvent(const Event& ev);
};

extern XpressNetRs485 gXpressNetRs485;
