#pragma once
// ═══════════════════════════════════════════════════════════════
//  slot_server.h  —  LocoNet slot server (120 slots)
//
//  Implements the LocoNet slot management protocol per the Digitrax
//  LocoNet Personal Edition specification. Manages up to 120 active
//  loco slots and handles all slot-related LocoNet OPC messages.
//
//  Slot lifecycle:
//    FREE   → (OPC_LOCO_ADR assigns slot) → COMMON
//    COMMON → (throttle takes control)    → IN_USE
//    IN_USE → (timeout or release)        → IDLE
//    IDLE   → (dispatch or purge)         → FREE
//
//  Integration with TMC architecture:
//    - Reads loco state from LocoRepository
//    - Dispatches speed/function changes via CommandBus
//    - Publishes loco state changes via EventBus
// ═══════════════════════════════════════════════════════════════

#include "loco_repository.h"
#include "command_bus.h"
#include "event_bus.h"
#include "module_arch.h"
#include "LocoNet2.h"

// ─────────────────────────────────────────────────────────────
//  TRK status byte bit flags (per LocoNet spec)
// ─────────────────────────────────────────────────────────────
constexpr uint8_t TRK_POWER_ON  = 0x02;  // Track power is on
constexpr uint8_t TRK_PROG_BUSY = 0x08;  // Programming track busy
constexpr uint8_t TRK_MLOK1     = 0x40;  // Master LocoNet 1 flag
constexpr uint8_t TRK_NORMAL    = 0x07;  // Normal operation bitmask

// TX queue capacity (number of LocoNet messages)
constexpr uint8_t LN_TX_QUEUE_SIZE = 8;

class SlotServer
{
public:
    // Constructor — takes references to all shared subsystems.
    SlotServer(LocoNetDispatcher& dispatcher,
               LocoRepository&    repo,
               CommandBus&        cmdBus,
               EventBus&          evtBus);

    // Processes all registered OPC handlers and flushes the TX queue.
    // Call from LocoNetModule::loop() on every iteration.
    void process(LocoNetDispatcher& dispatcher);

    // Runs slot timeout detection (call every 100ms).
    // Releases slots that have been idle for LN_SLOT_TIMEOUT_MS.
    void process100ms(LocoNetDispatcher& dispatcher);

    // Returns the current track status byte
    uint8_t trkStatus() const       { return _trkStatus; }

    // Sets the track status byte (used by LocoNetModule on power changes)
    void    setTrkStatus(uint8_t t) { _trkStatus = t; }

    // Queues a LocoNet message for transmission (public for LocoNetModule)
    void queueMsg(const LnMsg& msg) { _queueMsg(msg); }

    // EventBus handler — see this constructor's own registration
    // comment and onEvent()'s own comment in slot_server.cpp for the
    // full rationale (relays a genuine OPC_SW_REQ onto the physical
    // LocoNet bus for a turnout command originating from another
    // protocol module, e.g. XpressNet or Z21).
    void onEvent(const Event& ev);

    // Puts a specific loco address into the same "released, ready to
    // be picked up" (dispatch pending) state a genuine LocoNet
    // "dispatch put" (OPC_MOVE_SLOTS <slot> 0) would — triggered from
    // the web interface instead of from an existing throttle's slot.
    // See this method's own comment in slot_server.cpp for the full
    // rationale (Rob: a Fred with no numeric keypad at all — only
    // Stop/Shift/F0-F8 — cannot itself select an address to dispatch,
    // but can still pick up whatever is already pending via its own,
    // simple "acquire dispatched slot" mechanism). Returns false if no
    // free slot could be found/created for this address.
    bool dispatchByAddress(uint16_t addr);

private:
    LocoNetDispatcher& _dispatcher;  // stored so dispatchByAddress() can
                                      // call _sendSlotData() from a web-
                                      // triggered context with no LocoNet
                                      // message/dispatcher of its own —
                                      // _sendSlotData()'s own dispatcher
                                      // parameter is otherwise unused
    LocoRepository& _repo;
    CommandBus&     _cmdBus;
    EventBus&       _evtBus;

    uint8_t _trkStatus = TRK_NORMAL & ~TRK_POWER_ON;  // starts reporting power-off

    // Echo-detection for the OPC_SW_REQ onEvent() sends onto the
    // physical LocoNet bus (see onEvent()'s own comment) — LocoNet is
    // a shared, single-wire bus, so a message this device transmits is
    // also received back by this same device's own PHY, exactly like
    // any other device's message would be. Without this check, that
    // echo reaches _handleSwReq() indistinguishable from a genuine,
    // new request from another device, which would re-dispatch to
    // DccHal (again publishing ACCESSORY_STATE, sourceId=DCC_HAL, since
    // DccHal always publishes as itself regardless of which protocol
    // originated the command) — which onEvent() would react to again,
    // sending another OPC_SW_REQ, which echoes back again... an
    // infinite loop (confirmed: Rob, "de wissel commando... blijft
    // komen" after this relay was first added). _lastSentSw1/2 record
    // the exact bytes of the most recently self-sent OPC_SW_REQ;
    // _handleSwReq() compares an incoming one against these before
    // deciding whether to process it further, and clears them once
    // matched so a later, genuine, byte-identical request from another
    // device (e.g. the same turnout commanded again) is not mistaken
    // for a stale echo.
    bool     _awaitingOwnSwReqEcho = false;
    uint8_t  _lastSentSw1 = 0;
    uint8_t  _lastSentSw2 = 0;
    uint32_t _lastSentSwReqMs = 0;  // millis() at send — see _handleSwReq()'s
                                     // own comment for why a timeout is needed
                                     // here, not just a byte match

    // Records which slot was most recently released via a genuine
    // "dispatch put" (OPC_MOVE_SLOTS <slot> 0) — see _handleMoveSlots()'s
    // own comment on "DISPATCH GET" for why this is needed: per the
    // standard LocoNet convention, "dispatch get" (OPC_MOVE_SLOTS 0 0)
    // must hand back specifically the loco that was just dispatched,
    // not an arbitrary free/common slot (Rob: e.g. a Fred whose own
    // "assign" button combination is unknown, but which can still
    // release/dispatch its current loco — another throttle, or the
    // same Fred via whatever picks up a dispatched slot, must then be
    // able to reliably re-acquire that SAME loco, not a different one
    // that happened to already be common for an unrelated reason).
    // LN_NO_SLOT means "nothing currently dispatched".
    uint8_t _lastDispatchedSlot = LN_NO_SLOT;

    // Lock-free TX queue
    LnMsg   _txQueue[LN_TX_QUEUE_SIZE];
    uint8_t _txHead = 0;
    uint8_t _txTail = 0;

    // Enqueue a message for transmission
    void _queueMsg(const LnMsg& msg);

    // Flush all queued messages to the LocoNet bus
    void _flushQueue(LocoNetDispatcher& dispatcher);

    // ── OPC (opcode) handler functions ───────────────────────
    // Each handler processes one type of incoming LocoNet message.

    void _handleLocoAdr    (LocoNetDispatcher&, const LnMsg*);  // OPC_LOCO_ADR — request slot for address
    void _handleMoveSlots  (LocoNetDispatcher&, const LnMsg*);  // OPC_MOVE_SLOTS — move/dispatch slot
    // Shared "dispatch get" logic (hand back the slot most recently
    // released via dispatch put — see _lastDispatchedSlot's own
    // comment) — used both by _handleMoveSlots()'s own OPC_MOVE_SLOTS
    // <0> <0> case and by _handleLocoAdr() when addr==0 (some
    // throttles, e.g. Rob's own Fred, use that as their own,
    // alternative dispatch-get convention).
    void _handleDispatchGet(LocoNetDispatcher&);
    void _handleRqSlotData (LocoNetDispatcher&, const LnMsg*);  // OPC_RQ_SL_DATA — request slot data
    void _handleWrSlotData (LocoNetDispatcher&, const LnMsg*);  // OPC_WR_SL_DATA — write slot data
    void _handleSlotStat1  (LocoNetDispatcher&, const LnMsg*);  // OPC_SLOT_STAT1 — set slot status
    void _handleLocoSpd    (LocoNetDispatcher&, const LnMsg*);  // OPC_LOCO_SPD — set speed
    void _handleLocoDirf   (LocoNetDispatcher&, const LnMsg*);  // OPC_LOCO_DIRF — set direction and F0-F4
    void _handleLocoSnd    (LocoNetDispatcher&, const LnMsg*);  // OPC_LOCO_SND — set F5-F8
    void _handleConsistFunc(LocoNetDispatcher&, const LnMsg*);  // OPC_CONSIST_FUNC — consist function
    void _handleLinkSlots  (LocoNetDispatcher&, const LnMsg*);  // OPC_LINK_SLOTS — link two slots
    void _handleUnlinkSlots(LocoNetDispatcher&, const LnMsg*);  // OPC_UNLINK_SLOTS — unlink slots
    void _handleSwReq      (LocoNetDispatcher&, const LnMsg*);  // OPC_SW_REQ — switch request
    void _handleSwState    (LocoNetDispatcher&, const LnMsg*);  // OPC_SW_STATE — switch state query
    void _handleSwAck      (LocoNetDispatcher&, const LnMsg*);  // OPC_SW_ACK — switch acknowledge
    void _handleGpOn       (LocoNetDispatcher&, const LnMsg*);  // OPC_GPON — global power on
    void _handleLocoF9F12  (LocoNetDispatcher&, const LnMsg*);  // OPC_LOCO_F9F12 — set F9-F12
    void _handleGpOff      (LocoNetDispatcher&, const LnMsg*);  // OPC_GPOFF — global power off
    void _handleIdle       (LocoNetDispatcher&, const LnMsg*);  // OPC_IDLE — emergency stop
    void _handleInputRep   (LocoNetDispatcher&, const LnMsg*);  // OPC_INPUT_REP — general sensor input report

    // ── LocoNet sensor feedback translation ──────────────────
    //
    // OPC_INPUT_REP reports one sensor's state per message (an 11-bit
    // address, plus a single high/low bit) — but the shared feedback
    // event format (Ev::feedback(), used by RS-Bus) reports a whole
    // 4-bit nibble at once (module/nibble/4-bit value), since that is
    // how RS-Bus itself groups its own feedback bits. To publish a
    // correct nibble value from a single-sensor LocoNet update, the
    // last-known state of every LocoNet sensor is cached here as a
    // bitmap (2048 sensors / 8 bits per byte = 256 bytes) — the newly
    // reported bit is written into this cache, and the surrounding
    // 3 bits of its nibble are read back out of it to build the
    // complete 4-bit value passed to Ev::feedback().
    //
    // The 11-bit address itself is computed as high*256 + low*2 + I,
    // where I (IN2 bit 5) is the LEAST SIGNIFICANT address bit, not a
    // separate type flag to discard — see _handleInputRep()'s own
    // comment in slot_server.cpp for the full rationale (an earlier
    // version got this wrong, confirmed against Rob's own log, which
    // collapsed two distinct sensors sharing the same IN1 byte onto a
    // single address).
    //
    // Address translation (agreed with Rob): the resulting 0-based
    // LocoNet sensor address N maps onto RS-Bus-style
    // module=(N/8)+1, nibble=(N%8)/4, bit=N%4 — i.e. LocoNet sensors
    // 0-7 (seen by users as 1-8) report as RS-Bus module 1, 8-15
    // (seen as 9-16) as module 2, and so on. This was a deliberate
    // choice (no existing convention for what a genuine RS-Bus
    // module's address "should" be for a LocoNet sensor) rather than
    // a value derived from the protocol itself.
    uint8_t _lnSensorState[256] = {};

    // ── Slot management ──────────────────────────────────────

    // Assigns a slot to a loco address, or returns existing slot number.
    // Creates a new LocoRepository entry if needed.
    uint8_t _assignSlot(uint16_t addr, uint8_t id1, uint8_t id2);

    // Returns the index of the first free (inactive) slot, or 0xFF if full.
    uint8_t _findFreeSlot() const;

    // ── Transmit helpers ──────────────────────────────────────

    // Sends an SL_RD_DATA response for the given slot number
    void _sendSlotData(LocoNetDispatcher&, uint8_t slot);

    // Sends an SL_RD_DATA response for the given loco entry
    void _sendSlotData(LocoNetDispatcher&, const LocoBase* loco);

    // Fills an LnMsg with SL_RD_DATA for a loco entry
    void _fillSlotMsg(LnMsg& msg, const LocoBase* loco) const;
    // Builds a genuine "FREE" OPC_SL_RD_DATA for a valid-but-unused
    // slot — see this method's own comment in slot_server.cpp for why
    // this replaced sending a failure LACK for that case.
    void _fillEmptySlotMsg(LnMsg& msg, uint8_t slot) const;

    // Sends a long acknowledgement (LACK) response
    void _sendLack(LocoNetDispatcher&, uint8_t replyToOpc, uint8_t ack);

    // ── LocoRepository write helpers ──────────────────────────

    // Write speed to loco and mark dirty for DCC resend
    void _writeSpeed(LocoBase* loco, uint8_t spd);

    // Write direction and F0-F4 to loco and mark dirty
    void _writeDirf(LocoBase* loco, uint8_t dirf);

    // Write F5-F8 sound functions to loco and mark dirty
    void _writeSnd(LocoBase* loco, uint8_t snd);

    // Mark both DCC and LocoNet dirty flags for a loco
    void _markBothDirty(LocoBase* loco);

    // ── EventBus publish helpers ──────────────────────────────

    // Publish a LOCO_STATE event for the given loco
    void _publishLocoEvent(const LocoBase* loco);

    // Publish a POWER_ON or POWER_OFF event
    void _publishPowerEvent(bool on);

    // Publish an ACCESSORY_STATE event for a turnout change
    void _publishAccessoryEvent(uint16_t addr, bool output, bool active);
};
