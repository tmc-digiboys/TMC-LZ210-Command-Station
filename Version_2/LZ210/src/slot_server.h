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

private:
    LocoRepository& _repo;
    CommandBus&     _cmdBus;
    EventBus&       _evtBus;

    uint8_t _trkStatus = TRK_NORMAL & ~TRK_POWER_ON;  // starts reporting power-off

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
