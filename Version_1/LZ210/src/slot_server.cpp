// ═══════════════════════════════════════════════════════════════
//  slot_server.cpp
//
//  Corrections relative to the first version:
//    - MODULE_ID_LOCONET → ModuleId::LOCONET_HAL
//    - EventType → EvType  (per module_arch.h)
//    - ev.active → ev.state  (per Event struct)
//    - extension field removed from LocoBase (does not exist)
//    - _savedIrq removed (lockRead uses spin_lock_blocking)
// ═══════════════════════════════════════════════════════════════

#include "slot_server.h"
#include "xpressnet_handler.h"  // for gCentrale.shortCircuit
#include <Arduino.h>
#include <cstring>

// ── Constructor ──────────────────────────────────────────────

// SlotServer() — stores references to the shared LocoRepository,
// CommandBus and EventBus, and registers a lambda handler for every
// LocoNet opcode this class cares about with the given
// LocoNetDispatcher, each simply forwarding to the corresponding
// private _handle*() method. This wires up the full OPC dispatch
// table in one place at construction time, rather than requiring the
// dispatcher itself to know about SlotServer's internals.
SlotServer::SlotServer(LocoNetDispatcher& dispatcher,
                       LocoRepository&    repo,
                       CommandBus&        cmdBus,
                       EventBus&          evtBus)
    : _repo(repo), _cmdBus(cmdBus), _evtBus(evtBus)
{
    dispatcher.onPacket(OPC_LOCO_ADR,     [this,&dispatcher](const LnMsg* p){ _handleLocoAdr    (dispatcher,p); });
    dispatcher.onPacket(OPC_MOVE_SLOTS,   [this,&dispatcher](const LnMsg* p){ _handleMoveSlots  (dispatcher,p); });
    dispatcher.onPacket(OPC_RQ_SL_DATA,   [this,&dispatcher](const LnMsg* p){ _handleRqSlotData (dispatcher,p); });
    dispatcher.onPacket(OPC_WR_SL_DATA,   [this,&dispatcher](const LnMsg* p){ _handleWrSlotData (dispatcher,p); });
    dispatcher.onPacket(OPC_SLOT_STAT1,   [this,&dispatcher](const LnMsg* p){ _handleSlotStat1  (dispatcher,p); });
    dispatcher.onPacket(OPC_LOCO_SPD,     [this,&dispatcher](const LnMsg* p){ _handleLocoSpd    (dispatcher,p); });
    dispatcher.onPacket(OPC_LOCO_DIRF,    [this,&dispatcher](const LnMsg* p){ _handleLocoDirf   (dispatcher,p); });
    dispatcher.onPacket(OPC_LOCO_SND,     [this,&dispatcher](const LnMsg* p){ _handleLocoSnd    (dispatcher,p); });
    dispatcher.onPacket(OPC_CONSIST_FUNC, [this,&dispatcher](const LnMsg* p){ _handleConsistFunc(dispatcher,p); });
    dispatcher.onPacket(OPC_LINK_SLOTS,   [this,&dispatcher](const LnMsg* p){ _handleLinkSlots  (dispatcher,p); });
    dispatcher.onPacket(OPC_UNLINK_SLOTS, [this,&dispatcher](const LnMsg* p){ _handleUnlinkSlots(dispatcher,p); });
    dispatcher.onPacket(OPC_SW_REQ,       [this,&dispatcher](const LnMsg* p){ _handleSwReq      (dispatcher,p); });
    dispatcher.onPacket(OPC_SW_STATE,     [this,&dispatcher](const LnMsg* p){ _handleSwState    (dispatcher,p); });
    dispatcher.onPacket(OPC_SW_ACK,       [this,&dispatcher](const LnMsg* p){ _handleSwAck      (dispatcher,p); });
    dispatcher.onPacket(OPC_GPON,         [this,&dispatcher](const LnMsg* p){ _handleGpOn       (dispatcher,p); });
    // OPC_A3: F9-F12 (Uhlenbrock/Intellibox extension, also used by FRED)
    dispatcher.onPacket(0xA3,             [this,&dispatcher](const LnMsg* p){ _handleLocoF9F12  (dispatcher,p); });
    dispatcher.onPacket(OPC_GPOFF,        [this,&dispatcher](const LnMsg* p){ _handleGpOff      (dispatcher,p); });
    dispatcher.onPacket(OPC_IDLE,         [this,&dispatcher](const LnMsg* p){ _handleIdle       (dispatcher,p); });
}

// ═══════════════════════════════════════════════════════════════
//  TX Queue — messages are sent outside of OPC callbacks so that
//  the CD backoff timer has a chance to elapse before transmission
// ═══════════════════════════════════════════════════════════════

// _queueMsg() — appends one message to the ring-style TX queue.
// Silently drops the message if the queue is already full (the next
// write index would collide with _txHead) rather than blocking or
// overwriting an existing entry.
void SlotServer::_queueMsg(const LnMsg& msg)
{
    uint8_t next = (_txTail + 1) % LN_TX_QUEUE_SIZE;
    if (next != _txHead) {  // not full
        _txQueue[_txTail] = msg;
        _txTail = next;
    }
}

// _flushQueue() — attempts to send every currently queued message, in
// FIFO order, via the dispatcher. Stops as soon as a send attempt
// does not return LN_IDLE (meaning the bus is still busy / a
// collision was detected), leaving the remaining queued messages in
// place to be retried on a later call. Only advances _txHead past a
// message once it has actually been accepted for transmission.
void SlotServer::_flushQueue(LocoNetDispatcher& dispatcher)
{
    while (_txHead != _txTail) {
        LnMsg& msg = _txQueue[_txHead];
        LN_STATUS s = dispatcher.send(&msg);
        if (s != LN_IDLE) break;  // bus still busy, try again next loop
        _txHead = (_txHead + 1) % LN_TX_QUEUE_SIZE;
    }
}

// ── process() ────────────────────────────────────────────────

// process() — called once per LocoNetModule::loop() iteration. First
// drains the TX queue (outside of any OPC callback, so any CD backoff
// delay has had a chance to elapse). Then, under the repository write
// lock, repeatedly pops the next LocoNet-dirty loco (nextLnDirty())
// and — provided it currently has a real slot assignment and is not
// FREE — releases the lock temporarily to send its updated
// SL_RD_DATA (_sendSlotData(), which itself takes its own read lock),
// then re-acquires the write lock to continue the loop. This
// unlock/resend/relock pattern avoids holding the repository lock
// across the (comparatively slow) act of queuing a LocoNet
// transmission. Returns early (without unlocking a lock it does not
// hold) if either lock attempt times out.
void SlotServer::process(LocoNetDispatcher& dispatcher)
{
    // First drain the TX queue — outside callbacks so CD backoff has elapsed
    _flushQueue(dispatcher);

    if (!_repo.lockWrite(200)) return;
    LocoBase* loco;
    while ((loco = _repo.nextLnDirty()) != nullptr) {
        if (loco->ln.slot != LN_NO_SLOT && loco->ln.stat != LN_STAT_FREE) {
            _repo.unlock();
            _sendSlotData(dispatcher, loco);
            if (!_repo.lockWrite(200)) return;
        }
    }
    _repo.unlock();
}

// ── process100ms() ───────────────────────────────────────────

// process100ms() — called roughly every 100ms (see LocoNetModule::
// loop()). Under the repository write lock, scans every active loco
// that currently holds a real LocoNet slot in the IN_USE state, and —
// if more than LN_SLOT_TIMEOUT_MS has elapsed since that slot's last
// recorded throttle activity — demotes it to IDLE and marks it
// LocoNet-dirty (so the next process() call broadcasts the updated
// SL_RD_DATA, informing any throttle that the slot has gone idle).
// After releasing the lock, calls process() to actually send out any
// resulting dirty-slot broadcasts (and to drain the TX queue) in the
// same cycle, rather than waiting for the next natural process() call.
void SlotServer::process100ms(LocoNetDispatcher& dispatcher)
{
    uint32_t now = millis();
    if (!_repo.lockWrite(500)) return;
    for (uint8_t i = 0; i < _repo.count(); i++) {
        LocoBase* l = const_cast<LocoBase*>(_repo.at(i));
        if (!l || !l->active) continue;
        if (l->ln.slot == LN_NO_SLOT) continue;
        if (l->ln.stat != LN_STAT_IN_USE) continue;
        if ((now - l->ln.lastActivityMs) > LN_SLOT_TIMEOUT_MS) {
            l->ln.stat  = LN_STAT_IDLE;
            l->ln.dirty = true;
        }
    }
    _repo.unlock();
    process(dispatcher);
}

// ═══════════════════════════════════════════════════════════════
//  OPC_LOCO_ADR (0xBF) — a throttle requests a slot for a DCC address
// ═══════════════════════════════════════════════════════════════

// _handleLocoAdr() — decodes the 14-bit DCC address from the
// message's adr_hi/adr_lo fields, and either reuses an existing slot
// for that address or allocates a new one via _assignSlot(). If no
// slot could be assigned (repository full), replies with a LACK
// (ack=0, meaning failure) instead. On success, replies with the
// full SL_RD_DATA for the assigned slot, so the requesting throttle
// learns the slot number and can proceed to claim it.
void SlotServer::_handleLocoAdr(LocoNetDispatcher& dispatcher,
                                 const LnMsg* pkt)
{
    uint16_t addr = ((uint16_t)pkt->la.adr_hi << 7) | pkt->la.adr_lo;
    uint8_t slot = _assignSlot(addr, 0, 0);
    if (slot == LN_NO_SLOT) {
        _sendLack(dispatcher, OPC_LOCO_ADR & 0x7F, 0);
        return;
    }

    _sendSlotData(dispatcher, slot);
}

// ═══════════════════════════════════════════════════════════════
//  OPC_MOVE_SLOTS (0xBA) — slot claim/dispatch/move operations
// ═══════════════════════════════════════════════════════════════

// _handleMoveSlots() — implements the four distinct operations that
// share the OPC_MOVE_SLOTS opcode, distinguished by the src/dst slot
// numbers in the message:
//
//   NULL MOVE (src==dst, src!=0): the throttle is claiming a slot it
//   already knows about (typically right after receiving SL_RD_DATA
//   from OPC_LOCO_ADR). Marks that slot IN_USE, refreshes its last-
//   activity timestamp, marks it dirty, and replies with the updated
//   SL_RD_DATA — or a failure LACK if no loco currently holds that
//   slot.
//
//   DISPATCH PUT (dst==0, src!=0): the throttle releases a slot back
//   to the dispatch pool. Demotes the slot from IN_USE/whatever it
//   was to COMMON, clears its throttle ID bytes, marks it dirty, and
//   replies with the updated SL_RD_DATA (or a failure LACK if the
//   slot was not found).
//
//   DISPATCH GET (src==0, dst==0): the throttle asks "give me any
//   available COMMON slot" — scans the repository (under a read
//   lock) for the first active loco whose slot is in the COMMON
//   state and replies with its SL_RD_DATA, or a failure LACK if none
//   is currently available.
//
//   Normal MOVE (any other src/dst combination): copies the dynamic
//   state (speed, direction, functions, LocoNet status/throttle ID)
//   from the src slot's loco entry to the dst slot's loco entry,
//   marks the destination dirty, and frees up the source slot
//   (FREE/LN_NO_SLOT). Always replies with SL_RD_DATA for the
//   destination slot, even if the copy could not actually be
//   performed (e.g. either slot not found) — matching the observed
//   behaviour expected by real LocoNet throttles for this less
//   common operation.
// ═══════════════════════════════════════════════════════════════
void SlotServer::_handleMoveSlots(LocoNetDispatcher& dispatcher,
                                   const LnMsg* pkt)
{
    uint8_t src = pkt->sm.src;
    uint8_t dst = pkt->sm.dest;

    // NULL MOVE: src==dst, src!=0 → activate slot
    if (src == dst && src != 0) {
        if (!_repo.lockWrite(500)) return;
        LocoBase* loco = _repo.findBySlot(src);
        if (loco) {
            loco->ln.stat           = LN_STAT_IN_USE;
            loco->ln.lastActivityMs = millis();
            loco->ln.dirty          = true;
        }
        _repo.unlock();
        if (loco) _sendSlotData(dispatcher, src);
        else      _sendLack(dispatcher, OPC_MOVE_SLOTS & 0x7F, 0);
        return;
    }

    // DISPATCH PUT: dst==0, src!=0 → release the slot
    if (dst == 0 && src != 0) {
        if (!_repo.lockWrite(500)) return;
        LocoBase* loco = _repo.findBySlot(src);
        if (loco) {
            loco->ln.stat  = LN_STAT_COMMON;
            loco->ln.id1   = 0;
            loco->ln.id2   = 0;
            loco->ln.dirty = true;
        }
        _repo.unlock();
        if (loco) _sendSlotData(dispatcher, src);
        else      _sendLack(dispatcher, OPC_MOVE_SLOTS & 0x7F, 0);
        return;
    }

    // DISPATCH GET: src==0, dst==0 → grab the first COMMON slot
    if (src == 0 && dst == 0) {
        if (!_repo.lockRead(500)) return;
        uint8_t found = LN_NO_SLOT;
        for (uint8_t i = 0; i < _repo.count(); i++) {
            const LocoBase* l = _repo.at(i);
            if (l && l->active && l->ln.stat == LN_STAT_COMMON) {
                found = l->ln.slot; break;
            }
        }
        _repo.unlock();
        if (found != LN_NO_SLOT) _sendSlotData(dispatcher, found);
        else                      _sendLack(dispatcher, OPC_MOVE_SLOTS & 0x7F, 0);
        return;
    }

    // Normal MOVE: copy src → dst
    if (!_repo.lockWrite(500)) return;
    LocoBase* srcL = _repo.findBySlot(src);
    LocoBase* dstL = _repo.findBySlot(dst);
    if (srcL && dstL) {
        dstL->speed     = srcL->speed;
        dstL->forward   = srcL->forward;
        dstL->functions = srcL->functions;
        dstL->ln.stat   = srcL->ln.stat;
        dstL->ln.id1    = srcL->ln.id1;
        dstL->ln.id2    = srcL->ln.id2;
        dstL->ln.dirty  = true;
        srcL->ln.stat   = LN_STAT_FREE;
        srcL->ln.slot   = LN_NO_SLOT;
        srcL->ln.dirty  = true;
    }
    _repo.unlock();
    _sendSlotData(dispatcher, dst);
}

// ═══════════════════════════════════════════════════════════════
//  OPC_RQ_SL_DATA (0xBB) — request slot data
// ═══════════════════════════════════════════════════════════════

// _handleRqSlotData() — validates the requested slot number (must be
// 1-120; slot 0 and anything above 120 is silently ignored), and
// replies with that slot's current SL_RD_DATA via _sendSlotData().
void SlotServer::_handleRqSlotData(LocoNetDispatcher& dispatcher,
                                    const LnMsg* pkt)
{
    uint8_t slot = pkt->sr.slot;
    if (slot == 0 || slot > 120) return;
    _sendSlotData(dispatcher, slot);
}

// ═══════════════════════════════════════════════════════════════
//  OPC_WR_SL_DATA (0xEF) — write slot data (throttle sets step mode
//  and its own throttle ID when first claiming a slot)
// ═══════════════════════════════════════════════════════════════

// _handleWrSlotData() — validates the slot number (rejecting with a
// failure LACK if out of range). Otherwise, under the repository
// write lock, decodes the requested DCC speed-step mode from the
// STAT1 bits (per ln_opc.h's SPDEX/SPD28/SPD14 flags) and stores both
// the LocoNet-native spdMode (used when building future SL_RD_DATA
// replies) and this codebase's own canonical stepMode convention used
// by DccHal::setSpeed() (0=14-step, 2=28-step, 4=128-step — anything
// not explicitly 14 or 28 defaults to 128-step). Also stores the
// throttle's two ID bytes (id1/id2) and marks the slot LocoNet-dirty.
// Always replies with a success LACK (ack=0x7F) regardless of whether
// a matching loco was actually found for this slot.
void SlotServer::_handleWrSlotData(LocoNetDispatcher& dispatcher,
                                    const LnMsg* pkt)
{
    uint8_t slot = pkt->sd.slot;
    if (slot == 0 || slot > 120) {
        _sendLack(dispatcher, OPC_WR_SL_DATA & 0x7F, 0);
        return;
    }
    if (!_repo.lockWrite(500)) return;
    LocoBase* loco = _repo.findBySlot(slot);
    if (loco) {
        // Read the speed step mode from the STAT1 bits per ln_opc.h
        uint8_t spdbits = pkt->sd.stat & 0x07;
        if (spdbits & STAT1_SL_SPDEX) {
            loco->ln.spdMode = LN_SPD_128;  // 0x04 = 128 steps
            loco->stepMode   = 4;   // per DccHal::setSpeed() convention: 0=14,2=28,otherwise=128
        } else if (spdbits & STAT1_SL_SPD28) {
            loco->ln.spdMode = LN_SPD_28;
            loco->stepMode   = 2;
        } else if (spdbits & STAT1_SL_SPD14) {
            loco->ln.spdMode = LN_SPD_14;
            loco->stepMode   = 0;   // was 1 — used to fall into DccHal::setSpeed()'s default branch (128-step!)
        } else {
            loco->ln.spdMode = LN_SPD_128;  // no bits set = also 128-step
            loco->stepMode   = 4;
        }
        loco->ln.id1   = pkt->sd.id1;
        loco->ln.id2   = pkt->sd.id2;
        loco->ln.dirty = true;
        _repo.markDirty(loco->address);
    }
    _repo.unlock();
    _sendLack(dispatcher, OPC_WR_SL_DATA & 0x7F, 0x7F);
}

// ═══════════════════════════════════════════════════════════════
//  OPC_SLOT_STAT1 (0xB5) — set slot status bits
// ═══════════════════════════════════════════════════════════════

// _handleSlotStat1() — under the repository write lock, looks up the
// loco assigned to the given slot and stores the requested status
// bits (masked to the two low bits, matching the FREE/COMMON/IDLE/
// IN_USE state values), marking the slot dirty. If the new status is
// FREE, also clears the slot assignment (LN_NO_SLOT), fully releasing
// it. If a matching loco was found, replies with the resulting
// SL_RD_DATA; otherwise sends no reply at all.
void SlotServer::_handleSlotStat1(LocoNetDispatcher& dispatcher,
                                   const LnMsg* pkt)
{
    uint8_t slot = pkt->ss.slot;
    uint8_t stat = pkt->ss.stat;
    if (!_repo.lockWrite(500)) return;
    LocoBase* loco = _repo.findBySlot(slot);
    if (loco) {
        loco->ln.stat  = stat & 0x03;
        loco->ln.dirty = true;
        if (loco->ln.stat == LN_STAT_FREE) loco->ln.slot = LN_NO_SLOT;
    }
    _repo.unlock();
    if (loco) _sendSlotData(dispatcher, slot);
}

// ═══════════════════════════════════════════════════════════════
//  OPC_LOCO_SPD (0xA0) — throttle sets a loco's speed
// ═══════════════════════════════════════════════════════════════

// _handleLocoSpd() — under the repository write lock, looks up the
// loco assigned to the given slot; if found AND its slot is currently
// IN_USE (i.e. genuinely claimed by a throttle, not just idle/common),
// applies the new speed via _writeSpeed() and publishes a LOCO_STATE
// event so other protocol modules (XpressNet/LAN/Z21) can inform
// their own clients of the change.
//
// NO auto power-on happens here. A LocoNet speed command
// (OPC_LOCO_SPD) must never switch DCC track power on by itself — a
// Fred throttle is a "dumb" device and does not generate a genuine
// OPC_GPON. Track power on/off only ever happens via an explicit user
// action (web/XpressNet/USB) or a genuine OPC_GPON message on the
// bus — see _handleGpOn().
void SlotServer::_handleLocoSpd(LocoNetDispatcher& dispatcher,
                                  const LnMsg* pkt)
{
    uint8_t slot = pkt->lsp.slot;

    if (!_repo.lockWrite(500)) return;
    LocoBase* loco = _repo.findBySlot(slot);
    if (loco && loco->ln.stat == LN_STAT_IN_USE) {
        _writeSpeed(loco, pkt->lsp.spd);
        _publishLocoEvent(loco);
    }
    _repo.unlock();
}

// ═══════════════════════════════════════════════════════════════
//  OPC_LOCO_DIRF (0xA1) — throttle sets direction and F0-F4
// ═══════════════════════════════════════════════════════════════

// _handleLocoDirf() — same guard pattern as _handleLocoSpd(): only
// acts if the slot is found and currently IN_USE. Applies the new
// direction/F0-F4 state via _writeDirf() and publishes a LOCO_STATE
// event.
void SlotServer::_handleLocoDirf(LocoNetDispatcher& dispatcher,
                                   const LnMsg* pkt)
{
    uint8_t slot = pkt->ldf.slot;
    if (!_repo.lockWrite(500)) return;
    LocoBase* loco = _repo.findBySlot(slot);
    if (loco && loco->ln.stat == LN_STAT_IN_USE) {
        _writeDirf(loco, pkt->ldf.dirf);
        _publishLocoEvent(loco);
    }
    _repo.unlock();
}

// ═══════════════════════════════════════════════════════════════
//  OPC_LOCO_SND (0xA2) — throttle sets F5-F8 (sound functions)
// ═══════════════════════════════════════════════════════════════

// _handleLocoSnd() — same guard pattern again: only acts if the slot
// is found and currently IN_USE. Applies the new F5-F8 state via
// _writeSnd() and publishes a LOCO_STATE event.
void SlotServer::_handleLocoSnd(LocoNetDispatcher& dispatcher,
                                  const LnMsg* pkt)
{
    uint8_t slot = pkt->ls.slot;
    if (!_repo.lockWrite(500)) return;
    LocoBase* loco = _repo.findBySlot(slot);
    if (loco && loco->ln.stat == LN_STAT_IN_USE) {
        _writeSnd(loco, pkt->ls.snd);
        _publishLocoEvent(loco);
    }
    _repo.unlock();
}

// ═══════════════════════════════════════════════════════════════
//  OPC_CONSIST_FUNC (0xB6) — set F0-F4 for a consist member
// ═══════════════════════════════════════════════════════════════

// _handleConsistFunc() — looks up the loco assigned to the given slot
// (note: unlike the LOCO_SPD/DIRF/SND handlers above, this does NOT
// require the slot to be IN_USE). Rebuilds F0-F4 (bits 0-4 of
// functions) from the message's dirf-style byte using the same bit
// mapping as _writeDirf() (F0↔bit4, F1↔bit0, F2↔bit1, F3↔bit2,
// F4↔bit3), while leaving every other function bit untouched. Marks
// the loco dirty for both DCC and LocoNet resend, and publishes a
// LOCO_STATE event.
void SlotServer::_handleConsistFunc(LocoNetDispatcher& dispatcher,
                                     const LnMsg* pkt)
{
    uint8_t slot = pkt->data[1];
    uint8_t dirf = pkt->data[2];
    if (!_repo.lockWrite(500)) return;
    LocoBase* loco = _repo.findBySlot(slot);
    if (loco) {
        uint32_t fn = loco->functions & ~0x1F;
        if (dirf & 0x10) fn |= 0x01;
        if (dirf & 0x01) fn |= 0x02;
        if (dirf & 0x02) fn |= 0x04;
        if (dirf & 0x04) fn |= 0x08;
        if (dirf & 0x08) fn |= 0x10;
        loco->functions = fn;
        _markBothDirty(loco);
        _publishLocoEvent(loco);
    }
    _repo.unlock();
}

// ═══════════════════════════════════════════════════════════════
//  LINK/UNLINK SLOTS — stub
// ═══════════════════════════════════════════════════════════════

// _handleLinkSlots() / _handleUnlinkSlots() — minimal stub
// implementations: rather than actually linking/unlinking two slots
// into/out of a consist relationship, they simply reply with the
// current (unmodified) SL_RD_DATA for both the source and destination
// slots, acknowledging the request without performing the underlying
// consist bookkeeping.
void SlotServer::_handleLinkSlots(LocoNetDispatcher& dispatcher,
                                   const LnMsg* pkt)
{
    _sendSlotData(dispatcher, pkt->sm.src);
    _sendSlotData(dispatcher, pkt->sm.dest);
}

void SlotServer::_handleUnlinkSlots(LocoNetDispatcher& dispatcher,
                                     const LnMsg* pkt)
{
    _sendSlotData(dispatcher, pkt->sm.src);
    _sendSlotData(dispatcher, pkt->sm.dest);
}

// ═══════════════════════════════════════════════════════════════
//  Turnout commands
// ═══════════════════════════════════════════════════════════════

// ─────────────────────────────────────────────────────────────
//  _handleSwReq() — OPC_SW_REQ: LocoNet turnout/accessory command
//
//  Decodes the 11-bit turnout output address from sw1/the low nibble
//  of sw2, then converts it from the 0-based wire value to this
//  codebase's 1-based output address convention (addr++).
//
//  LocoNet OPC_SW_REQ SW2 byte: the OUT bit means coil on/off
//  (activate), and the DIR bit means direction (1=normal/straight/
//  closed, 0=thrown/diverging — the same convention as RCN-213's
//  R-bit). These two were previously swapped when passed through to
//  cmd.output/cmd.active — now corrected: cmd.output carries the
//  direction (matching the shared RCN-213 convention used throughout
//  the rest of the codebase: 0=thrown,1=straight), and cmd.active
//  carries the activate flag.
//
//  Dispatches an ACCESSORY_SET command onto the CommandBus, and
//  publishes a matching ACCESSORY_STATE event so other protocol
//  modules can inform their own clients of the change.
// ─────────────────────────────────────────────────────────────
void SlotServer::_handleSwReq(LocoNetDispatcher& dispatcher,
                               const LnMsg* pkt)
{
    uint16_t addr  = ((uint16_t)(pkt->srq.sw2 & 0x0F) << 7) |
                      (pkt->srq.sw1 & 0x7F);
    addr++;
    // LocoNet OPC_SW_REQ SW2: OUT bit = coil on/off (activate),
    // DIR bit = direction (1=normal/straight/closed, 0=thrown/diverging —
    // the same convention as RCN-213's R-bit). These two were previously
    // swapped when passed through to cmd.output/cmd.active.
    bool activate = (pkt->srq.sw2 & OPC_SW_REQ_OUT) != 0;
    bool closed   = (pkt->srq.sw2 & OPC_SW_REQ_DIR) != 0;

    Command cmd{};
    cmd.type     = CmdType::ACCESSORY_SET;
    cmd.sourceId = ModuleId::LOCONET_HAL;
    cmd.address  = addr;
    cmd.output   = closed ? 1 : 0;   // direction (RCN-213: 0=thrown,1=straight)
    cmd.active   = activate;
    _cmdBus.dispatch(cmd);

    _publishAccessoryEvent(addr, closed, activate);
}

// _handleSwState() — OPC_SW_STATE: a query for the current state of a
// turnout. Stubbed out: always replies with a plain success LACK
// (ack=0x7F) without actually reporting any real turnout state.
void SlotServer::_handleSwState(LocoNetDispatcher& dispatcher,
                                 const LnMsg* pkt)
{
    LnMsg reply = makeLongAck(OPC_SW_STATE & 0x7F, 0x7F);
    _queueMsg(reply);
}

// _handleSwAck() — OPC_SW_ACK: functionally identical to a regular
// OPC_SW_REQ (handled by forwarding to _handleSwReq()), but expects
// an explicit acknowledgement in return, which is sent here as a
// success LACK (ack=0x7F).
void SlotServer::_handleSwAck(LocoNetDispatcher& dispatcher,
                               const LnMsg* pkt)
{
    _handleSwReq(dispatcher, pkt);
    _sendLack(dispatcher, OPC_SW_ACK & 0x7F, 0x7F);
}

// ═══════════════════════════════════════════════════════════════
//  Power / emergency stop
// ═══════════════════════════════════════════════════════════════

// ─────────────────────────────────────────────────────────────
//  _handleGpOn() — OPC_GPON: a device on the bus requests global
//  power on
//
//  An external OPC_GPON (e.g. from a Fred throttle) must NOT
//  automatically resume the station if:
//    - gCentrale.shortCircuit:   a short circuit is currently active
//    - gCentrale.trackPowerOff:  the user has deliberately switched
//                                 power off
//  In both cases the user must explicitly issue power-on via an
//  interface (XpressNet/LAN/USB) instead of having it happen
//  implicitly just because some LocoNet device asked for it.
//
//  Otherwise: sets the TRK_POWER_ON bit in the track status byte,
//  dispatches a POWER_ON command onto the CommandBus (so DccHal
//  actually enables track power), and publishes a POWER_ON event so
//  other protocol modules can inform their own clients.
// ─────────────────────────────────────────────────────────────
void SlotServer::_handleGpOn(LocoNetDispatcher& dispatcher,
                              const LnMsg* pkt)
{
    if (gCentrale.shortCircuit || gCentrale.trackPowerOff) {
        return;
    }
    _trkStatus |= TRK_POWER_ON;
    Command cmd{};
    cmd.type = CmdType::POWER_ON;
    cmd.sourceId = ModuleId::LOCONET_HAL;
    _cmdBus.dispatch(cmd);
    _publishPowerEvent(true);
}

// _handleGpOff() — OPC_GPOFF: a device on the bus requests global
// power off. Unlike _handleGpOn(), this is always honoured
// immediately and unconditionally (there is no reason to refuse a
// request to turn power OFF). Clears the TRK_POWER_ON bit, dispatches
// a POWER_OFF command onto the CommandBus, and publishes a POWER_OFF
// event.
void SlotServer::_handleGpOff(LocoNetDispatcher& dispatcher,
                               const LnMsg* pkt)
{
    _trkStatus &= ~TRK_POWER_ON;
    Command cmd{};
    cmd.type = CmdType::POWER_OFF;
    cmd.sourceId = ModuleId::LOCONET_HAL;
    _cmdBus.dispatch(cmd);
    _publishPowerEvent(false);
}

// _handleIdle() — OPC_IDLE: the LocoNet-wide emergency stop message.
// Simply dispatches an EMERGENCY_STOP command onto the CommandBus,
// letting DccHal handle the actual broadcast DCC emergency stop; no
// direct reply is sent and no event is published here (EMERGENCY_STOP
// handling on the DCC side publishes its own event separately).
void SlotServer::_handleIdle(LocoNetDispatcher& dispatcher,
                              const LnMsg* pkt)
{
    Command cmd{};
    cmd.type = CmdType::EMERGENCY_STOP;
    cmd.sourceId = ModuleId::LOCONET_HAL;
    _cmdBus.dispatch(cmd);
}

// ═══════════════════════════════════════════════════════════════
//  Slot management
// ═══════════════════════════════════════════════════════════════

// ─────────────────────────────────────────────────────────────
//  _assignSlot() — find or allocate a LocoNet slot for a DCC address
//
//  Under the repository write lock: first scans for an EXISTING
//  active loco with a real slot already assigned to this address. If
//  found, resets that slot's status back to IDLE (rather than leaving
//  it in whatever state it was) and refreshes its last-activity
//  timestamp — this forces the requesting throttle to perform a fresh
//  NULL MOVE to actually claim it, which is the expected LocoNet
//  behaviour for a throttle re-requesting an address it (or another
//  throttle) already has a slot for. Returns that existing slot
//  number.
//
//  If no existing slot was found, allocates a brand new one:
//  _findFreeSlot() picks the first unused slot number, then
//  LocoRepository::findOrCreate() gets (or creates) the loco entry
//  for this address. Initialises the new slot's LocoNet fields
//  (assigned slot number, IDLE status — available for a throttle to
//  claim — default 128-step speed mode matching this codebase's
//  DccHal::setSpeed() convention, the caller-supplied throttle ID
//  bytes, current timestamp, and dirty=false since nothing needs
//  broadcasting yet for a brand new, not-yet-claimed slot).
//
//  Returns LN_NO_SLOT if either the free-slot search or the
//  repository allocation fails (repository full).
// ─────────────────────────────────────────────────────────────
uint8_t SlotServer::_assignSlot(uint16_t addr, uint8_t id1, uint8_t id2)
{
    if (!_repo.lockWrite(500)) return LN_NO_SLOT;

    // Existing slot? — reset to IDLE so a throttle (e.g. FRED) can do a NULL MOVE
    for (uint8_t i = 0; i < _repo.count(); i++) {
        LocoBase* l = const_cast<LocoBase*>(_repo.at(i));
        if (l && l->active && l->address == addr && l->ln.slot != LN_NO_SLOT) {
            // Reset to IDLE — the throttle must perform a fresh NULL MOVE
            l->ln.stat           = LN_STAT_IDLE;
            l->ln.lastActivityMs = millis();
            uint8_t s = l->ln.slot;
            _repo.unlock();
            return s;
        }
    }

    // New slot
    uint8_t slot = _findFreeSlot();
    if (slot == LN_NO_SLOT) { _repo.unlock(); return LN_NO_SLOT; }

    LocoBase* loco = _repo.findOrCreate(addr);
    if (!loco) { _repo.unlock(); return LN_NO_SLOT; }

    loco->ln.slot           = slot;
    loco->ln.stat           = LN_STAT_IDLE;  // IDLE: available to be claimed
    loco->ln.spdMode        = LN_SPD_128;
    loco->stepMode          = 4;              // DCC128 (per DccHal::setSpeed() convention: 0=14,2=28,otherwise=128)
    loco->ln.id1            = id1;
    loco->ln.id2            = id2;
    loco->ln.lastActivityMs = millis();
    loco->ln.dirty          = false;

    _repo.unlock();
    return slot;
}

// _findFreeSlot() — builds a 121-entry "used" table by scanning every
// active repository entry that currently holds a real LocoNet slot,
// marking that slot number used. Then scans slot numbers 1 through
// 120 in order and returns the first one NOT marked used. Returns
// LN_NO_SLOT if all 120 slots are currently taken.
uint8_t SlotServer::_findFreeSlot() const
{
    bool used[121] = {};
    for (uint8_t i = 0; i < _repo.count(); i++) {
        const LocoBase* l = _repo.at(i);
        if (l && l->active && l->ln.slot != LN_NO_SLOT)
            used[l->ln.slot] = true;
    }
    for (uint8_t s = 1; s <= 120; s++)
        if (!used[s]) return s;
    return LN_NO_SLOT;
}

// ═══════════════════════════════════════════════════════════════
//  SL_RD_DATA
// ═══════════════════════════════════════════════════════════════

// ─────────────────────────────────────────────────────────────
//  _fillSlotMsg() — build a complete SL_RD_DATA (OPC_SL_RD_DATA)
//  message from a loco entry
//
//  Sets the fixed opcode and 14-byte message size, and the slot
//  number itself.
//
//  STAT1 byte layout, per the Personal Edition spec + DR5000
//  reference behaviour:
//    bits 5-4: slot state
//      FREE   = 0x00
//      COMMON = 0x10 (ACTIVE only)
//      IDLE   = 0x20 (BUSY only)
//      IN_USE = 0x30 (BUSY|ACTIVE)
//    bits 2-0: speed steps (Personal Edition table 1)
//      011 = 128 steps (SPDEX)
//      010 = 14 steps  (SPD14)
//      001 = 28 steps  (SPD28)
//      000 = 28 steps 3-byte
//
//  Builds the state portion of STAT1 first from loco->ln.stat (via a
//  switch over the four LN_STAT_* values), then ORs in the speed-step
//  portion from loco->ln.spdMode. The speed-step bit combinations per
//  the Personal Edition table are:
//    000 = 28 steps 3-byte (default/fallback)
//    001 = 28 steps Trinary   (SPD28)
//    010 = 14 steps           (SPD14)
//    011 = 128 steps          (SPD14|SPD28) ← what the DR5000 uses
//    100 = 28 steps Advanced  (SPDEX)
//    111 = 128 steps Advanced (SPDEX|SPD14|SPD28)
//
//  Fills in the 14-bit address (split across adr/adr2), the current
//  speed, the shared track status byte, and a fixed ss2 byte (0x08,
//  matching observed DR5000 reference behaviour).
//
//  Rebuilds the DIRF byte from the loco's forward flag and F0-F4 bits
//  (bit5=direction, then F0→bit4, F1→bit0, F2→bit1, F3→bit2, F4→bit3
//  — the same LocoNet-native bit layout used elsewhere in this file),
//  and the SND byte from F5-F8 (F5→bit0 .. F8→bit3).
//
//  Copies the stored throttle ID bytes, and finally computes and
//  writes the message's LocoNet checksum via writeChecksum().
// ─────────────────────────────────────────────────────────────
void SlotServer::_fillSlotMsg(LnMsg& msg, const LocoBase* loco) const
{
    msg.sd.command   = OPC_SL_RD_DATA;
    msg.sd.mesg_size = 14;
    msg.sd.slot      = loco->ln.slot;

    uint8_t stat = 0x00;
    switch (loco->ln.stat) {
        case LN_STAT_IN_USE: stat = STAT1_SL_BUSY | STAT1_SL_ACTIVE; break; // 0x30
        case LN_STAT_IDLE:   stat = STAT1_SL_BUSY;                   break; // 0x20
        case LN_STAT_COMMON: stat = STAT1_SL_ACTIVE;                 break; // 0x10
        default:             stat = 0x00;                            break; // FREE
    }
    switch (loco->ln.spdMode) {
        case LN_SPD_14:  stat |= STAT1_SL_SPD14;                    break; // 010
        case LN_SPD_28:
        case LN_SPD_28T: stat |= STAT1_SL_SPD28;                    break; // 001
        case LN_SPD_28A: stat |= STAT1_SL_SPDEX;                    break; // 100
        default:         stat |= STAT1_SL_SPD14 | STAT1_SL_SPD28;   break; // 011 = 128 steps
    }
    msg.sd.stat = stat;

    msg.sd.adr  = loco->address & 0x7F;
    msg.sd.adr2 = (loco->address >> 7) & 0x7F;
    msg.sd.spd  = loco->speed;
    msg.sd.trk  = _trkStatus;
    msg.sd.ss2  = 0x08;  // per DR5000 reference behaviour

    uint8_t dirf = loco->forward ? 0x20 : 0x00;
    if (loco->functions & 0x001) dirf |= 0x10; // F0
    if (loco->functions & 0x002) dirf |= 0x01; // F1
    if (loco->functions & 0x004) dirf |= 0x02; // F2
    if (loco->functions & 0x008) dirf |= 0x04; // F3
    if (loco->functions & 0x010) dirf |= 0x08; // F4
    msg.sd.dirf = dirf;

    uint8_t snd = 0;
    if (loco->functions & 0x020) snd |= 0x01; // F5
    if (loco->functions & 0x040) snd |= 0x02; // F6
    if (loco->functions & 0x080) snd |= 0x04; // F7
    if (loco->functions & 0x100) snd |= 0x08; // F8
    msg.sd.snd = snd;

    msg.sd.id1 = loco->ln.id1;
    msg.sd.id2 = loco->ln.id2;

    writeChecksum(msg);
}

// _sendSlotData(slot) — under the repository read lock, looks up the
// loco assigned to the given slot number. If none is found, replies
// with a failure LACK for OPC_RQ_SL_DATA instead. Otherwise builds
// the SL_RD_DATA message via _fillSlotMsg() while still holding the
// lock, releases the lock, and then queues both an OPC_BUSY message
// and the SL_RD_DATA message for transmission (see the queuing
// rationale below).
void SlotServer::_sendSlotData(LocoNetDispatcher& dispatcher, uint8_t slot)
{
    if (!_repo.lockRead(500)) return;
    LocoBase* loco = _repo.findBySlot(slot);
    if (!loco) {
        _repo.unlock();
        _sendLack(dispatcher, OPC_RQ_SL_DATA & 0x7F, 0);
        return;
    }
    LnMsg msg;
    _fillSlotMsg(msg, loco);
    _repo.unlock();
    // Queue OPC_BUSY and SL_RD_DATA on the TX queue
    // so they are sent outside the OPC callback
    // (after the CD backoff timeout has elapsed)
    LnMsg busy = makeMsg(OPC_BUSY, 0, 0);
    _queueMsg(busy);
    _queueMsg(msg);
}

// _sendSlotData(loco) — overload used when the caller already holds a
// valid LocoBase pointer (e.g. from process()'s nextLnDirty() loop),
// avoiding a redundant lookup-by-slot-number. Builds the SL_RD_DATA
// message directly from the given entry and queues it (preceded by
// an OPC_BUSY message) exactly as the slot-number overload does.
void SlotServer::_sendSlotData(LocoNetDispatcher& dispatcher,
                                const LocoBase* loco)
{
    LnMsg msg;
    _fillSlotMsg(msg, loco);
    LnMsg busy = makeMsg(OPC_BUSY, 0, 0);
    _queueMsg(busy);
    _queueMsg(msg);
}

// _sendLack() — builds a standard LocoNet long-acknowledgement
// message (replying to the given opcode with the given ack byte,
// typically 0x7F for success or 0x00 for failure) and queues it for
// transmission via _queueMsg().
void SlotServer::_sendLack(LocoNetDispatcher& dispatcher,
                            uint8_t replyToOpc, uint8_t ack)
{
    LnMsg lack = makeLongAck(replyToOpc, ack);
    _queueMsg(lack);
}


// ═══════════════════════════════════════════════════════════════
//  OPC 0xA3 — F9-F12 (RE_OPC_IB2_F9_F12)
//  Format: A3 <slot> <func> <cksum>
//  func: bit0=F9, bit1=F10, bit2=F11, bit3=F12
// ═══════════════════════════════════════════════════════════════

// _handleLocoF9F12() — an Uhlenbrock/Intellibox-style extension
// opcode (also used by Fred throttles) for setting functions F9-F12,
// which are not covered by the standard OPC_LOCO_SND (F5-F8) message.
// Same IN_USE guard as the other loco-update handlers. Clears the
// existing F9-F12 bits (bits 9-12 of the function bitmap) and rebuilds
// them from the message's func byte, refreshes the slot's last-
// activity timestamp, marks both DCC and LocoNet dirty, and publishes
// a LOCO_STATE event.
void SlotServer::_handleLocoF9F12(LocoNetDispatcher& dispatcher,
                                   const LnMsg* pkt)
{
    uint8_t slot = pkt->data[1];
    uint8_t func = pkt->data[2];

    if (!_repo.lockWrite(500)) return;
    LocoBase* loco = _repo.findBySlot(slot);
    if (loco && loco->ln.stat == LN_STAT_IN_USE) {
        uint32_t fn = loco->functions & ~0x1E00; // clear F9-F12 (bits 9-12)
        if (func & 0x01) fn |= (1UL << 9);   // F9
        if (func & 0x02) fn |= (1UL << 10);  // F10
        if (func & 0x04) fn |= (1UL << 11);  // F11
        if (func & 0x08) fn |= (1UL << 12);  // F12
        loco->functions         = fn;
        loco->ln.lastActivityMs = millis();
        _markBothDirty(loco);
        _publishLocoEvent(loco);
    }
    _repo.unlock();
}

// ═══════════════════════════════════════════════════════════════
//  Repository write helpers
// ═══════════════════════════════════════════════════════════════

// _writeSpeed() — stores the new speed (already in this codebase's
// canonical 0-127 range, regardless of step mode) into the loco
// entry, refreshes its last-activity timestamp, marks both DCC and
// LocoNet dirty flags, and dispatches a LOCO_SPEED command onto the
// CommandBus so DccHal (and any other subscribed module) picks up the
// change.
void SlotServer::_writeSpeed(LocoBase* loco, uint8_t spd)
{
    loco->speed             = spd;   // canonical value (0-127, regardless of step mode)
    loco->ln.lastActivityMs = millis();
    _markBothDirty(loco);

    Command cmd = Cmd::locoSpeed(ModuleId::LOCONET_HAL,
                                     loco->address,
                                     spd,
                                     loco->forward,
                                     loco->stepMode);
    _cmdBus.dispatch(cmd);
}

// _writeDirf() — decodes the DIRF byte's direction bit and F0-F4 bits
// (using the same bit mapping as _fillSlotMsg() uses to encode them:
// F0↔bit4, F1↔bit0, F2↔bit1, F3↔bit2, F4↔bit3), updates the loco's
// forward flag and function bitmap accordingly, refreshes the last-
// activity timestamp, and marks both dirty flags. Dispatches the
// updated speed+direction via the CommandBus (since direction is
// bundled with speed in this codebase's LOCO_SPEED command). For the
// F0-F4 function change itself, rather than dispatching a narrower
// per-function command, simply marks the whole loco dirty via
// LocoRepository::markDirty() — DccHal's dirty-poll loop then resends
// the full function state as a group, which avoids any off-by-one
// risk from trying to address individual function numbers here.
void SlotServer::_writeDirf(LocoBase* loco, uint8_t dirf)
{
    loco->forward = (dirf & 0x20) != 0;

    uint32_t fn = loco->functions & ~0x1F;
    if (dirf & 0x10) fn |= 0x01;
    if (dirf & 0x01) fn |= 0x02;
    if (dirf & 0x02) fn |= 0x04;
    if (dirf & 0x04) fn |= 0x08;
    if (dirf & 0x08) fn |= 0x10;
    loco->functions = fn;

    loco->ln.lastActivityMs = millis();
    _markBothDirty(loco);

    // Send speed+direction via the CommandBus
    _cmdBus.dispatch(Cmd::locoSpeed(ModuleId::LOCONET_HAL,
                                    loco->address, loco->speed,
                                    loco->forward, loco->stepMode));
    // Functions F0-F4: sent as a group via the setFunctions0to4 path,
    // by marking the loco dirty — the DccHal dirty loop picks up the
    // full state. This avoids off-by-one problems with individual
    // function numbering.
    _repo.markDirty(loco->address);
}

// _writeSnd() — decodes the SND byte's F5-F8 bits, rebuilds those
// four bits of the function bitmap while leaving every other function
// untouched, refreshes the last-activity timestamp, and marks both
// dirty flags. As with _writeDirf(), the actual function resend is
// handled by marking the whole loco dirty (LocoRepository::
// markDirty()) rather than dispatching a narrower command — DccHal
// picks up the full state on its next dirty-poll pass.
void SlotServer::_writeSnd(LocoBase* loco, uint8_t snd)
{
    uint32_t fn = loco->functions & ~0x1E0;
    if (snd & 0x01) fn |= 0x020;
    if (snd & 0x02) fn |= 0x040;
    if (snd & 0x04) fn |= 0x080;
    if (snd & 0x08) fn |= 0x100;
    loco->functions = fn;

    loco->ln.lastActivityMs = millis();
    _markBothDirty(loco);

    // Functions F5-F8: via the dirty flag — DccHal picks up the full state
    _repo.markDirty(loco->address);
}

// ─────────────────────────────────────────────────────────────
//  _markBothDirty() — mark the loco dirty for DCC resend, but
//  deliberately NOT for a LocoNet SL_RD_DATA broadcast
//
//  NOTE: does NOT set loco->ln.dirty here. This function is only
//  called from _writeSpeed/_writeDirf/_writeSnd — routine speed/
//  function changes from a throttle that already owns the slot. A
//  reference command station does NOT send an SL_RD_DATA broadcast
//  back in that situation (confirmed by comparison, session of 28
//  June 2026) — that broadcast is reserved for genuine slot-lifecycle
//  changes (claim/dispatch/release/status change — see the other
//  ln.dirty=true call sites elsewhere in this file).
//  _repo.markDirty() is still needed regardless: other protocol
//  modules (XpressNet/LAN/Z21) still need to forward the change on to
//  THEIR OWN clients.
// ─────────────────────────────────────────────────────────────
void SlotServer::_markBothDirty(LocoBase* loco)
{
    _repo.markDirty(loco->address);
}

// ═══════════════════════════════════════════════════════════════
//  EventBus — uses EvType and Event.state per module_arch.h
// ═══════════════════════════════════════════════════════════════

// _publishLocoEvent() — builds and publishes a LOCO_STATE event
// reflecting the given loco's current address, speed, direction and
// function bitmap, tagged with ModuleId::LOCONET_HAL as its source
// (so the LocoNet module itself does not receive this event back —
// see EventBus's own sourceId-skip logic).
void SlotServer::_publishLocoEvent(const LocoBase* loco)
{
    Event ev{};
    ev.type      = EvType::LOCO_STATE;
    ev.sourceId  = ModuleId::LOCONET_HAL;
    ev.address   = loco->address;
    ev.speed     = loco->speed;
    ev.forward   = loco->forward;
    ev.functions = loco->functions;
    _evtBus.publish(ev);
}

// _publishPowerEvent() — builds and publishes a POWER_ON or POWER_OFF
// event depending on `on`, tagged with ModuleId::LOCONET_HAL as its
// source.
void SlotServer::_publishPowerEvent(bool on)
{
    Event ev{};
    ev.type    = on ? EvType::POWER_ON : EvType::POWER_OFF;
    ev.sourceId = ModuleId::LOCONET_HAL;
    _evtBus.publish(ev);
}

// _publishAccessoryEvent() — builds and publishes an ACCESSORY_STATE
// event for the given output address, tagged with ModuleId::
// LOCONET_HAL as its source. `output` is stored in ev.output (0/1),
// and `active` is stored in ev.state (note: the Event struct's field
// is named `state`, not `active`, despite the parameter name here).
void SlotServer::_publishAccessoryEvent(uint16_t addr,
                                         bool output, bool active)
{
    Event ev{};
    ev.type     = EvType::ACCESSORY_STATE;
    ev.sourceId = ModuleId::LOCONET_HAL;
    ev.address  = addr;
    ev.output   = output ? 1 : 0;
    ev.state    = active;   // Event.state, not Event.active
    _evtBus.publish(ev);
}
