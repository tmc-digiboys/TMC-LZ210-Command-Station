// ═══════════════════════════════════════════════════════════════
//  loconet_module.cpp  —  LocoNet protocol module
//
//  Integrates LocoNet 1.1 into the TMC-LZ210 architecture.
//  Bridges between the LocoNet bus and the CommandBus/EventBus so
//  that LocoNet throttles and other devices work alongside LenzLAN,
//  Z21LAN and XpressNet handhelds on the same command station.
//
//  Architecture:
//    LocoNetBus          — byte-level bus arbiter (from LocoNet2 library)
//    LocoNetDispatcher   — opcode dispatcher, routes OPC_* to handlers
//    LocoNetStreamRP2040 — PIO UART physical layer (Serial1, GP0/GP1)
//    SlotServer          — manages the 120 LocoNet slots
//
//  Construction order is critical: _lnBus must be constructed first,
//  then the dispatcher and PHY (which take a reference to _lnBus),
//  then the SlotServer (which takes a reference to the dispatcher).
// ═══════════════════════════════════════════════════════════════
#include "loconet_module.h"
#include "loco_repository.h"
#include "command_bus.h"
#include "event_bus.h"
#include "led_status.h"
#include <Arduino.h>

LocoNetModule* LocoNetModule::_instance = nullptr;
LocoNetModule  gLocoNet;

// ─────────────────────────────────────────────────────────────
//  Constructor — build member objects in dependency order
//
//  _lnBus → _dispatcher and _phy → _slotServer
//  Stores the instance pointer for use in the static _cmdHandler.
// ─────────────────────────────────────────────────────────────
LocoNetModule::LocoNetModule()
    : HardwareModule("LocoNet", ModuleId::LOCONET_HAL, ModuleCore::CORE0)
    , _lnBus()
    , _dispatcher(&_lnBus)
    , _phy(&Serial1, LN_RX_PIN, LN_TX_PIN, &_lnBus, false, true)
    , _slotServer(_dispatcher,
                  LocoRepository::instance(),
                  CommandBus::instance(),
                  EventBus::instance())
{
    _instance = this;
}

// ─────────────────────────────────────────────────────────────
//  begin() — start the LocoNet physical layer and register handlers
//
//  Initialises the LocoRepository (safe to call multiple times).
//  Starts the RP2040 PIO UART on Serial1 (GP0=TX, GP1=RX) with
//  TX inverted for LocoNet open-collector signalling.
//  Registers as a CommandBus handler so loco updates from LenzLAN
//  and Z21 are reflected back on the LocoNet bus as SL_RD_DATA.
// ─────────────────────────────────────────────────────────────
void LocoNetModule::begin()
{
    // Initialise shared LocoRepository (spinlock claim is guarded internally)
    LocoRepository::instance().begin();

    // Start PHY: Serial1 on GP0/GP1, TX inverted for LocoNet signalling
    _phy.start();

    // Register as CommandBus handler so loco updates from LenzLAN/Z21
    // appear as SL_RD_DATA broadcasts on the LocoNet bus
    CommandBus::instance().registerHandler(ModuleId::LOCONET_HAL, _cmdHandler);
}

// ─────────────────────────────────────────────────────────────
//  loop() — process LocoNet traffic and update dirty slots
//
//  _phy.process() reads incoming LocoNet bytes and dispatches them
//  through the dispatcher to the SlotServer OPC handlers.
//  _slotServer.process() sends SL_RD_DATA for any slots marked dirty
//  (e.g. by speed/function changes from other protocol modules).
//  process100ms() runs slot timeout detection every 100ms to release
//  idle slots back to the free pool.
//
//  IMPORTANT: everything after _phy.process() (draining the TX queue,
//  broadcasting dirty slots, 100ms maintenance) must NOT run while an
//  INCOMPLETE message currently sits in the receive buffer. Otherwise
//  we risk sending our own (blocking, byte-by-byte) response while
//  another device (e.g. a Fred throttle) is still in the middle of
//  sending a multi-byte message — causing the remaining bytes of that
//  message to be lost in the receive buffer while we are busy
//  transmitting. See the session of 28 June: Fred's OPC_WR_SL_DATA
//  was being systematically cut off after the first byte because of
//  this. messageInProgress() answers exactly the right question ("is
//  there currently an incomplete message?"), unlike isBusy()'s
//  time-based heuristic (LocoNetRxByteMicros = exactly one byte time,
//  too tight for the normal inter-byte gap of a real device). Delaying
//  by one cycle is harmless — all queue/dirty logic simply gets picked
//  up again the next time regardless.
// ─────────────────────────────────────────────────────────────
void LocoNetModule::loop()
{
    // v2 hardware: pulse the LocoNet bus-activity LED whenever bytes
    // are waiting to be read — checked BEFORE _phy.process() consumes
    // them, since that call drains the underlying Serial1 buffer.
    // This intentionally pulses on any incoming byte (not only on a
    // complete, successfully-dispatched message), giving a simple and
    // robust "there is activity on the LocoNet bus" indication without
    // needing to hook into the LocoNet2 library's own internals.
    if (Serial1.available()) gLeds.onLnBusTraffic();


    // Process incoming LocoNet bytes → dispatcher → SlotServer handlers

    _phy.process();

    if (_phy.messageInProgress()) return;
    
    _slotServer.process(_dispatcher);
    
    // Every 100ms: run slot timeout detection
    uint32_t now = millis();
    if (now - _last100ms >= 100) {
        _last100ms = now;

        _slotServer.process100ms(_dispatcher);

    }

}

// ─────────────────────────────────────────────────────────────
//  onCommand() — CommandBus handler entry point
//
//  Delegates to the static _cmdHandler.
// ─────────────────────────────────────────────────────────────
void LocoNetModule::onCommand(const Command& cmd)
{
    _cmdHandler(cmd);
}

// ─────────────────────────────────────────────────────────────
//  _cmdHandler() — static CommandBus handler
//
//  Translates CommandBus commands from other protocol modules
//  into LocoNet OPC messages so LocoNet throttles see the changes.
//
//  POWER_ON:       sends OPC_GPON, sets TRK_POWER_ON in slot server
//  POWER_OFF:      sends OPC_GPOFF, clears TRK_POWER_ON
//  EMERGENCY_STOP: sends OPC_IDLE (global emergency stop)
//  LN_DISPATCH:    sends OPC_MOVE_SLOTS to place loco in dispatch slot 0
//  LOCO_SPEED/
//  LOCO_FUNCTION:  marks the loco's LocoNet slot dirty so SlotServer
//                  sends an SL_RD_DATA broadcast on the next loop
//
//  Static because the CommandBus cannot pass a `this` pointer; uses
//  the _instance pointer set by the constructor. Returns immediately
//  if no instance has been constructed yet, or if the command
//  originated from this very module (cmd.sourceId ==
//  ModuleId::LOCONET_HAL) — the latter check prevents a command that
//  was itself derived from LocoNet traffic from being echoed straight
//  back onto the LocoNet bus.
// ─────────────────────────────────────────────────────────────
void LocoNetModule::_cmdHandler(const Command& cmd)
{
    if (!_instance) return;

    // Do not echo our own commands back to LocoNet
    if (cmd.sourceId == ModuleId::LOCONET_HAL) return;

    if (cmd.type == CmdType::POWER_ON) {
        _instance->_slotServer.setTrkStatus(
            _instance->_slotServer.trkStatus() | TRK_POWER_ON);
            digitalWrite(7,HIGH);
        LnMsg msg = makeMsg(OPC_GPON, 0, 0);
        _instance->_slotServer.queueMsg(msg);
        digitalWrite(7,LOW);
        return;
    }
    if (cmd.type == CmdType::POWER_OFF) {
        _instance->_slotServer.setTrkStatus(
            _instance->_slotServer.trkStatus() & ~TRK_POWER_ON);
        LnMsg msg = makeMsg(OPC_GPOFF, 0, 0);
        _instance->_slotServer.queueMsg(msg);
        return;
    }
    if (cmd.type == CmdType::EMERGENCY_STOP) {
        LnMsg msg = makeMsg(OPC_IDLE, 0, 0);
        _instance->_slotServer.queueMsg(msg);
        return;
    }

    // LN_DISPATCH: move the loco's slot to the dispatch slot (slot 0)
    if (cmd.type == CmdType::LN_DISPATCH) {
        LocoRepository& repo = LocoRepository::instance();
        if (repo.lockWrite(200)) {
            for (uint8_t i = 0; i < repo.count(); i++) {
                const LocoBase* l = repo.at(i);
                if (l && l->active && l->address == cmd.address &&
                    l->ln.slot != LN_NO_SLOT) {
                    LnMsg msg = makeMsg(OPC_MOVE_SLOTS, l->ln.slot, 0);
                    _instance->_slotServer.queueMsg(msg);
                    break;
                }
            }
            repo.unlock();
        }
        return;
    }

    // Loco update from another module: mark ln.dirty so SlotServer
    // sends an SL_RD_DATA broadcast to notify LocoNet throttles
    if (cmd.type != CmdType::LOCO_SPEED &&
        cmd.type != CmdType::LOCO_FUNCTION) return;

    LocoRepository& repo = LocoRepository::instance();
    if (!repo.lockWrite(200)) return;

    for (uint8_t i = 0; i < repo.count(); i++) {
        LocoBase* l = const_cast<LocoBase*>(repo.at(i));
        if (l && l->active && l->address == cmd.address &&
            l->ln.slot != LN_NO_SLOT)
        {
            l->ln.dirty = true;
            break;
        }
    }
    repo.unlock();
}
