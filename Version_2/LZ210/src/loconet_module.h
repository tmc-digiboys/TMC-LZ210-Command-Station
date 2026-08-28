#pragma once
// ═══════════════════════════════════════════════════════════════
//  loconet_module.h  —  LocoNet 1.1 protocol module
//
//  Wraps the LocoNet2 library as a HardwareModule and bridges it
//  to the TMC-LZ210 CommandBus and EventBus architecture.
//
//  Pin assignments:
//    GP0 = LN_TX_PIN (inverted output for open-collector bus)
//    GP1 = LN_RX_PIN
// ═══════════════════════════════════════════════════════════════

#include "module_arch.h"
#include "slot_server.h"
#include "LocoNet2.h"
#include "LocoNetStreamRP2040.h"

#include "hardware_config.h"

constexpr int8_t LN_TX_PIN = HW_LN_TX_PIN;   // LocoNet TX (inverted for open-collector)
constexpr int8_t LN_RX_PIN = HW_LN_RX_PIN;   // LocoNet RX

class LocoNetModule : public HardwareModule
{
public:
    // LocoNetModule() — constructs the member objects in strict
    // dependency order (see the private section below) and records
    // `this` in the static _instance pointer, so the static
    // _cmdHandler() can reach this module's state.
    LocoNetModule();

    // begin() — starts the LocoNet PHY (physical layer) and registers
    // this module's CommandBus handler.
    void begin()                       override;

    // loop() — processes incoming LocoNet traffic and updates any
    // slots that have been marked dirty by other protocol modules.
    void loop()                        override;

    // onCommand() — entry point for CommandBus commands; simply
    // delegates to the static _cmdHandler() (which does the actual
    // translation into LocoNet OPC messages).
    void onCommand(const Command& cmd) override;

private:
    // Member construction order is critical — do not reorder:
    LocoNetBus           _lnBus;       // 1. Bus arbiter (must be first)
    LocoNetDispatcher    _dispatcher;  // 2. Uses _lnBus
    LocoNetStreamRP2040  _phy;         // 3. Uses _lnBus
    SlotServer           _slotServer;  // 4. Uses _dispatcher

    uint32_t _last100ms = 0;           // Timestamp for 100ms slot timeout tick

    static LocoNetModule* _instance;   // Singleton pointer for static handler
    static void _cmdHandler(const Command& cmd);  // Static CommandBus handler
};

extern LocoNetModule gLocoNet;
