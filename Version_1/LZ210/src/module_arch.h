// ═══════════════════════════════════════════════════════════════
//  module_arch.h  —  Core architecture definitions for TMC-LZ210
//
//  This file defines the fundamental building blocks of the TMC
//  DCC command station: ModuleBase, HardwareModule, ProtocolModule,
//  Command and Event types.
//
//  ARCHITECTURE OVERVIEW:
//  ┌─────────────────────────────────────────────────────────┐
//  │  Core 1 (protocol / network)                           │
//  │  LocoNet  XpressNet  LenzLAN  Z21LAN  LenzUSB Webserver│
//  │              │                │                        │
//  │         CommandBus ──────────────────────────────────► │
//  ├─────────────────────────────────────────────────────────┤
//  │  Core 0 (hardware / realtime)                          │
//  │  DccHal  RsBusHal  OledDisplay                         │
//  │              │                                         │
//  │         EventBus ◄──────────────────────────────────   │
//  └─────────────────────────────────────────────────────────┘
//
//  COMMUNICATION:
//  - CommandBus: core1 → core0 (lock-free ring buffer)
//    Protocol modules decode XpressNet/Z21 messages and send
//    protocol-agnostic Commands to the DCC HAL.
//  - EventBus: core0 → core1 (handler registration)
//    Hardware modules publish Events (power, loco state,
//    RS-Bus feedback) that protocol modules broadcast to clients.
//
//  MODULES:
//  All modules inherit from ModuleBase and are registered in the
//  ModuleRegistry. Core0 modules are started via beginCore0(),
//  core1 modules via beginCore1().
// ═══════════════════════════════════════════════════════════════
#pragma once
#include <Arduino.h>
#include <stdint.h>

// ─────────────────────────────────────────────────────────────
//  ModuleCore — which RP2350 core the module runs on
// ─────────────────────────────────────────────────────────────
enum class ModuleCore : uint8_t {
    CORE0 = 0,   // Realtime hardware core (DCC, RS-Bus)
    CORE1 = 1,   // Protocol/network core (TCP, UDP, USB)
    BOTH  = 2,   // Runs on both cores (e.g. LocoRepository)
};

// ─────────────────────────────────────────────────────────────
//  ModuleId — unique identifier per module
//
//  Used for:
//  - Loop prevention: CommandBus stores source module ID so a
//    module does not receive its own echo back
//  - EventBus registration: one handler per moduleId
// ─────────────────────────────────────────────────────────────
namespace ModuleId {
    constexpr uint8_t DCC_HAL         = 0x01;  // DccHal (core0)
    constexpr uint8_t RS_BUS_HAL      = 0x02;  // RsBusHal PIO (core0)
    constexpr uint8_t XPRESSNET_RS485 = 0x03;  // XpressNetRs485 (core1)
    constexpr uint8_t LOCONET_HAL     = 0x04;  // LocoNetHal (core1)
    constexpr uint8_t OLED_DISPLAY    = 0x06;  // OledDisplay (core0)
    constexpr uint8_t LENZ_LAN        = 0x10;  // LenzLan TCP port 5550 (core1)
    constexpr uint8_t LENZ_USB        = 0x11;  // LenzUsb CDC serial (core1)
    constexpr uint8_t Z21_LAN         = 0x12;  // Z21Lan UDP port 21105 (core1)
    constexpr uint8_t WEBSERVER       = 0x20;  // Webserver HTTP (core1)
    constexpr uint8_t EEPROM_STORE    = 0x30;  // EepromStore LittleFS (core0/1)
}

// ─────────────────────────────────────────────────────────────
//  Command — protocol-agnostic command (core1 → core0)
//
//  Protocol modules (LenzLAN, Z21, XpressNet RS485) decode
//  incoming messages and send a Command on the CommandBus.
//  The DccHal executes the command as a DCC packet.
//
//  Fields are interpreted depending on type:
//  LOCO_SPEED:     address, speed (0-127), forward, stepMode
//  LOCO_FUNCTION:  address, fn (0-68), fnState
//  LOCO_ESTOP:     address (0=all locos)
//  ACCESSORY_SET:  address (1-2048), output (0/1), active
//  POWER_ON/OFF:   no fields
//  CV_READ:        cv
//  CV_WRITE:       cv, cvValue
//  POM_WRITE:      address, cv, cvValue
//  POM_READ:       address, cv
// ─────────────────────────────────────────────────────────────
enum class CmdType : uint8_t {
    LOCO_SPEED        = 0x01,
    LOCO_FUNCTION     = 0x02,
    LOCO_ESTOP        = 0x03,
    ACCESSORY_SET     = 0x10,
    POWER_ON          = 0x20,
    POWER_OFF         = 0x21,
    EMERGENCY_STOP    = 0x22,
    CV_READ           = 0x30,
    CV_WRITE          = 0x31,
    POM_WRITE         = 0x32,
    POM_READ          = 0x33,
    LN_DISPATCH       = 0x40,   // LocoNet: place loco in dispatch slot
};

struct Command {
    CmdType  type;
    uint8_t  sourceId;   // ModuleId of sender (loop prevention)
    uint16_t address;    // Loco or accessory address
    uint8_t  speed;      // 0-127
    bool     forward;    // Direction of travel
    uint8_t  stepMode;   // 0=14, 2=28, other=128 speed steps
    uint8_t  fn;         // Function number 0-68
    bool     fnState;    // Function on/off
    uint8_t  output;     // Turnout output: 0=thrown, 1=straight
    bool     active;     // Turnout coil activation (true=pulse on)
    uint16_t cv;         // CV number
    uint8_t  cvValue;    // CV value
};

// ─────────────────────────────────────────────────────────────
//  Event — hardware event (core0 → core1)
//
//  Hardware modules publish Events in response to DCC events,
//  short circuits, RS-Bus feedback etc. Protocol modules receive
//  these Events and broadcast them to their clients.
//
//  FEEDBACK:      fbModule (1-127), fbNibble (0/1), fbDat (4-bit)
//  FEEDBACK_BULK: fbBulkAddr[], fbBulkData[], fbBulkCount
//                 (max 28 RS-Bus pairs, stored inline)
//  LOCO_STATE:    address, speed, forward, functions (32-bit bitmap)
//  ACCESSORY_STATE: address, output, state
// ─────────────────────────────────────────────────────────────
enum class EvType : uint8_t {
    POWER_ON            = 0x01,
    POWER_OFF           = 0x02,
    EMERGENCY_STOP      = 0x03,
    SHORT_CIRCUIT       = 0x04,
    LOCO_STATE          = 0x10,   // address, speed, forward, functions
    ACCESSORY_STATE     = 0x11,   // address, output, state
    FEEDBACK            = 0x20,   // fbModule, fbNibble, fbDat, fbExt
    FEEDBACK_BULK       = 0x21,   // fbBulkAddr[], fbBulkData[], fbBulkCount
    CV_OK               = 0x30,   // cv, cvValue
    CV_NOFOUND          = 0x31,   // no ACK from decoder
    CV_SHORT            = 0x32,   // short circuit during programming
    POM_OK              = 0x33,   // address, cv, cvValue
    POM_ERR             = 0x34,
    PROG_MODE_ON        = 0x35,
    PROG_MODE_OFF       = 0x36,
};

struct Event {
    EvType   type;
    uint8_t  sourceId;      // ModuleId of sender
    uint16_t address;       // Loco or accessory address
    uint8_t  speed;
    bool     forward;
    uint32_t functions;     // Function bitmap: bit0=F0, bit1=F1, ..., bit28=F28
    uint8_t  output;        // Turnout output
    bool     state;         // Turnout position
    uint16_t fbModule;      // RS-Bus module address (FEEDBACK)
    uint8_t  fbNibble;      // RS-Bus nibble 0 or 1 (FEEDBACK)
    uint8_t  fbDat;         // RS-Bus nibble value 0-15 (FEEDBACK)
    bool     fbExt;         // RS-Bus extended type flag (FEEDBACK)
    // Bulk RS-Bus feedback (FEEDBACK_BULK): inline, max 28 pairs
    // addrByte: bit7=nibble selector, bits 6-0=module address
    // dataByte: bits 7-6=01 Lenz marker, bits 3-0=nibble value
    uint8_t  fbBulkAddr[28];
    uint8_t  fbBulkData[28];
    uint8_t  fbBulkCount = 0;
    uint16_t cv;
    uint8_t  cvValue;
};

// ─────────────────────────────────────────────────────────────
//  ModuleBase — abstract base class for all modules
//
//  Every module has a name, unique ID and target core assignment.
//  The ModuleRegistry calls begin() once during startup and
//  loop() repeatedly during normal operation.
// ─────────────────────────────────────────────────────────────
class ModuleBase {
public:
    ModuleBase(const char* name, uint8_t id, ModuleCore core)
        : _name(name), _id(id), _core(core), _enabled(true) {}
    virtual ~ModuleBase() = default;

    // Called once during startup on the appropriate core
    virtual void begin() = 0;

    // Called repeatedly in the main loop on the appropriate core
    virtual void loop()  = 0;

    // Optional handlers (default no-op)
    virtual void onCommand(const Command& cmd) {}
    virtual void onEvent(const Event& ev)      {}

    const char* name()    const { return _name; }
    uint8_t     id()      const { return _id; }
    ModuleCore  core()    const { return _core; }
    bool        enabled() const { return _enabled; }
    void        setEnabled(bool v) { _enabled = v; }

    // Type discrimination without RTTI (Arduino: -fno-rtti)
    virtual bool isHardwareModule() const  { return false; }
    virtual bool isProtocolModule() const  { return false; }
    virtual uint8_t connectionCount() const { return 0; }
    virtual uint8_t maxConnections()  const { return 0; }

protected:
    const char* _name;
    uint8_t     _id;
    ModuleCore  _core;
    bool        _enabled;
};

// ─────────────────────────────────────────────────────────────
//  HardwareModule — base class for hardware modules (core0)
//
//  Hardware modules control GPIO, PIO or other RP2350 peripherals.
//  The ModuleRegistry can detect pin conflicts between modules.
// ─────────────────────────────────────────────────────────────
class HardwareModule : public ModuleBase {
public:
    HardwareModule(const char* name, uint8_t id, ModuleCore core)
        : ModuleBase(name, id, core) {}
    bool isHardwareModule() const override { return true; }

    // Return array of GPIO pins in use (for conflict detection)
    virtual void usedPins(const uint8_t*& pins, uint8_t& count) const {
        pins = nullptr; count = 0;
    }

    // Bitmask of PIO state machines in use
    // bits 0-3=PIO0 SM0-3, bits 4-7=PIO1 SM0-3, bits 8-11=PIO2 SM0-3
    virtual uint16_t usedPioSmMask() const { return 0; }

    // Emergency reset (e.g. on short circuit)
    virtual void hardwareReset() {}

    // Hardware diagnostics — returns false if hardware is faulty
    virtual bool hardwareOk() const { return true; }
};

// ─────────────────────────────────────────────────────────────
//  ProtocolModule — base class for protocol modules (core1)
//
//  Protocol modules translate network/serial traffic into Commands
//  and receive Events for broadcasting to their clients.
//
//  Shared XpressNet parsing: all protocol modules use
//  XpressNetHandler as a common decoder, avoiding duplication
//  of parse logic across LenzLAN, LenzUSB and XpressNet RS485.
// ─────────────────────────────────────────────────────────────
class ProtocolModule : public ModuleBase {
public:
    ProtocolModule(const char* name, uint8_t id, ModuleCore core)
        : ModuleBase(name, id, core) {}
    bool isProtocolModule() const override { return true; }

    // Required: broadcast events to all connected clients
    virtual void onEvent(const Event& ev) override = 0;
};
