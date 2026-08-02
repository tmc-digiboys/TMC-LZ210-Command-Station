// ═══════════════════════════════════════════════════════════════
//  dcc_hal.h  —  DCC Hardware Abstraction Layer
//
//  Manages the DCCPacketScheduler library and translates CommandBus
//  commands into DCC packets on the track (core0).
//
//  Pin assignments (configurable via constructor):
//    GP2  (pinDcc)  — DCC signal output
//    GP3  (pinDcc2) — DCC signal inverted (for H-bridge)
//    GP4  (pinEn)   — Third, PIO-driven DCC output without RailCom,
//                     used as RailSync for LocoNet boosters (via
//                     enable_additional_DCC_output() in begin()). On
//                     this hardware revision there is NO manual
//                     H-bridge enable anymore (the H-bridge is driven
//                     purely via GP2/GP3).
//    GP5  (pinCde)  — CDE booster E-signal input (short-circuit
//                     detection). Normally a square wave synchronised
//                     with the DCC signal; on a short circuit the
//                     booster pulls the line low. A continuous low
//                     phase longer than sys.cde_short_us (EEPROM,
//                     configurable via the web interface, default
//                     CDE_SHORT_US) triggers an emergency power-off.
//                     The measured normal low phase is ~70µs (one DCC
//                     bit half-period); the default of 500µs gives a
//                     margin of roughly 7x against false triggers.
//    GP27 (pinAck)  — ACK detection and short circuit protection
//
//  Turnout addressing (configured via web interface, stored in EEPROM):
//    dcc.swschema: 0=ROCO (offset 3, linear), 1=IB (offset 7, linear),
//    2=LENZ (offset 3, non-linear — RCN-213 non-linear address
//    sequence, see DCCPacket::getBitstream() in the external
//    DCCPacketScheduler library for the wrap-around formula, corrected
//    and verified against concrete reference measurements).
//    dcc.swformat: 0=linear, 1=non-linear (RCN-213, can also be
//    combined with ROCO/IB independently of schema=LENZ, for old Lenz
//    LS100/LS150-style decoders).
//    Address conversion is handled in DCCPacket::getBitstream().
//
//  Note: Short circuit and ACK detection via GP27 are currently
//  disabled because the DRV8871 H-bridge is not suitable for DCC
//  current sensing. See dcc_current_monitor.h for details.
// ═══════════════════════════════════════════════════════════════
#pragma once
#include "hardware_config.h"
#include "module_arch.h"
#include "command_bus.h"
#include "event_bus.h"
#include "loco_repository.h"
#include <DCCPacketScheduler_new.h>

// Default/fallback threshold for CDE short-circuit detection: a
// continuous low phase on the E-signal longer than this is considered
// a short circuit. Configurable via the EEPROM parameter
// "sys.cde_short_us" (web interface, Systeem section) — this constant
// is only the fallback default. A normal DCC square wave produces low
// phases of ~70µs. 500µs gives a margin of roughly 7x while still
// reacting quickly to a genuine short circuit.
constexpr uint32_t CDE_SHORT_US = 500;

class DccHal : public HardwareModule {
public:
    DccHal(uint8_t pinDcc  = HW_DCC_PIN,
           uint8_t pinDcc2 = HW_DCC2_PIN,
           uint8_t pinEn   = HW_DCC_RAILSYNC_PIN,
           uint8_t pinAck  = HW_DCC_ACK_PIN,
           uint8_t pinCde  = HW_CDE_PIN)
        : HardwareModule("DccHal", ModuleId::DCC_HAL, ModuleCore::CORE0)
        , _pinDcc(pinDcc), _pinDcc2(pinDcc2)
        , _pinEn(pinEn),   _pinAck(pinAck)
        , _pinCde(pinCde) {}

    // ── HardwareModule interface ───────────────────────────
    // Initialises GPIO pins, loads turnout schema from EEPROM,
    // registers the CommandBus handler
    void begin() override;

    // Updates DCC packet scheduler, processes dirty locos,
    // checks CDE short circuit detection
    void loop()  override;

    void usedPins(const uint8_t*& pins, uint8_t& count) const override {
        static uint8_t p[5];
        p[0]=_pinDcc; p[1]=_pinDcc2; p[2]=_pinEn; p[3]=_pinAck; p[4]=_pinCde;
        pins = p; count = 5;
    }

    // Emergency reset — turns off track power immediately
    void hardwareReset() override { setPower(false); }

    // Returns false if DCC hardware initialisation failed
    bool hardwareOk()    const override { return _initOk; }

    // ── Track power and loco control ──────────────────────

    // Turns track power on or off. Publishes POWER_ON/OFF event.
    void setPower(bool on);

    // Sends an emergency stop to all locos on the track
    void emergencyStop();

    // Sets loco speed (addr=DCC address, speed=0-127, fwd=direction,
    // mode=0/2/4 for 14/28/128 steps)
    void setSpeed(uint16_t addr, uint8_t speed, bool fwd, uint8_t mode);

    // Sets a loco function on or off (fn=0-68)
    void setFunction(uint16_t addr, uint8_t fn, bool state);

    // Activates a turnout output (addr=1-2048, output=0/1, active=coil on)
    void setAccessory(uint16_t addr, uint8_t output, bool active);

    // Reads a CV from the programming track (service mode)
    void cvRead(uint16_t cv);

    // Writes a CV to the programming track (service mode)
    void cvWrite(uint16_t cv, uint8_t value);

    // Writes a CV via POM (programming on main — no readback)
    void pomWrite(uint16_t addr, uint16_t cv, uint8_t value);

    // Reads a CV via POM (bitwise, requires BiDi/RailCom)
    void pomRead(uint16_t addr, uint16_t cv);

    // Returns current track power state
    bool getPower() const { return _power; }

    // Returns true if the DCC scheduler is currently repeating
    // service mode packets (programming track active)
    bool isInServiceMode() const;

    // Reports a valid ACK received from a decoder to the scheduler.
    // Called by DccCurrentMonitor when ACK detection is enabled.
    void notifyAck();

    // Called from the CDE falling-edge interrupt (ISR context, core0).
    // Records the timestamp of the falling edge.
    static void _cdeIsr();

private:
    DCCPacketScheduler _dcc;
    uint8_t _pinDcc, _pinDcc2, _pinEn, _pinAck, _pinCde;
    bool    _power  = false;
    bool    _initOk = false;

    // CDE short-circuit detection — interrupt-based:
    //   _cdeFallUs : timestamp (micros()) of the last falling edge,
    //                set inside the ISR on every falling edge on _pinCde.
    //   _cdeLineDown is tracked as a file-scope static in dcc_hal.cpp
    //   (not as a member, since ISR access only needs the timestamp
    //   here, and the "is the line currently down" flag is only ever
    //   used within dcc_hal.cpp itself).
    static volatile uint32_t _cdeFallUs;

    // Static CommandBus handler — called on core0 for each incoming command
    static void _onCommand(const Command& cmd);

    // Checks the CDE short-circuit flag and triggers power-off if set
    void _checkCdeShort();

    // DCCPacketScheduler library callbacks (need access to _pinAck)
    friend void notifyRailpower(uint8_t state);
    friend void notifyLokAll(uint16_t, uint8_t, uint8_t,
                              uint8_t, uint8_t, uint8_t, uint8_t);
    friend void notifyCVVerify(uint16_t, uint8_t);
    friend void notifyCVPOMRead(uint16_t, uint8_t);
    friend void notifyCVNack(uint16_t);
    friend uint16_t notifyCurrentSense();

    // Iterates the loco repository and sends updated speed/function
    // packets for all locos marked as dirty
    void _processDirtyLocos();

    // Loads turnout address schema from EEPROM and applies it to
    // the DCC scheduler (ROCO/IB/LENZ offset, linear/non-linear)
    void _applySwitchMode();

    // Rescales a canonical speed (0-127, regardless of step mode) to
    // the range expected by the library's setSpeed14()/setSpeed28().
    // mode: 0=14-step (single stop/estop reservation), 2=28-step
    // (double reservation per RCN-212 — see the definition in
    // dcc_hal.cpp for a full explanation).
    static uint8_t _scaleSpeed(uint8_t speed, uint8_t mode);

    // Short circuit detection — currently disabled (hardware limitation)
    void _checkShortCircuit();
};

extern DccHal gDccHal;
