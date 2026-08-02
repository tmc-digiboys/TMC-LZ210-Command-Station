// ═══════════════════════════════════════════════════════════════
//  dcc_hal.cpp  —  DCC Hardware Abstraction Layer
//
//  Drives DCC signal output via the DCCPacketScheduler library.
//  Translates protocol-agnostic CommandBus commands into DCC packets
//  on the track.
//
//  Responsibilities:
//  - Track power on/off and emergency stop
//  - Loco speed and function updates (via dirty-poll of LocoRepository)
//  - Turnout (accessory decoder) commands
//  - CV read/write on the programming track and via POM
//  - Short circuit and ACK detection (currently disabled — see below)
//
//  Turnout addressing is configured via the web interface:
//  - dcc.swschema: ROCO (offset 3) / IB (offset 7) / LENZ (non-linear)
//  - dcc.swformat: Linear / Non-linear (RCN-213)
//  Address conversion is performed inside DCCPacket::getBitstream().
//
//  The DCCPacketScheduler notify callbacks (notifyRailpower etc.) are
//  called by the library and publish events on the EventBus.
//
//  Note on current sensing (GP27 / DRV8871):
//  The DRV8871 H-bridge is not suitable for DCC current sensing.
//  Its over-current protection triggers in microseconds and the
//  100mOhm shunt produces only ~6mV for a 60mA ACK pulse — too
//  small for reliable ADC measurement. Both DccCurrentMonitor calls
//  are therefore disabled until the hardware is revised.
// ═══════════════════════════════════════════════════════════════

#include "dcc_hal.h"
#include "oled_display.h"
#include "eeprom_store.h"
#include "led_status.h"
#include "dcc_current_monitor.h"
#include "xpressnet_handler.h"  // for gCentrale (trackPowerOff/emergencyStop)

// Global instance used by all modules and the notify callbacks
DccHal gDccHal;

// Static pointer for use in notify callbacks (no 'this' pointer available)
static DccHal* _instance = nullptr;

// ─────────────────────────────────────────────────────────────
//  CDE short-circuit detection — interrupt-based (GP05 = E-signal)
//
//  The E-signal is normally a square wave (~70µs low per DCC bit). On
//  a short circuit the booster pulls the line permanently low — there
//  is NO rising edge anymore.
//
//  Approach:
//  - Falling-edge ISR: stores the timestamp in _cdeFallUs and sets
//    _cdeLineDown=true.
//  - Rising-edge ISR: resets _cdeLineDown=false (line back high, so a
//    normal square wave — no short circuit).
//  - loop() (_checkCdeShort): if _cdeLineDown=true AND
//    (micros() - _cdeFallUs) >= CDE_SHORT_US → short circuit.
//
//  Why loop() and not the ISR itself: setPower() and the EventBus are
//  not ISR-safe; using this flag means the actual action always
//  happens outside ISR context.
// ─────────────────────────────────────────────────────────────
volatile uint32_t DccHal::_cdeFallUs    = 0;

// Separate flag tracking whether the line is currently low.
// Not declared as a member (that one is gone), but as a file-scope static.
static volatile bool _cdeLineDown = false;

// _cdeIsr() — GPIO interrupt handler for the CDE E-signal pin,
// triggered on both edges (CHANGE). On a falling edge (line just went
// low), records the current timestamp in _cdeFallUs and sets
// _cdeLineDown=true, starting the short-circuit timing window. On a
// rising edge (line back high — a normal square wave, not a short
// circuit), simply clears _cdeLineDown. Does nothing if no DccHal
// instance has been registered yet (_instance still null).
void DccHal::_cdeIsr() {
    if (!_instance) return;
    if (!digitalRead(_instance->_pinCde)) {
        // Falling edge — line goes low, start timing
        _instance->_cdeFallUs = micros();
        _cdeLineDown = true;
    } else {
        // Rising edge — line back high, normal square wave, no short circuit
        _cdeLineDown = false;
    }
}

// ─────────────────────────────────────────────────────────────
//  _checkCdeShort() — poll the CDE short-circuit flag and act on it
//
//  Called every loop() iteration. Does nothing if track power is
//  already off (nothing to protect), or if the E-signal line is not
//  currently held low (either it is high, or it has never gone low
//  since the flag was last cleared). If the line has been low for
//  less than the configured threshold (sys.cde_short_us, falling back
//  to CDE_SHORT_US if not set in EEPROM), this is still within the
//  normal square-wave low phase, so nothing is done yet.
//
//  Once the line has been continuously low for longer than the
//  threshold, this is treated as a genuine short circuit: the flag is
//  cleared (so this only fires once per short-circuit event), and the
//  configured response mode (EEPROM "sys.cde_mode") is checked:
//    0 = turn the station off: power-off + (implicitly, via the
//        POWER_OFF command/EventBus) informs LocoNet devices as well —
//        the default and safest behaviour.
//    1 = let the booster handle it locally: the station itself stays
//        powered on, and this function returns without taking any
//        action of its own.
//  For mode 0, logs the detection, updates the relevant gCentrale
//  flags (track power off, no longer an emergency stop, but a genuine
//  short circuit), dispatches a POWER_OFF command onto the command
//  bus (so protocol modules are informed the normal way), directly
//  turns off DCC track power via setPower(false), and lights the
//  short-circuit status LED.
// ─────────────────────────────────────────────────────────────
void DccHal::_checkCdeShort() {
    if (!_power) return;             // already off, nothing to do
    if (!_cdeLineDown) return;       // line high (or has never gone low)
    uint16_t thresholdUs = eepromStore().getUint16("sys.cde_short_us", CDE_SHORT_US);
    if ((micros() - _cdeFallUs) < thresholdUs) return;  // still within threshold

    // Line continuously low for longer than the threshold — short circuit detected.
    _cdeLineDown = false;

    uint8_t mode = eepromStore().getUint8("sys.cde_mode", 0);
    // sys.cde_mode:
    //   0 = turn station off: power-off + LocoNet GPOFF (default, safest)
    //   1 = booster handles it locally: station stays on, booster deals with it itself
    if (mode == 1) return;

    // Mode 0: turn the station off
    Serial2.printf("[%lu] CDE kortsluit gedetecteerd — power off\n", millis());
    gCentrale.trackPowerOff = true;
    gCentrale.emergencyStop = false;
    gCentrale.shortCircuit  = true;

    Command cmd{};
    cmd.type     = CmdType::POWER_OFF;
    cmd.sourceId = ModuleId::DCC_HAL;
    CommandBus::instance().dispatch(cmd);

    setPower(false);
    gLeds.onShortCircuit();
}

// ─────────────────────────────────────────────────────────────
//  begin() — initialise DCC hardware and turnout addressing
//
//  Configures GPIO pins, loads the turnout address schema and
//  DCC scheduler settings from EEPROM, and registers the
//  CommandBus handler on core0.
//
//  Stores `this` in the file-scope _instance pointer, so the static
//  notify callbacks and the CommandBus handler (which cannot carry a
//  real `this` pointer) can still reach the instance's state.
//
//  Sets _pinAck as a plain digital input (note: GP4/_pinEn is
//  deliberately NOT configured as a manual enable pin on this
//  hardware revision — see setPower() and the
//  enable_additional_DCC_output() call below for how it is actually
//  used).
//
//  Configures _pinCde (the CDE booster E-signal input) as an input
//  with pull-up, and attaches _cdeIsr() as an interrupt handler on
//  both edges (CHANGE), so short-circuit timing can be measured
//  continuously regardless of the main loop's timing.
//
//  Calls _applySwitchMode() to configure the DCC scheduler's turnout
//  addressing (offset/linearity) from EEPROM — this must happen
//  before enabling the additional DCC output below, since that
//  relies on the scheduler already being set up.
//
//  Enables the third, PIO-driven DCC output on GP4 (_pinEn), used as
//  RailSync for LocoNet boosters, via the library's
//  enable_additional_DCC_output(). This call MUST happen AFTER the
//  scheduler has been initialised (_applySwitchMode() → _dcc.setup()).
//
//  Marks track power as off and initialisation as successful, then
//  registers _onCommand() as this module's CommandBus handler on
//  core0, so commands dispatched by protocol modules reach this class.
//
//  The DccCurrentMonitor initialisation call is left in as a comment,
//  since that feature is currently disabled (see the file header
//  comment on why the DRV8871 hardware is unsuitable for it).
// ─────────────────────────────────────────────────────────────
void DccHal::begin() {
    _instance = this;

    // GP4 (_pinEn) is NOT a manual enable pin on this hardware revision —
    // see setPower() and the enable_additional_DCC_output() call below.
    // Only _pinAck remains a plain digital input.
    pinMode(_pinAck, INPUT);

    // GP5 (_pinCde) — CDE booster E-signal input.
    // Interrupt on both edges: the ISR measures the low-phase duration
    // and sets _cdeLineDown if it exceeds CDE_SHORT_US (= 250µs).
    pinMode(_pinCde, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(_pinCde), DccHal::_cdeIsr, CHANGE);

    _applySwitchMode();

    // Third, PIO-driven DCC output without RailCom on GP4, used as
    // RailSync for LocoNet boosters. Must be called AFTER the
    // scheduler has been initialised (_applySwitchMode() → _dcc.setup()).
    _dcc.enable_additional_DCC_output(_pinEn);

    _power  = false;
    _initOk = true;

    // DccCurrentMonitor disabled — DRV8871 not suitable for DCC current sensing
    // gDccCurrentMonitor.begin(_pinAck, shortAdc);

    CommandBus::instance().registerHandler(
        ModuleId::DCC_HAL, DccHal::_onCommand, ModuleCore::CORE0);
}

// ─────────────────────────────────────────────────────────────
//  _applySwitchMode() — load turnout address schema from EEPROM
//
//  Reads dcc.swschema (0=ROCO, 1=IB, 2=LENZ) and dcc.swformat
//  (0=linear, 1=non-linear) from the parameter store.
//
//  Also reads DCC scheduler parameters (RailCom, programming repeat
//  counts) and injects them via setExternalConfig() so the scheduler
//  uses EEPROM values instead of its own internal storage.
//
//  LENZ schema (2) uses non-linear addressing (offset equal to
//  ROCO=3, combined with the RCN-213 non-linear address sequence).
//  The earlier library implementation of that sequence contained a
//  bug (wrong wrap-around formula at every 256-address boundary),
//  which caused e.g. address 571 to end up at the wrong physical
//  address. Corrected in DCCPacket.cpp (external DCCPacketScheduler
//  library, not part of this project) — verified against concrete
//  reference measurements.
//
//  dcc.swformat=1 (non-linear) can also be checked independently of
//  schema=LENZ, for old Lenz LS100/LS150-style decoders on a ROCO/IB
//  offset.
//  NOTE: DCCPacketScheduler::setup() itself always sets
//  nonLinearAddressing/trntFormat based on the format argument it is
//  called with — so the override below must happen AFTER setup(),
//  otherwise setup() would overwrite it back to linear.
// ─────────────────────────────────────────────────────────────
void DccHal::_applySwitchMode() {
    const ParamDef* ps = eepromStore().findParam("dcc.swschema");
    const ParamDef* pf = eepromStore().findParam("dcc.swformat");
    uint8_t schema = ps ? ps->value.u8 : 0;  // 0=ROCO, 1=IB, 2=LENZ
    uint8_t format = pf ? pf->value.u8 : 0;  // 0=linear, 1=non-linear

    // Inject scheduler config from EepromStore, overriding internal EEPROM reads
    DCCPacketScheduler::ExternalConfig extCfg;
    extCfg.railcom      = (int8_t)eepromStore().getBool("dcc.railcom");
    extCfg.progReadMode = (int8_t)eepromStore().getUint8("dcc.progreadmode");
    extCfg.progRepeat   = (int16_t)eepromStore().getUint8("dcc.progrepeat");
    extCfg.rstSRepeat   = (int16_t)eepromStore().getUint8("dcc.rstsrepeat");
    extCfg.rstCRepeat   = (int16_t)eepromStore().getUint8("dcc.rstcrepeat");
    _dcc.setExternalConfig(extCfg);

    uint8_t trnFmt;
    switch (schema) {
        case 1:  trnFmt = IB;   break;
        default: trnFmt = ROCO; break;   // also for LENZ (schema==2): same offset as ROCO
    }

    if (format == 1 || schema == 2) {
        // Non-linear encoding: either explicitly checked, or inherent
        // to the LENZ schema. _dcc.setup() itself always sets
        // nonLinearAddressing/trntFormat based on the format argument
        // — so call setup() first and only apply the desired
        // combination AFTER that, otherwise setup() would overwrite
        // our own setting.
        _dcc.setup(_pinDcc, _pinDcc2, DCC128, ROCO, OFF);
        DCCPacket::nonLinearAddressing = true;
        DCCPacket::trntFormat = trnFmt;
    } else {
        _dcc.setup(_pinDcc, _pinDcc2, DCC128, trnFmt, OFF);
    }
}

// ─────────────────────────────────────────────────────────────
//  loop() — DCC scheduler update and dirty loco polling (core0)
//
//  Calls the DCC library update to generate DCC packets on the
//  track pins. Processes incoming CommandBus commands from core1.
//  Sends updated speed and function packets for any locos that
//  have been marked dirty in the LocoRepository.
//  Current monitor is disabled (DRV8871 hardware limitation).
// ─────────────────────────────────────────────────────────────
void DccHal::loop() {
    _dcc.update();
    CommandBus::instance().poll();
    _processDirtyLocos();
    _checkCdeShort();   // CDE booster short-circuit detection via GP05
    // gDccCurrentMonitor.loop();  // disabled — hardware not suitable (DRV8871)
}

// ─────────────────────────────────────────────────────────────
//  _onCommand() — static CommandBus handler (core0)
//
//  Dispatches each CommandBus message to the appropriate DccHal
//  method. Static because the CommandBus cannot pass a this pointer.
//  Uses the _instance pointer set in begin().
// ─────────────────────────────────────────────────────────────
void DccHal::_onCommand(const Command& cmd) {
    if (!_instance) return;
    switch (cmd.type) {
        case CmdType::LOCO_SPEED:
            _instance->setSpeed(cmd.address, cmd.speed,
                                 cmd.forward, cmd.stepMode); break;
        case CmdType::LOCO_FUNCTION:
            _instance->setFunction(cmd.address, cmd.fn, cmd.fnState); break;
        case CmdType::LOCO_ESTOP:
            // Individual loco stop uses speed step 1 (emergency stop DCC packet)
            if (cmd.address == 0) _instance->emergencyStop();
            else _instance->setSpeed(cmd.address, 1, true, 4);
            break;
        case CmdType::ACCESSORY_SET:
            _instance->setAccessory(cmd.address, cmd.output, cmd.active); break;
        case CmdType::POWER_ON:       _instance->setPower(true);   break;
        case CmdType::POWER_OFF:      _instance->setPower(false);  break;
        case CmdType::EMERGENCY_STOP: _instance->emergencyStop();   break;
        case CmdType::CV_READ:        _instance->cvRead(cmd.cv);   break;
        case CmdType::CV_WRITE:       _instance->cvWrite(cmd.cv, cmd.cvValue); break;
        case CmdType::POM_WRITE:      _instance->pomWrite(cmd.address, cmd.cv, cmd.cvValue); break;
        case CmdType::POM_READ:       _instance->pomRead(cmd.address, cmd.cv); break;
        default: break;
    }
}

// ─────────────────────────────────────────────────────────────
//  setPower() — turn track power on or off
//
//  Updates the DCC library power state, drives the H-bridge enable
//  pin, publishes a POWER_ON/OFF event on the EventBus, and updates
//  the OLED display and status LED.
// ─────────────────────────────────────────────────────────────
void DccHal::setPower(bool on) {
    _power = on;
    // _dcc.setpower(..., true) already calls notifyRailpower() itself
    // (synchronously — see DCCPacketScheduler::setpower() in the
    // library), which in turn publishes the EventBus event. An extra
    // publish() here would therefore be a GUARANTEED double
    // publication on every toggle — removed.
    _dcc.setpower(on ? ON : OFF, true);
    // GP4 (_pinEn) is NO LONGER a manual H-bridge enable on this
    // hardware revision (the H-bridge is driven purely via DCC +
    // inverted DCC on GP2/GP3). GP4 is now a third, PIO-driven DCC
    // output without RailCom, used as RailSync for LocoNet boosters
    // (see begin() — enable_additional_DCC_output()). A digitalWrite()
    // here would disturb that PIO drive, so it is deliberately omitted.
    gOled.showPower(on);
    if (on) gLeds.onPowerOn();
    else    gLeds.onPowerOff();
}

// ─────────────────────────────────────────────────────────────
//  emergencyStop() — broadcast DCC emergency stop to all locos
//
//  Sends the DCC broadcast emergency stop packet and publishes
//  an EMERGENCY_STOP event so protocol modules can inform clients.
// ─────────────────────────────────────────────────────────────
void DccHal::emergencyStop() {
    _dcc.eStop();
    EventBus::instance().publish(Ev::estop(ModuleId::DCC_HAL));
}

// ─────────────────────────────────────────────────────────────
//  _scaleSpeed() — rescale a canonical speed to the library's range
//
//  Command.speed / LocoBase.speed are, everywhere in this codebase,
//  the canonical value: 0=stop, 1=emergency stop, 2-127=126 logical
//  steps — REGARDLESS of the number of DCC speed steps.
//
//  IMPORTANT (RCN-212): 14-step and 28-step are NOT symmetric!
//  - 14-step: the "intermediate step" bit (bit4 of the command byte)
//    is reserved for FL on 14-step decoders, so there is no doubling:
//    0=stop, 1=emergency stop, 2-15=14 running steps (single
//    reservation).
//  - 28-step: that same bit position IS used here as an extra speed
//    bit (the "U" bit), which means stop and emergency stop EACH have
//    TWO raw values (depending on U): 0 or 1 = stop, 2 or 3 =
//    emergency stop. Only from raw value 4 onward does the first
//    genuine running step begin, up to and including 31 (28 steps).
//    See RCN-212: "G-GGGG = U-0000 → Halt, U-0001 → Nothalt,
//    0-0010 → first running step, 1-1111 → full speed."
//
//  DCCPacketScheduler::setSpeed14()/setSpeed28() only shuffle bits —
//  they have no stop/estop semantics of their own, and therefore
//  expect US to already supply these RCN-212 values correctly.
//  Without this reservation, a low, valid running step for 28-step
//  sometimes ended up at raw value 2 or 3, which the decoder (and any
//  RCN-212-compliant monitor) reads as an emergency stop instead of a
//  low running step.
// ─────────────────────────────────────────────────────────────
uint8_t DccHal::_scaleSpeed(uint8_t speed, uint8_t mode) {
    if (mode == 0) {
        // 14-step: single reservation
        if (speed <= 1) return speed;
        const uint16_t range = 127 - 2, span = 15 - 2;
        uint16_t s = 2 + ((uint16_t)(speed - 2) * span + range / 2) / range;
        return (uint8_t)(s > 15 ? 15 : s);
    } else {
        // 28-step: double reservation per RCN-212
        if (speed == 0) return 0;               // stop
        if (speed == 1) return 2;               // emergency stop
        const uint16_t range = 127 - 2, span = 31 - 4;
        uint16_t s = 4 + ((uint16_t)(speed - 2) * span + range / 2) / range;
        return (uint8_t)(s > 31 ? 31 : s);
    }
}

// ─────────────────────────────────────────────────────────────
//  setSpeed() — set loco speed and direction
//
//  addr:  DCC address (1-9999 short, 1-9999 long)
//  speed: canonical value 0-127 (0=stop, 1=emergency stop,
//         2-127=126 logical steps), regardless of step mode — see
//         _scaleSpeed() above for the conversion to the library's
//         range for each step mode.
//  fwd:   true=forward, false=reverse
//  mode:  0=14 steps, 2=28 steps, other=128 steps
//
//  The DCC speed byte is: bit7=direction, bits6-0=speed. The library
//  handles the internal DCC bit-shuffling.
// ─────────────────────────────────────────────────────────────
void DccHal::setSpeed(uint16_t addr, uint8_t speed, bool fwd, uint8_t mode) {
    if (!_power) return;   // don't send DCC packets while power is off
    uint8_t scaled;
    switch (mode) {
        case 0:  scaled = _scaleSpeed(speed, 0); break;  // 14 steps
        case 2:  scaled = _scaleSpeed(speed, 2); break;  // 28 steps
        default: scaled = speed;                  break;  // 128 steps: unchanged
    }
    uint8_t dccSpd = (fwd ? 0x80 : 0x00) | (scaled & 0x7F);
    switch (mode) {
        case 0:  _dcc.setSpeed14 (addr, dccSpd); break;
        case 2:  _dcc.setSpeed28 (addr, dccSpd); break;
        default: _dcc.setSpeed128(addr, dccSpd); break;
    }
}

// ─────────────────────────────────────────────────────────────
//  setFunction() — set one loco function on or off
//
//  fn:    function number 0-68 (F0=headlight, F1-F28=aux functions)
//  state: true=on, false=off
// ─────────────────────────────────────────────────────────────
void DccHal::setFunction(uint16_t addr, uint8_t fn, bool state) {
    if (!_power) return;   // don't send DCC packets while power is off
    _dcc.setLocoFunc(addr, state ? 1 : 0, fn);
}

// ─────────────────────────────────────────────────────────────
//  setAccessory() — send a turnout (basic accessory) DCC packet
//
//  addr:   logical output address (1-based, as used by Z21/XpressNet)
//  output: RCN-213 R-bit convention, passed straight through:
//          0 = thrown/diverging (afgebogen), 1 = straight/closed (rechtdoor)
//          "Bei Standardanwendungen bedeutet R=0 Weiche auf Abzweig...
//          und R=1 Weiche gerade" (RCN-213). This is now a direct 1:1
//          mapping — output IS the R-bit, no inversion. (Previously
//          inverted here, which combined with a second, since-fixed
//          inversion in the XpressNet 0x52 handler caused reversed
//          turnout switching for standard XpressNet clients.)
//  active: true=coil energised (pulse on), false=coil off
//
//  Address conversion (linear/non-linear) happens in getBitstream().
//  Pulses the DCC traffic LED and publishes an ACCESSORY_STATE event.
// ─────────────────────────────────────────────────────────────
void DccHal::setAccessory(uint16_t addr, uint8_t output, bool active) {
    bool state = (output != 0);  // output IS the R-bit directly (RCN-213)
    _dcc.setBasicAccessoryPos(addr, state, active);
    gLeds.onDccTraffic();
    EventBus::instance().publish(
        Ev::accessoryState(ModuleId::DCC_HAL, addr, output, active));
}

// ─────────────────────────────────────────────────────────────
//  CV and POM programming — thin wrappers around the DCC scheduler
// ─────────────────────────────────────────────────────────────

// Read a CV from the programming track (service mode, requires ACK)
void DccHal::cvRead(uint16_t cv)             { _dcc.opsReadDirectCV(cv); }

// Write a CV to the programming track (service mode)
void DccHal::cvWrite(uint16_t cv, uint8_t v) { _dcc.opsProgDirectCV(cv, v); }

// Write a CV via POM (programming on main — no readback, no service mode)
void DccHal::pomWrite(uint16_t a, uint16_t cv, uint8_t v) { _dcc.opsProgramCV(a, cv, v); }

// Read a CV via POM (requires BiDi/RailCom support in the decoder)
void DccHal::pomRead(uint16_t a, uint16_t cv) { _dcc.opsPOMreadCV(a, cv); }

// ─────────────────────────────────────────────────────────────
//  _processDirtyLocos() — send DCC packets for changed locos
//
//  Iterates the LocoRepository for entries marked dirty.
//  For speed-dirty locos: sends a new speed/direction packet.
//  For fn-dirty locos: sends F0-F4, F5-F8, F9-F12, F13-F20 and
//  F21-F28 packets to fully refresh all function states.
//
//  Function bit mapping (internal → library):
//    Internal:  bit0=F0, bit1=F1, ..., bit28=F28
//    Library setFunctions0to4 expects: bit4=F0, bit3=F4, bit2=F3,
//    bit1=F2, bit0=F1 (F0 in the high bit, F1-F4 shifted down)
// ─────────────────────────────────────────────────────────────
void DccHal::_processDirtyLocos() {
    if (!_power) return;   // don't send DCC packets while power is off
    bool hadTraffic = false;

    for (;;) {
        bool speedDirty = false, fnDirty = false;
        uint16_t address = 0;
        uint8_t  speed = 0, stepMode = 0;
        bool     forward = false;
        uint32_t fn = 0;
        bool     gotOne = false;

        if (!locoRepo().lockRead(500)) break;
        LocoBase* loco = locoRepo().nextDirty(&speedDirty, &fnDirty);
        if (loco) {
            gotOne   = true;
            address  = loco->address;
            speed    = loco->speed;
            stepMode = loco->stepMode;
            forward  = loco->forward;
            fn       = loco->functions;
        }
        locoRepo().unlock();
        if (!gotOne) break;

        if (speedDirty) {
            setSpeed(address, speed, forward, stepMode);
            hadTraffic = true;
        }
        if (fnDirty) {
            // Remap F0-F4 from internal bit layout to library byte layout
            uint8_t f0to4 = (((fn >> 0) & 1) << 4)   // F0 → bit4
                          | (((fn >> 4) & 1) << 3)   // F4 → bit3
                          | (((fn >> 3) & 1) << 2)   // F3 → bit2
                          | (((fn >> 2) & 1) << 1)   // F2 → bit1
                          | (((fn >> 1) & 1) << 0);  // F1 → bit0
            _dcc.setFunctions0to4  (address, f0to4);
            _dcc.setFunctions5to8  (address, (fn >>  5) & 0x0F);
            _dcc.setFunctions9to12 (address, (fn >>  9) & 0x0F);
            _dcc.setFunctions13to20(address, (fn >> 13) & 0xFF);
            _dcc.setFunctions21to28(address, (fn >> 21) & 0xFF);
            hadTraffic = true;
        }
    }
    if (hadTraffic) gLeds.onDccTraffic();
}

// ─────────────────────────────────────────────────────────────
//  isInServiceMode() — check if the DCC scheduler is in service mode
//
//  Returns true while service mode (programming track) packets are
//  being repeated. Used by DccCurrentMonitor to gate ACK detection.
// ─────────────────────────────────────────────────────────────
bool DccHal::isInServiceMode() const {
    return _dcc.isInServiceMode();
}

// ─────────────────────────────────────────────────────────────
//  notifyAck() — report a valid decoder ACK to the scheduler
//
//  Called by DccCurrentMonitor when it detects a valid ACK pulse.
//  Tells the scheduler that a CV read or write was acknowledged.
//  Currently not called (current monitor is disabled).
// ─────────────────────────────────────────────────────────────
void DccHal::notifyAck() {
    _dcc.setAckReceived();
}

// ─────────────────────────────────────────────────────────────
//  _checkShortCircuit() — short circuit detection (disabled)
//
//  Replaced by DccCurrentMonitor. Left as placeholder.
// ─────────────────────────────────────────────────────────────

// ═══════════════════════════════════════════════════════════════
//  DCCPacketScheduler notify callbacks
//
//  These global functions are called by the DCCPacketScheduler
//  library in response to DCC events. They publish the information
//  as events on the EventBus so protocol modules can inform clients.
// ═══════════════════════════════════════════════════════════════

// Called when the library changes track power state (ON/OFF)
void notifyRailpower(uint8_t state) {
    EventBus::instance().publish(Ev::power(ModuleId::DCC_HAL, state == ON));
}

// Called when loco state is received (e.g. via RailCom acknowledgement).
// Function state is ignored here — we track functions ourselves via the
// LocoRepository to avoid the library's different bit encoding.
void notifyLokAll(uint16_t addr, uint8_t steps, uint8_t speed,
                  uint8_t f0, uint8_t f1, uint8_t f2, uint8_t f3) {
    (void)steps; (void)f0; (void)f1; (void)f2; (void)f3;
    bool fwd = (speed & 0x80) != 0;
    uint8_t spd = speed & 0x7F;
    locoRepo().setSpeed(addr, spd, fwd);
    if (locoRepo().lockRead(200)) {
        const LocoBase* loco = locoRepo().find(addr);
        uint32_t fn = loco ? loco->functions : 0;
        locoRepo().unlock();
        EventBus::instance().publish(
            Ev::locoState(ModuleId::DCC_HAL, addr, spd, fwd, fn));
    }
}

// Called when a CV value is confirmed by the programming track ACK
void notifyCVVerify(uint16_t cv, uint8_t value) {
    EventBus::instance().publish(Ev::cvOk(ModuleId::DCC_HAL, cv, value));
}

// Called when a CV value is read via POM (programming on main)
void notifyCVPOMRead(uint16_t cvAddr, uint8_t value) {
    EventBus::instance().publish(Ev::pomOk(ModuleId::DCC_HAL, 0, cvAddr, value));
}

// Called when no ACK is received from a decoder (CV not found)
void notifyCVNack(uint16_t cv) {
    (void)cv;
    EventBus::instance().publish(Ev::cvNofound(ModuleId::DCC_HAL));
}

// Called by the scheduler to read the current sense ADC value.
// Returns 0 if no instance is available (monitor disabled).
uint16_t notifyCurrentSense() {
    return _instance ? (uint16_t)analogRead(_instance->_pinAck) : 0;
}
