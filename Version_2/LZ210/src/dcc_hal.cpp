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
//  - Short circuit (HBridgeFault) and ACK (DccCurrentMonitor) detection
//    on the three onboard v2 H-bridges — see hbridge_fault.h and
//    dcc_current_monitor.h
//
//  Turnout addressing is configured via the web interface:
//  - dcc.swschema: ROCO (offset 3) / IB (offset 7) / LENZ (non-linear)
//  - dcc.swformat: Linear / Non-linear (RCN-213)
//  Address conversion is performed inside DCCPacket::getBitstream().
//
//  The DCCPacketScheduler notify callbacks (notifyRailpower etc.) are
//  called by the library and publish events on the EventBus.
// ═══════════════════════════════════════════════════════════════

#include "dcc_hal.h"
#include "trace_log.h"
#include "oled_display.h"
#include "eeprom_store.h"
#include "led_status.h"
#include "xpressnet_handler.h"  // for gCentrale (trackPowerOff/emergencyStop)

// Global instance used by all modules and the notify callbacks
DccHal gDccHal;

// Static pointer for use in notify callbacks (no 'this' pointer available)
static DccHal* _instance = nullptr;

// ─────────────────────────────────────────────────────────────
//  _checkCdeShort() — poll the CDE E-signal voltage and act on it
//
//  ADC-based, not digital: see CDE_SHORT_MV's comment in dcc_hal.h for
//  why. Reads _pinCde via analogRead(), averaged over CDE_ADC_SAMPLES
//  samples (plus one mandatory throwaway read first, to let the
//  RP2350's single, multiplexed ADC core settle onto this pin after
//  whichever other channel it last sampled — see DccCurrentMonitor::
//  loop()'s own comment for the same, previously-confirmed issue),
//  and converts to millivolts for a readable, directly-configurable
//  threshold.
//
//  Called every loop() iteration. Does nothing if track power is
//  already off (nothing to protect), or if not currently in a fault
//  window (_cdeFaultSinceUs == 0, i.e. the last sample seen was above
//  threshold and stayed that way).
//
//  If the current reading is ABOVE threshold, this may be a genuine
//  recovery, or one of the brief spikes seen while a fault is still
//  ongoing (Rob, via LSA: a real short circuit isn't continuously,
//  perfectly low-voltage — brief upward spikes recur roughly every
//  ~100µs, each ~1µs wide, throughout the fault). Only treated as a
//  genuine recovery — clearing _cdeFaultSinceUs — once the reading has
//  stayed above threshold for at least sys.cde_clear_us (falling back
//  to CDE_CLEAR_US); until then, _cdeFaultSinceUs (the fault window's
//  original start time) is left untouched, so the cumulative low-
//  voltage time survives these spikes instead of being reset by each
//  one.
//
//  If the current reading is BELOW threshold but for less than the
//  configured duration (sys.cde_short_us, falling back to
//  CDE_SHORT_US), this is still within the normal square-wave low
//  phase, so nothing is done yet.
//
//  Once the reading has been continuously below threshold for longer
//  than that duration, this is treated as a genuine short circuit: the
//  fault window is cleared (so this only fires once per short-circuit
//  event), and the configured response mode (EEPROM "sys.cde_mode") is
//  checked:
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

    // Whether a CDE booster is actually connected at all — EEPROM
    // "sys.cde_enable" (web interface, Systeem section), default true.
    // With no booster connected, the E-signal input floats/reads a
    // permanently low, undefined voltage — this ADC-based detection
    // would then read "below threshold" continuously and immediately
    // trip a false short-circuit, forever, with nothing ever able to
    // clear it (confirmed, Rob). Rather than guessing at a "floating"
    // voltage signature to distinguish "no booster" from "genuine
    // short" automatically, this is simply an explicit setting: turn
    // detection off entirely when there genuinely is no booster wired
    // up to detect a short circuit from in the first place.
    if (!eepromStore().getBool("sys.cde_enable", true)) return;

    // Multiple throwaway reads, not just one — this specific node has a
    // notably high source impedance (~34K Thevenin, via R704/R705),
    // and the RP2350's ADC sample-and-hold capacitor needs more than a
    // single acquisition cycle to fully charge against a source this
    // high-impedance. Confirmed via TraceLog (Rob): a single throwaway
    // read left the "real" readings consistently and substantially
    // UNDER the true voltage (~995-1078mV logged, against an expected
    // ~1850-2050mV range) — an undercharged sample-and-hold reads low,
    // not noisy/random, which is exactly this pattern. Several
    // throwaway reads in a row let the capacitor progressively settle
    // closer to the true voltage before the real, averaged reads are
    // taken.
    for (uint8_t i = 0; i < CDE_ADC_SETTLE_READS; i++) (void)analogRead(_pinCde);
    uint32_t sum = 0;
    for (uint8_t i = 0; i < CDE_ADC_SAMPLES; i++) sum += analogRead(_pinCde);
    uint16_t mv = (uint16_t)((sum * 3300UL) / (CDE_ADC_SAMPLES * 4095UL));

    uint16_t thresholdMv = eepromStore().getUint16("sys.cde_short_mv", CDE_SHORT_MV);
    bool belowThreshold = (mv < thresholdMv);

    // TEMP DIAGNOSTIC (Rob) — only logs on a threshold-crossing
    // transition (not every poll), so a multi-second manual test
    // doesn't flood TraceLog's ring buffer.
    static bool lastBelow = false;
    if (belowThreshold != lastBelow) {
        lastBelow = belowThreshold;
        traceLog().logf(TraceLevel::INFO, TraceSource::CDE, "mv=%u threshold=%u below=%d faultSinceUs=%lu",
                         mv, thresholdMv, belowThreshold,
                         (unsigned long)_cdeFaultSinceUs);
    }

    if (!belowThreshold) {
        if (_cdeFaultSinceUs == 0) return;  // not in a fault window anyway

        // Reading is above threshold WHILE a fault window is open: could
        // be a genuine recovery, or one of the brief spikes described
        // above. Only accept it as a recovery once it has stayed above
        // threshold for at least the configured clear-debounce.
        if (_cdeRecoverSinceUs == 0) { _cdeRecoverSinceUs = micros(); return; }
        uint16_t clearUs = eepromStore().getUint16("sys.cde_clear_us", CDE_CLEAR_US);
        if ((micros() - _cdeRecoverSinceUs) >= clearUs) {
            traceLog().logf(TraceLevel::WARNING, TraceSource::CDE, "CLEARED after %lu us above threshold", (unsigned long)clearUs);
            _cdeFaultSinceUs   = 0;
            _cdeRecoverSinceUs = 0;
        }
        return;
    }

    // Reading is below threshold: cancel any pending recovery — this
    // spike didn't lead to a genuine, sustained recovery after all —
    // and start (or continue) the fault window.
    _cdeRecoverSinceUs = 0;
    if (_cdeFaultSinceUs == 0) { _cdeFaultSinceUs = micros(); return; }

    uint16_t thresholdUs = eepromStore().getUint16("sys.cde_short_us", CDE_SHORT_US);
    if ((micros() - _cdeFaultSinceUs) < thresholdUs) return;  // still within threshold

    // Below threshold continuously for longer than the duration threshold — short circuit detected.
    _cdeFaultSinceUs = 0;

    uint8_t mode = eepromStore().getUint8("sys.cde_mode", 0);
    // sys.cde_mode:
    //   0 = turn station off: power-off + LocoNet GPOFF (default, safest)
    //   1 = booster handles it locally: station stays on, booster deals with it itself
    if (mode == 1) return;

    // Mode 0: turn the station off
    traceSerial.printf("[%lu] CDE kortsluit gedetecteerd (mv=%u) — power off\n", millis(), mv);
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
//  _checkHBridgeFault() — v2 hardware H-bridge nFAULT response
//
//  Called once per bridge from loop(), only when that bridge's
//  HBridgeFault::check() just reported a new fault edge (see
//  hbridge_fault.h). Unlike the CDE E-signal check above (which has
//  a configurable "let the booster handle it locally" mode via
//  sys.cde_mode), a genuine nFAULT from the bridge's own silicon
//  (overcurrent, overtemperature, or supply/charge-pump undervoltage
//  — see hbridge_fault.h) is always treated as serious: there is no
//  "log only" option here. Logs which bridge faulted, updates
//  gCentrale's bookkeeping, dispatches POWER_OFF the normal way (so
//  protocol modules inform their clients), turns off track power
//  directly, and lights the short-circuit status LED.
// ─────────────────────────────────────────────────────────────
void DccHal::_checkHBridgeFault(HBridgeFault& fb) {
    if (!_power) return;  // already off, nothing to do

    traceSerial.printf("[%lu] H-brug fault (%s) gedetecteerd — power off\n",
                   millis(), fb.name());
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
//  _pinCde (the CDE booster E-signal input) needs no pinMode()/
//  attachInterrupt() setup here — it is read via analogRead() in
//  _checkCdeShort() (polled every loop() iteration), which configures
//  the pin for ADC use internally. See CDE_SHORT_MV's comment in
//  dcc_hal.h for why this replaced an earlier digital, interrupt-
//  driven approach entirely: the E-signal's fault voltage (~1.8-1.9V)
//  turned out to sit inside the RP2350's undefined digital input
//  region, which digitalRead() could not reliably distinguish from
//  normal operation regardless of timing/debounce tuning. (Note: GP4/
//  _pinEn is deliberately NOT configured as a manual enable pin on
//  this hardware revision — see setPower() and the
//  enable_additional_DCC_output() call below for how it is actually
//  used.)
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
//  v2 hardware: also initialises the three HBridgeFault and three
//  DccCurrentMonitor instances (see hbridge_fault.h/
//  dcc_current_monitor.h).
// ─────────────────────────────────────────────────────────────
void DccHal::begin() {
    _instance = this;

    // v2 hardware: the four rail-power enable signals (DCC main, CDE,
    // LocoNet booster, programming track). All start LOW/disabled —
    // track power is off at boot until an explicit power-on command
    // — and are subsequently driven together by setPower() (see its
    // comment for the full lifecycle, including why HW_SM_ACTIVE
    // follows power-on/off too rather than staying off by default).
    pinMode(HW_DCC_ACTIVE, OUTPUT); digitalWrite(HW_DCC_ACTIVE, LOW);
    pinMode(HW_CDE_ACTIVE, OUTPUT); digitalWrite(HW_CDE_ACTIVE, LOW);
    pinMode(HW_LN_ENABLE,  OUTPUT); digitalWrite(HW_LN_ENABLE,  LOW);
    pinMode(HW_SM_ACTIVE,  OUTPUT); digitalWrite(HW_SM_ACTIVE,  LOW);
    _serviceModeRequested = false;
    _serviceModeStartMs   = 0;

    // v2 hardware: the three onboard H-bridges' low-active nFAULT
    // pins (short-circuit/overcurrent detection).
    _faultDcc.begin();
    _faultSm.begin();
    _faultCde.begin();

    // v2 hardware: the three onboard H-bridges' analogue SENSE lines
    // (decoder ACK detection).
    _senseDcc.begin();
    _senseSm.begin();
    _senseCde.begin();

    _applySwitchMode();

    // Third, PIO-driven DCC output without RailCom on GP4, used as
    // RailSync for LocoNet boosters. Must be called AFTER the
    // scheduler has been initialised (_applySwitchMode() → _dcc.setup()).
    _dcc.enable_additional_DCC_output(_pinEn);

    _power  = false;
    _initOk = true;

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
//
//  v2 hardware: also checks all three H-bridges' nFAULT pins
//  (short-circuit/overcurrent — see hbridge_fault.h) and drives all
//  three H-bridges' ACK detection (see dcc_current_monitor.h).
//
//  v2 hardware: also handles the service-mode H-bridge output
//  routing and its timeout. There is only one shared DCC bitstream
//  generator feeding the DCC main, CDE, and LocoNet-booster
//  H-bridge outputs, plus the dedicated programming-track output —
//  so while a genuine service-mode operation (dedicated
//  programming-track CV read/write — NOT Programming On Main, which
//  needs no output switching at all, see setPower()'s comment)
//  is in progress, HW_DCC_ACTIVE/HW_CDE_ACTIVE/HW_LN_ENABLE are
//  temporarily disabled, leaving only the programming track
//  (HW_SM_ACTIVE) carrying that packet stream — otherwise
//  service-mode packets, which are not addressed to any one loco,
//  would reach every loco on the whole layout simultaneously.
//
//  Unlike an earlier design, this is no longer driven by polling
//  isInServiceMode() every cycle — that value turned out to
//  fluctuate rapidly (multiple times per second) WITHIN a single
//  genuine session, causing the H-bridge outputs to oscillate on/off
//  throughout the session instead of switching cleanly once at the
//  start and once at the end, which very likely disrupted the verify
//  sequence itself. The switch is now explicit and one-shot instead:
//  cvRead()/cvWrite() switch to service-mode routing immediately when
//  a session starts, and notifyCVVerify()/notifyCVNack() (a definite
//  result) restore normal routing via _exitServiceMode() — see
//  _serviceModeRequested's comment in dcc_hal.h for the full history.
//
//  The timeout below is the safety net for the case neither callback
//  ever fires at all (no decoder present, no ACK, etc.): if a session
//  has been open longer than SERVICE_MODE_TIMEOUT_MS (EEPROM
//  "dcc.svc_timeout_ms"), it is force-closed via
//  _exitServiceMode(false, 0, 0) — otherwise HW_DCC_ACTIVE/
//  HW_CDE_ACTIVE/HW_LN_ENABLE, and the rest of the layout, would stay
//  dark indefinitely.
// ─────────────────────────────────────────────────────────────
void DccHal::loop() {

    _dcc.update();

    CommandBus::instance().poll();

    _processDirtyLocos();

    _checkCdeShort();   // CDE booster short-circuit detection via GP05

    // v2 hardware: nFAULT (short-circuit/overcurrent) on all three
    // bridges. Every debounced fault edge is logged via TraceLog
    // (previously this comment's own claim was aspirational, not
    // actually true — an isolated blip that never escalated produced
    // no log line at all, due to a short-circuiting "&&" that gated
    // the log on shouldEscalate() too; fixed below, Rob: needed
    // visibility into individually-harmless blips to diagnose trains
    // repeatedly stopping/track power dropping, suspected to be too
    // many small nFAULT events even though none was escalating on its
    // own). Two independent signals can each trigger a station-wide
    // power-off:
    // repeated faults within a short window (shouldEscalate()), or
    // nFAULT observed continuously low for at least a duration
    // threshold (checkDuration()) — see HBridgeFault's comments for
    // why both are needed: with IMODE=GND (fixed off-time, automatic
    // retry, confirmed by Rob against the datasheet), a genuinely
    // persistent fault doesn't necessarily accumulate enough debounced
    // edges within the window to reach shouldEscalate()'s retry count
    // (confirmed empirically — a real short stopped escalating once
    // that check alone was in place), so checkDuration() catches it
    // independently via raw continuous-low time instead. An isolated
    // blip is logged but triggers neither escalation path. All
    // thresholds are EEPROM-configurable (web interface), shared
    // across all three bridges rather than per-bridge, to keep this
    // one tunable policy rather than three separate ones.
    uint32_t faultWindowMs   = eepromStore().getUint32("sys.fault_window_ms", 1000);
    uint8_t  faultRetryCnt   = eepromStore().getUint8("sys.fault_retry_count", 3);
    uint32_t faultDurationMs = eepromStore().getUint32("sys.fault_duration_ms", 50);
    // Recovery-confirm debounce for checkDuration() below — same
    // reasoning as CDE_CLEAR_US (dcc_hal.h)/"sys.cde_clear_us": a
    // storm of closely-spaced fault blips (Rob, TraceLog: a ~2-second
    // run of individual "fault edge" events with no escalation at
    // all) needs the pin to read genuinely HIGH for at least this long
    // before checkDuration() treats it as recovered, rather than
    // resetting on the very first HIGH sample caught between blips.
    uint32_t faultClearMs    = eepromStore().getUint32("sys.fault_clear_ms", 10);

    // check() must be captured once per bridge and reused as the gate
    // for shouldEscalate() below (not called again/separately) —
    // shouldEscalate() only looks at already-recorded history and is
    // otherwise stateless, so without this gate it would re-evaluate
    // true on every single loop() iteration for as long as the window/
    // count condition holds, re-triggering _checkHBridgeFault()
    // repeatedly instead of once at the moment a new qualifying edge
    // occurs.
    //
    // Logging every individual debounced fault edge here too — a
    // previous version only acted on (and therefore only logged) an
    // edge if it ALSO happened to satisfy shouldEscalate() in the same
    // call, via a short-circuiting "&&": an isolated blip that never
    // escalated produced no log line at all, despite this section's
    // own comment above claiming otherwise. Logged separately, before
    // the escalation check, so a genuine pattern of frequent-but-
    // individually-harmless blips (Rob: trains repeatedly stopping,
    // track power going out — suspected too many small nFAULT events
    // even though none is individually escalating) becomes visible via
    // TraceLog, one line per H-bridge (name() distinguishes DCC/SM/CDE).
    bool dccEdge = _faultDcc.check();
    bool smEdge  = _faultSm.check();
    bool cdeEdge = _faultCde.check();
    if (dccEdge) traceLog().logf(TraceLevel::INFO, TraceSource::HBRIDGE, "%s fault edge", _faultDcc.name());
    if (smEdge)  traceLog().logf(TraceLevel::INFO, TraceSource::HBRIDGE, "%s fault edge", _faultSm.name());
    if (cdeEdge) traceLog().logf(TraceLevel::INFO, TraceSource::HBRIDGE, "%s fault edge", _faultCde.name());

    if (dccEdge && _faultDcc.shouldEscalate(faultWindowMs, faultRetryCnt)) {
        traceLog().logf(TraceLevel::WARNING, TraceSource::HBRIDGE, "%s ESCALATE (repeat count within window)", _faultDcc.name());
        _checkHBridgeFault(_faultDcc);
    }
    if (_faultDcc.checkDuration(faultDurationMs, faultClearMs)) {
        traceLog().logf(TraceLevel::WARNING, TraceSource::HBRIDGE, "%s ESCALATE (continuous low duration)", _faultDcc.name());
        _checkHBridgeFault(_faultDcc);
    }

    if (smEdge && _faultSm.shouldEscalate(faultWindowMs, faultRetryCnt)) {
        traceLog().logf(TraceLevel::WARNING, TraceSource::HBRIDGE, "%s ESCALATE (repeat count within window)", _faultSm.name());
        _checkHBridgeFault(_faultSm);
    }
    if (_faultSm.checkDuration(faultDurationMs, faultClearMs)) {
        traceLog().logf(TraceLevel::WARNING, TraceSource::HBRIDGE, "%s ESCALATE (continuous low duration)", _faultSm.name());
        _checkHBridgeFault(_faultSm);
    }

    if (cdeEdge && _faultCde.shouldEscalate(faultWindowMs, faultRetryCnt)) {
        traceLog().logf(TraceLevel::WARNING, TraceSource::HBRIDGE, "%s ESCALATE (repeat count within window)", _faultCde.name());
        _checkHBridgeFault(_faultCde);
    }
    if (_faultCde.checkDuration(faultDurationMs, faultClearMs)) {
        traceLog().logf(TraceLevel::WARNING, TraceSource::HBRIDGE, "%s ESCALATE (continuous low duration)", _faultCde.name());
        _checkHBridgeFault(_faultCde);
    }

    // v2 hardware: ACK detection on all three bridges — only the one
    // actually routed to the track during service mode (currently
    // always HW_SM_ACTIVE) will see a meaningful pulse; see
    // dcc_current_monitor.cpp's loop() comment.
    _senseDcc.loop();
    _senseSm.loop();
    _senseCde.loop();

    if (_serviceModeRequested) {
        uint32_t timeoutMs = eepromStore().getUint32("dcc.svc_timeout_ms",
                                                       SERVICE_MODE_TIMEOUT_MS);
        if (millis() - _serviceModeStartMs > timeoutMs) {
            _exitServiceMode(false, 0, 0);
        }
    }
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
    // Generic command log — every Command dispatched via CommandBus,
    // from whichever protocol module originated it (cmd.sourceId),
    // logged here in full before being acted on. Combined with the
    // RX/-> XN logs already in each protocol handler, this closes the
    // loop end-to-end: raw bytes in -> XpressNetHandler -> CommandBus
    // -> here -> actual DCC output (Rob).
    traceLog().logf(TraceLevel::DEBUG, TraceSource::DCCHAL,
        "CMD type=%d src=%u addr=%u speed=%u fwd=%d step=%u fn=%u fnSt=%d out=%u act=%d cv=%u cvVal=%u",
        (int)cmd.type, cmd.sourceId, cmd.address, cmd.speed, cmd.forward,
        cmd.stepMode, cmd.fn, cmd.fnState, cmd.output, cmd.active,
        cmd.cv, cmd.cvValue);
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
        case CmdType::POWER_ON:    {
        	   digitalWrite(6,HIGH);
           _instance->setPower(true);  
				digitalWrite(6,LOW);           
            break;
        }
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
//  _setMainOutputs() — drives HW_DCC_ACTIVE, HW_CDE_ACTIVE and
//  HW_LN_ENABLE together (v2 hardware only). See the declaration in
//  dcc_hal.h for the full reasoning on why these three are grouped
//  and HW_SM_ACTIVE is handled separately.
// ─────────────────────────────────────────────────────────────
void DccHal::_setMainOutputs(bool on) {
    digitalWrite(HW_DCC_ACTIVE, on ? HIGH : LOW);
    digitalWrite(HW_CDE_ACTIVE, on ? HIGH : LOW);
    digitalWrite(HW_LN_ENABLE,  on ? HIGH : LOW);
}

// ─────────────────────────────────────────────────────────────
//  _exitServiceMode() — ends the current service-mode session
//
//  Called from three places: notifyCVVerify() (success), notifyCVNack()
//  (definite failure — no ACK), and loop()'s timeout check (no result
//  at all within SERVICE_MODE_TIMEOUT_MS / EEPROM "dcc.svc_timeout_ms").
//
//  Restores the normal-operation H-bridge outputs (only if track power
//  is still on — if setPower(false) happened mid-session, _power is
//  already false and every enable pin is already low, so touching
//  them again would incorrectly turn them back on). Also updates
//  gCentrale's programming-mode bookkeeping so
//  XpressNetHandler::handleProgResult() can report the real outcome
//  to the client — a successful CV value (0x63 0x14) or the Lenz
//  "no receiver at the programming connector / no acknowledge"
//  reply (0x61 0x13) — instead of "not ready" forever. See
//  handleProgResult() in xpressnet_handler.cpp for the reply itself.
//
//  ok=true, cv/value meaningful: successful read/verify.
//  ok=false: failure (NACK) or timeout — cv/value are ignored. The
//  protocol has no separate reply code to distinguish "decoder said
//  no" from "nothing came back at all" (timeout), so neither do we —
//  both report as 0x61 0x13.
// ─────────────────────────────────────────────────────────────
void DccHal::_exitServiceMode(bool ok, uint16_t cv, uint8_t value) {
    if (!_serviceModeRequested) return;  // already closed — nothing to do
    _serviceModeRequested = false;
    if (_power) _setMainOutputs(true);

    gCentrale.progModeActive  = false;
    gCentrale.progResultReady = true;
    gCentrale.progResultOk    = ok;
    if (ok) {
        gCentrale.pendingCvAddr  = cv;
        gCentrale.progResultData = value;
    }
}

// ─────────────────────────────────────────────────────────────
//  setPower() — turn track power on or off
//
//  Updates the DCC library power state, drives all four v2 H-bridge
//  enable signals, publishes a POWER_ON/OFF event on the EventBus,
//  and updates the OLED display and status LED.
//
//  v2 hardware note: all four rail-power enable signals
//  (HW_DCC_ACTIVE/HW_CDE_ACTIVE/HW_LN_ENABLE/HW_SM_ACTIVE) follow
//  overall track power together. HW_SM_ACTIVE (the dedicated
//  programming-track output) is included here — deliberately, unlike
//  an earlier design — specifically so a layout's isolated
//  programming siding still carries normal running power when no CV
//  operation is in progress, letting a loco simply drive on/off it
//  like any other track section. It only loses that normal power
//  temporarily, and exclusively, while a genuine service-mode CV
//  operation is actually in progress — handled separately in loop()
//  via isInServiceMode(), not here, since that can start and end
//  independently of setPower() being called. Programming On Main
//  (POM) does NOT trigger any of this switching — POM packets are
//  addressed to one specific loco like any normal operational
//  packet, so there is no "leaks to the whole layout" concern the
//  way there is for genuine (broadcast-style) service-mode packets,
//  which is exactly why this whole switching scheme exists: there is
//  only one shared DCC bitstream generator feeding every output, so
//  whichever outputs are enabled receive whatever it is currently
//  producing.
// ─────────────────────────────────────────────────────────────
void DccHal::setPower(bool on) {
    _power = on;
    // _dcc.setpower(..., true) already calls notifyRailpower() itself
    // (synchronously — see DCCPacketScheduler::setpower() in the
    // library), which in turn publishes the EventBus event. An extra
    // publish() here would therefore be a GUARANTEED double
    // publication on every toggle — removed.
    _dcc.setpower(on ? ON : OFF, true);
    _setMainOutputs(on);
    digitalWrite(HW_SM_ACTIVE, on ? HIGH : LOW);
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
//  output: RCN-213 R-bit convention as received from the protocol
//          handlers (0=thrown/diverging, 1=straight/closed) —
//          inverted before being sent as the DCC R-bit, or not,
//          depending on "sys.turnout_invert" (EEPROM, web interface).
//
//  IMPORTANT: the Z21 spec itself is explicit that the mapping from
//  this bit to a turnout's actual physical position is wiring-
//  dependent, not something the protocol can dictate ("the following
//  description deliberately omits the terms 'straight' and
//  'branching'. Instead we will speak about 'output 1' and 'output
//  2'"). There is therefore no single theoretically-correct answer
//  here — only what matches the decoders actually wired up on a given
//  layout.
//
//  History: this mapping used to unconditionally invert (a fixed,
//  hardcoded choice tuned against Rob's own layout at home) — but Rob
//  found that the TMC layout, tested separately, needs the OPPOSITE
//  convention: it worked correctly (untested at the time, but
//  presumably with no inversion, or the historical double-inversion —
//  see below) the Thursday before this inversion was added, and all
//  the tuning since then happened exclusively against the home layout.
//  A single hardcoded choice can therefore never be correct for both
//  installations at once — this is now EEPROM-configurable
//  ("sys.turnout_invert", default true, matching the existing/tuned
//  behaviour for Rob's home layout) so each LZ210 installation can
//  independently match its own decoders' wiring.
//
//  Earlier still: a genuine bug once had a SEPARATE inversion also in
//  the XpressNet 0x52 handler (handleTurnout()) that happened to
//  cancel out with this one — long since consolidated into this one
//  shared, single point (rather than duplicated across each protocol
//  handler) so there is exactly one place this ever needs to be
//  corrected/configured.
//  active: true=coil energised (pulse on), false=coil off
//
//  Address conversion (linear/non-linear) happens in getBitstream().
//  Pulses the DCC traffic LED and publishes an ACCESSORY_STATE event.
// ─────────────────────────────────────────────────────────────
void DccHal::setAccessory(uint16_t addr, uint8_t output, bool active) {
    bool invert = eepromStore().getBool("sys.turnout_invert", true);
    bool state  = invert ? (output == 0) : (output != 0);
    _dcc.setBasicAccessoryPos(addr, state, active);
    gLeds.onDccTraffic();
    EventBus::instance().publish(
        Ev::accessoryState(ModuleId::DCC_HAL, addr, output, active));
}

// ─────────────────────────────────────────────────────────────
//  CV and POM programming — thin wrappers around the DCC scheduler
// ─────────────────────────────────────────────────────────────

// Read a CV from the programming track (service mode, requires ACK).
// v2 hardware: explicitly and immediately switches the shared H-bridge
// outputs to the dedicated programming track only (HW_DCC_ACTIVE/
// HW_CDE_ACTIVE/HW_LN_ENABLE disabled, HW_SM_ACTIVE left on) — a
// one-shot decision made HERE, at the deliberate start of a session,
// not re-evaluated every loop() cycle. See _serviceModeRequested's
// comment in dcc_hal.h for why. Only actually switches if not already
// in a session (guards against a stray double CV_READ dispatch
// re-triggering the outputs mid-session).
void DccHal::cvRead(uint16_t cv) {
    if (!_serviceModeRequested) {
        _serviceModeRequested = true;
        _serviceModeStartMs   = millis();
        _setMainOutputs(false);
    }
    _dcc.opsReadDirectCV(cv);
}

// Write a CV to the programming track (service mode). See cvRead()'s
// comment — identical output-switching behaviour.
void DccHal::cvWrite(uint16_t cv, uint8_t v) {
    if (!_serviceModeRequested) {
        _serviceModeRequested = true;
        _serviceModeStartMs   = millis();
        _setMainOutputs(false);
    }
    _dcc.opsProgDirectCV(cv, v);
}

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


        if (!locoRepo().lockRead(500)) { digitalWrite(5, LOW); break; }
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
            uint8_t f0to4 = (((fn >> 0) & 1) << 4)
            | (((fn >> 4) & 1) << 3)
                | (((fn >> 3) & 1) << 2)
                | (((fn >> 2) & 1) << 1)
                | (((fn >> 1) & 1) << 0);
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

// Called when the library reports the track power state (ON/OFF).
//
// IMPORTANT: the DCCPacketScheduler library may call this callback
// repeatedly to (re)confirm the current power state as part of its own
// internal packet-generation cycle, not only on a genuine ON<->OFF
// transition (observed on v2/RP2350B hardware: notifyRailpower(ON)
// firing roughly every ~80ms for as long as track power stayed on,
// even though nothing had actually changed since the last call — not
// yet confirmed whether v1 hardware is affected the same way, or
// simply never surfaced it). Publishing an EventBus event on every
// single call, unconditionally, previously turned each of those
// re-confirmations into a fresh POWER_ON broadcast to every connected
// client — a continuous flood after a single genuine power-on command.
// Only publish when the state actually differs from the last call.
static uint8_t sLastRailpowerState = 0xFF;  // 0xFF = not yet known, forces the first real call through
void notifyRailpower(uint8_t state) {
    if (state == sLastRailpowerState) return;  // unchanged — library re-confirming, not a real transition
    sLastRailpowerState = state;
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

// Called when a CV value is confirmed by the programming track ACK.
// v2 hardware: this is a definite, successful result — ends the
// current service-mode session (restores normal H-bridge outputs,
// updates gCentrale so the client's poll gets a real reply instead of
// "not ready" forever). Guarded on _instance in case this somehow
// fires before begin() (shouldn't normally happen, but avoids a null
// dereference if it does).
void notifyCVVerify(uint16_t cv, uint8_t value) {
    if (_instance) _instance->_exitServiceMode(true, cv, value);
    EventBus::instance().publish(Ev::cvOk(ModuleId::DCC_HAL, cv, value));
}

// Called when a CV value is read via POM (programming on main)
void notifyCVPOMRead(uint16_t cvAddr, uint8_t value) {
    EventBus::instance().publish(Ev::pomOk(ModuleId::DCC_HAL, 0, cvAddr, value));
}

// Called when no ACK is received from a decoder (CV not found). v2
// hardware: this is a definite, failed result — ends the current
// service-mode session the same way notifyCVVerify() does for a
// success, just with ok=false (see _exitServiceMode()'s comment).
void notifyCVNack(uint16_t cv) {
    (void)cv;  // not currently surfaced to the client — see _exitServiceMode()'s comment
    if (_instance) _instance->_exitServiceMode(false, 0, 0);
    EventBus::instance().publish(Ev::cvNofound(ModuleId::DCC_HAL));
}

// notifyCurrentSense() is deliberately NOT implemented here — see the
// friend-declaration comment in dcc_hal.h for the full reasoning
// (this weak-linked library callback would otherwise enable the
// library's own internal, single-pin, fixed-threshold ACK-detection
// state machine; ACK detection instead runs entirely through our own
// three per-bridge DccCurrentMonitor instances).
