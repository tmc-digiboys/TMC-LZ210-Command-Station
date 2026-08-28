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
//    GP27 (pinAck)  — legacy v1 ACK/short-circuit pin, superseded on
//                     v2 hardware by the three dedicated nFAULT/SENSE
//                     pin pairs below (HBridgeFault/DccCurrentMonitor)
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
//  v2 hardware: three onboard H-bridges (DCC main, SM/programming
//  track, CDE), each with its own nFAULT pin (short-circuit/
//  overcurrent, see hbridge_fault.h) and SENSE pin (decoder ACK
//  detection, see dcc_current_monitor.h). Superseded the old,
//  single-pin GP27 approach (see the pinAck note above), which
//  targeted a DRV8871 H-bridge unsuitable for current sensing.
// ═══════════════════════════════════════════════════════════════
#pragma once
#include "hardware_config.h"
#include "module_arch.h"
#include "command_bus.h"
#include "event_bus.h"
#include "loco_repository.h"
#include "hbridge_fault.h"
#include "dcc_current_monitor.h"
#include <DCCPacketScheduler_new.h>

// ADC-based CDE short-circuit detection. Superseded the earlier
// digital (digitalRead()) approach entirely: the E-signal's voltage
// during a genuine short circuit turned out to sit at ~1.8-1.9V —
// squarely inside the RP2350's undefined digital input region (below
// a typical VIH of ~2.0V, above a typical VIL of ~0.8V), so
// digitalRead() could not reliably distinguish it from the ~2.0-3.1V
// seen during normal operation, no matter how the timing/debounce
// logic around it was tuned (confirmed, Rob: an LSA — with its own,
// more permissive threshold — showed a clean, correctly-timed low
// signal that the RP2350 itself still didn't reliably register).
//
// Default/fallback voltage threshold (millivolts): a reading below
// this is considered a short circuit. Configurable via "sys.
// cde_short_mv" (web interface, Systeem section). Sits roughly midway
// between Rob's measured normal (~2050mV) and fault (~1820-1940mV)
// voltages — Rob should tune this against his own board's actual
// measured values.
constexpr uint16_t CDE_SHORT_MV = 1950;

// Default/fallback threshold for CDE short-circuit detection: a
// continuous low-voltage phase on the E-signal longer than this is
// considered a short circuit. Configurable via the EEPROM parameter
// "sys.cde_short_us" (web interface, Systeem section) — this constant
// is only the fallback default. A normal DCC square wave produces low
// phases of ~70µs. 500µs gives a margin of roughly 7x while still
// reacting quickly to a genuine short circuit.
constexpr uint32_t CDE_SHORT_US = 500;

// Default/fallback debounce for confirming the E-signal has GENUINELY
// recovered (voltage back above CDE_SHORT_MV to stay), rather than one
// of the brief spikes seen DURING an actual, ongoing short circuit.
// Configurable via "sys.cde_clear_us" (web interface, Systeem
// section).
//
// Discovered empirically (Rob, LSA): once a genuine short is present,
// the line isn't continuously, perfectly low — brief upward spikes
// recur roughly every ~100µs, each only ~1µs wide. The ORIGINAL design
// only accounted for filtering NORMAL DCC's own ~70µs low phases (see
// CDE_SHORT_US's comment above) — it had no separate concept of "the
// line briefly went high, but that doesn't mean the fault actually
// cleared". This debounce requires the voltage to stay above threshold
// for this long before accepting a recovery, so a fault's cumulative
// low-time survives brief spikes instead of being reset by each one.
//
// This default (30µs) is a reasoned starting point — comfortably
// longer than the observed ~1µs spike width, short enough to still
// react quickly to an actual recovery — but Rob should verify/tune
// this against his board's actual measured spike width via LSA, the
// same way CDE_SHORT_US itself was tuned.
constexpr uint32_t CDE_CLEAR_US = 30;

// Number of ADC samples averaged per check (after the settle reads
// below). The RP2350's ADC is datasheet-specified at 9-9.2 ENOB
// (~5.6-6.4mV per effective step over the 3.3V range) — comfortably
// resolving the ~100-200mV gap between this circuit's normal and fault
// voltages even from a single sample, but averaging a few further
// reduces noise and the chance of landing on a documented DNL
// discontinuity at a particular ADC code.
constexpr uint8_t CDE_ADC_SAMPLES = 4;

// Number of throwaway ADC reads taken (and discarded) immediately
// before the real, averaged samples above — needed for two, separate
// reasons: (1) the RP2350's ADC is a single core multiplexed across
// every channel, so the very first read after switching to a
// different pin can carry residual charge from whatever channel was
// last sampled (see DccCurrentMonitor::loop()'s own comment for the
// same, previously-confirmed issue elsewhere in this project); (2)
// this specific node has a notably high source impedance (~34K
// Thevenin, via R704/R705), and the ADC's sample-and-hold capacitor
// needs more than one acquisition cycle to fully charge against a
// source this high-impedance — a single throwaway read was confirmed,
// via TraceLog (Rob), to leave the "real" reading substantially UNDER
// the true voltage (~995-1078mV logged against an expected
// ~1850-2050mV range: an undercharged sample-and-hold reads low, not
// noisy, which is exactly this pattern). This default (8) is a
// reasoned starting point — Rob should verify via TraceLog that the
// logged mv values now land in the expected range, and increase this
// further if they are still low.
constexpr uint8_t CDE_ADC_SETTLE_READS = 8;

// Default/fallback timeout for a genuine service-mode CV read/write
// session (dedicated programming track) — if no definite result
// (success via notifyCVVerify(), or failure via notifyCVNack()) has
// arrived within this many milliseconds of cvRead()/cvWrite() being
// called, loop() forces the session closed and reports a failure, so
// a session that never gets a result (no decoder present, no ACK
// detected, etc.) cannot leave HW_DCC_ACTIVE/HW_CDE_ACTIVE/
// HW_LN_ENABLE disabled — and the rest of the layout dark —
// indefinitely. Configurable via EEPROM key "dcc.svc_timeout_ms"
// (web interface); this constant is only the fallback default.
constexpr uint32_t SERVICE_MODE_TIMEOUT_MS = 3000;

class DccHal : public HardwareModule {
public:
    DccHal(uint8_t pinDcc  = HW_DCC_PIN,
           uint8_t pinDcc2 = HW_DCC2_PIN,
           uint8_t pinEn   = HW_DCC_RAILSYNC_PIN,
           uint8_t pinCde  = HW_CDE_PIN)
        : HardwareModule("DccHal", ModuleId::DCC_HAL, ModuleCore::CORE0)
        , _pinDcc(pinDcc), _pinDcc2(pinDcc2)
        , _pinEn(pinEn)
        , _pinCde(pinCde) {}

    // ── HardwareModule interface ───────────────────────────
    // Initialises GPIO pins, loads turnout schema from EEPROM,
    // registers the CommandBus handler
    void begin() override;

    // Updates DCC packet scheduler, processes dirty locos,
    // checks CDE short circuit detection
    void loop()  override;

    void usedPins(const uint8_t*& pins, uint8_t& count) const override {
        static uint8_t p[14];
        p[0]=_pinDcc; p[1]=_pinDcc2; p[2]=_pinEn; p[3]=_pinCde;
        p[4]=HW_DCC_ACTIVE; p[5]=HW_CDE_ACTIVE; p[6]=HW_LN_ENABLE; p[7]=HW_SM_ACTIVE;
        p[8]=HW_DCC_FAULT; p[9]=HW_SM_FAULT; p[10]=HW_CDE_FAULT;
        p[11]=HW_DCC_SENSE; p[12]=HW_SM_SENSE; p[13]=HW_CDE_SENSE;
        pins = p; count = 14;
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

    // v2 hardware: read-only access to the three H-bridges' fault/ACK
    // monitors, for the web interface status display (see
    // webserver.cpp's _panelDccHal()).
    const HBridgeFault&     faultDcc() const { return _faultDcc; }
    const HBridgeFault&     faultSm()  const { return _faultSm;  }
    const HBridgeFault&     faultCde() const { return _faultCde; }
    const DccCurrentMonitor& senseDcc() const { return _senseDcc; }
    const DccCurrentMonitor& senseSm()  const { return _senseSm;  }
    const DccCurrentMonitor& senseCde() const { return _senseCde; }

    // Returns true if the DCC scheduler is currently repeating
    // service mode packets (programming track active)
    bool isInServiceMode() const;

    // Reports a valid ACK received from a decoder to the scheduler.
    // Called by whichever DccCurrentMonitor instance detects a valid
    // ACK pulse (see dcc_current_monitor.h).
    void notifyAck();

private:
    DCCPacketScheduler _dcc;
    uint8_t _pinDcc, _pinDcc2, _pinEn, _pinCde;
    bool    _power  = false;
    bool    _initOk = false;

    // v2 hardware: explicit, deliberate service-mode session tracking
    // — replaces an earlier design that polled isInServiceMode() every
    // loop() cycle, which turned out to fluctuate rapidly WITHIN a
    // single genuine programming session (the library appears to
    // internally drop in and out of its own "service mode" flag
    // between individual verify sub-steps), causing the H-bridge
    // enable outputs to oscillate on/off every ~50-150ms throughout
    // the whole session instead of switching cleanly once at the
    // start and once at the end — very likely disruptive to the
    // verify sequence itself. Now set explicitly by cvRead()/
    // cvWrite() (session start) and cleared explicitly by
    // notifyCVVerify()/notifyCVNack() (session end, definite result)
    // or by the timeout below (session end, no result at all).
    bool     _serviceModeRequested = false;
    uint32_t _serviceModeStartMs   = 0;

    // ADC-based CDE short-circuit detection — polled, not interrupt-
    // driven (see CDE_SHORT_MV's comment in this header for why the
    // earlier digital/interrupt approach was replaced entirely). All
    // state here is only ever touched from _checkCdeShort(), called
    // every loop() iteration on core0 — no ISR involved, so plain
    // (non-volatile) members are fine.
    //   _cdeFaultSinceUs : micros() timestamp of the first sample that
    //                read below threshold in the current fault window
    //                — 0 when not currently in one. Only set on a
    //                fresh transition into "below threshold" (not
    //                already faulted), so cumulative low-time survives
    //                the brief above-threshold spikes described at
    //                CDE_CLEAR_US's comment, instead of being reset by
    //                each one.
    //   _cdeRecoverSinceUs : micros() timestamp of the most recent
    //                sample that read above threshold WHILE already in
    //                a fault window — used to confirm the voltage has
    //                stayed above threshold for at least CDE_CLEAR_US
    //                (a genuine recovery) before clearing
    //                _cdeFaultSinceUs, rather than clearing it
    //                immediately on the very first above-threshold
    //                sample.
    uint32_t _cdeFaultSinceUs    = 0;
    uint32_t _cdeRecoverSinceUs  = 0;

    // Static CommandBus handler — called on core0 for each incoming command
    static void _onCommand(const Command& cmd);

    // Checks the CDE short-circuit flag and triggers power-off if set
    void _checkCdeShort();

    // v2 hardware: drives HW_DCC_ACTIVE, HW_CDE_ACTIVE and
    // HW_LN_ENABLE together — the three "normal operation" rail-power
    // enable signals. Used both by setPower() (all three follow
    // overall track power) and by loop()'s service-mode transition
    // handling (all three get temporarily disabled while genuine
    // service-mode programming is active — see loop() for the full
    // reasoning). Deliberately does NOT touch HW_SM_ACTIVE, which
    // follows a different lifecycle (tied only to setPower(), staying
    // on throughout a service-mode session — see setPower()'s comment).
    void _setMainOutputs(bool on);

    // v2 hardware: ends the current service-mode session (called from
    // notifyCVVerify()/notifyCVNack() on a definite result, or from
    // loop() on timeout with no result at all). Restores
    // HW_DCC_ACTIVE/HW_CDE_ACTIVE/HW_LN_ENABLE (only if track power is
    // still on — no-op otherwise), clears _serviceModeRequested, and
    // updates gCentrale's programming-mode bookkeeping so
    // XpressNetHandler::handleProgResult() can report the outcome to
    // the client instead of "not ready" forever. ok=false with cv=0
    // reports a plain failure/timeout (no CV value available).
    void _exitServiceMode(bool ok, uint16_t cv, uint8_t value);

    // DCCPacketScheduler library callbacks
    friend void notifyRailpower(uint8_t state);
    friend void notifyLokAll(uint16_t, uint8_t, uint8_t,
                              uint8_t, uint8_t, uint8_t, uint8_t);
    friend void notifyCVVerify(uint16_t, uint8_t);
    friend void notifyCVPOMRead(uint16_t, uint8_t);
    friend void notifyCVNack(uint16_t);
    // notifyCurrentSense() deliberately NOT implemented: this is a
    // weak-linked library callback that, if defined, would enable the
    // library's OWN internal ACK-detection state machine (a single,
    // fixed-threshold, single-pin mechanism unaware of this board's
    // three separate H-bridges) — leaving it undefined cleanly skips
    // that whole block, so ACK detection runs entirely through our
    // own three per-bridge DccCurrentMonitor instances instead (see
    // dcc_current_monitor.h). Confirmed with Rob: the library's
    // internal detection does not work reliably on this hardware
    // anyway.

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

    // v2 hardware: the three onboard H-bridges' low-active nFAULT
    // pins (DCC/main, SM/programming track, CDE/local booster output)
    // — short-circuit/overcurrent/overtemperature detection. See
    // hbridge_fault.h for the full reasoning, including the confirmed
    // active-low polarity (TI "nFAULT" naming convention).
    HBridgeFault _faultDcc{HW_DCC_FAULT, "DCC"};
    HBridgeFault _faultSm {HW_SM_FAULT,  "SM"};
    HBridgeFault _faultCde{HW_CDE_FAULT, "CDE"};

    // Shared handler for all three HBridgeFault instances above —
    // called once per bridge from loop(). Checks for a newly detected
    // fault and, on any fault, cuts track power station-wide (unlike
    // the CDE E-signal's configurable sys.cde_mode, a genuine nFAULT
    // from the bridge's own silicon is treated as always-serious —
    // no "log only" option).
    void _checkHBridgeFault(HBridgeFault& fb);

    // v2 hardware: the three onboard H-bridges' analogue SENSE lines
    // — decoder ACK detection. See dcc_current_monitor.h for the full
    // reasoning, including the mV-based, per-bridge configurable
    // threshold (67mV default for the DCC/DRV8874 bridge, 149mV
    // default for the SM/CDE DRV8876 bridges).
    // 149→160mV: a deliberately modest increase, not aggressive — the
    // smallest genuine ACK rise observed in practice so far (~206 ADC
    // counts ≈ 166mV) leaves only a thin margin above any threshold
    // raise; going much higher risks rejecting genuine, smaller ACKs
    // rather than just filtering noise. Amplitude alone also didn't
    // cleanly separate genuine ACKs from the spurious ~2ms events Rob
    // observed (some of those had rises comparable to or larger than
    // some genuine ACKs) — duration (DCC_ACK_MIN_MS/MAX in
    // dcc_current_monitor.h) is likely the more effective lever for
    // those specifically, not this threshold.
    DccCurrentMonitor _senseDcc{HW_DCC_SENSE, "dcc", 67};
    DccCurrentMonitor _senseSm {HW_SM_SENSE,  "sm",  160, /*servesServiceMode=*/true};
    DccCurrentMonitor _senseCde{HW_CDE_SENSE, "cde", 160};

};

extern DccHal gDccHal;
