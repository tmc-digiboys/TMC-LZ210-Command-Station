#pragma once
// ═══════════════════════════════════════════════════════════════
//  dcc_current_monitor.h  —  decoder ACK detection via H-bridge
//  analogue SENSE lines
//
//  v2 hardware has three onboard H-bridges (DCC main, SM/programming
//  track, CDE), each with its own analogue SENSE line
//  (HW_DCC_SENSE/HW_SM_SENSE/HW_CDE_SENSE) — a voltage proportional
//  to the bridge's output current.
//
//  Short-circuit/overcurrent detection is now handled separately and
//  more reliably by each bridge's own low-active nFAULT pin (see
//  hbridge_fault.h) — the DRV887x chips already detect that in
//  hardware faster and more accurately than an ADC-threshold guess
//  ever could. This class is therefore scoped to exactly what the
//  SENSE line is uniquely useful for: detecting the brief current
//  pulse a decoder draws to acknowledge a service-mode CV read/write
//  packet, per the NMRA DCC specification.
//
//  Threshold values (per Rob, from board measurements):
//    DCC bridge (DRV8874): 67mV
//    SM/CDE bridges (DRV8876): 149mV
//  These depend on exactly which H-bridge chip variant is populated
//  on a given board, so each bridge's threshold is independently
//  configurable via EEPROM (web interface) — these constants are only
//  the fallback defaults.
//
//  Only one bridge is actually driving the track during a genuine
//  service-mode session (HW_SM_ACTIVE — see DccHal::_exitServiceMode()
//  and its callers), so in practice only that instance's ACK state
//  machine will see a meaningful pulse during service mode; the other
//  two simply see nothing on their (disabled) bridges, which is
//  harmless.
//
//  Runs on core0, called from DccHal::loop() once per instance.
// ═══════════════════════════════════════════════════════════════
#include <Arduino.h>
#include "module_arch.h"
#include "eeprom_store.h"

// ACK detection constants. Widened from the textbook NMRA 4-8ms to
// match the DCCPacketScheduler library's own internal ACK_SENSE_MIN/
// ACK_SENSE_MAX constants (3-14ms, see DCCPacketScheduler_new.h) —
// real decoders and this library's own (now-disabled, see
// notifyCurrentSense()'s comment in dcc_hal.h) internal ACK detection
// both tolerate this wider range. NOTE: does not by itself explain
// every rejected pulse seen in practice — Rob observed several
// consistent 2ms "FAIL" events during a real CV read, which remain
// below even this widened 3ms floor; if that pattern persists after
// this change, the floor may need lowering further, or those specific
// 2ms events may be a separate, systematic artifact (e.g. a switching
// transient from the packet itself) rather than genuine ACKs.
#define DCC_ACK_MIN_MS         3    // Minimum valid ACK duration (ms)
#define DCC_ACK_MAX_MS        14    // Maximum valid ACK duration (ms)
// Brief delay after a pulse ends (SUCCESS/FAIL) before capturing a
// new baseline — see AckState::SETTLING's comment above for why this
// is needed. Deliberately short (must stay well under the gap between
// successive bit-verify attempts, or it would itself delay/interfere
// with reading).
#define DCC_ACK_SETTLE_MS      2
// Minimum time after a genuine ACK-SUCCESS before a new rise is even
// considered — see AckState's comment below for why (Rob, via LSA
// measurement: service-mode packets repeat every 8ms exactly, so a
// genuine, distinct ACK response cannot occur more often than that;
// anything detected sooner after a valid SUCCESS must be noise from
// the same underlying electrical event, not an independent response).
// Originally set to 6ms (2ms margin below the 8ms interval), which
// turned out to still leave a vulnerable trailing window once all
// debug-print timing was removed elsewhere (Err2 returned) — that gap
// was apparently wide enough for noise from the nFAULT retry bursts
// (oscillating far faster, ~100-110us period) to still land and be
// mistaken for a new ACK attempt. Tightened to 7ms (1ms margin) to
// close most of that window while still leaving a small allowance for
// jitter before the next genuine packet's ACK opportunity.
#define DCC_ACK_REFRACTORY_MS  7
// Consecutive samples required above/below threshold before a rise or
// fall is actually acted on — see the debounce counters in the header
// below and _checkAck()'s WAIT/DETECTING cases. Added after a GPIO
// scope-probe test (Rob) showed rises being detected but NONE ever
// validating as a genuine SUCCESS on the difficult loco once all
// debug-print timing was removed — the leading theory is that this
// loco's marginal contact produces a genuinely noisy/choppy signal,
// which coarser (print-delayed) sampling had been inadvertently
// smoothing over into what looked like one valid-duration pulse, and
// faster sampling now correctly resolves into several individually
// too-short fragments instead. Requiring a few consecutive
// confirming samples before switching state is a real debounce/
// hysteresis fix for that, rather than another timing-constant guess.
// 2 is a deliberately modest starting point — enough to reject a
// single-sample glitch without eating meaningfully into the 3ms
// minimum valid pulse width's margin.
#define DCC_ACK_DEBOUNCE_SAMPLES 2
#define DCC_ADC_MAX             4095 // RP2350 ADC full scale (12-bit)
#define DCC_ADC_VREF_MV       3300 // RP2350 ADC reference voltage (mV)

// ─────────────────────────────────────────────────────────────
//  AckState — state machine for decoder ACK detection
//
//  Transitions each service-mode packet cycle through IDLE/SUCCESS/
//  FAIL → SETTLING (brief delay, letting a just-ended pulse's voltage
//  genuinely decay) → WAIT (new baseline captured, only once settled)
//  → DETECTING (voltage rise seen) → SUCCESS or FAIL (duration
//  evaluated once voltage falls back to baseline, or times out) — see
//  _checkAck() in the .cpp for the full transition logic.
//
//  SETTLING was added after observing (Rob, in practice) that a
//  genuine, large ACK pulse's voltage sometimes hadn't fully decayed
//  by the time the very next baseline was captured immediately
//  afterward — producing an artificially elevated baseline for the
//  NEXT bit-verify attempt, which could compound across consecutive
//  attempts happening close together in time (observed baselines
//  climbing from ~1250 to ~1470 across two back-to-back attempts,
//  rather than a single isolated contamination event, which is what
//  first ruled out ADC channel-switching crosstalk as the cause).
//
//  REFRACTORY PERIOD (DCC_ACK_REFRACTORY_MS): after a genuine
//  ACK-SUCCESS, a new rise is ignored for a further several
//  milliseconds — see that constant's comment for the reasoning (Rob,
//  via LSA: service-mode packets repeat roughly every 8ms, so
//  multiple "ACK" detections closer together than that cannot all be
//  genuine, independent decoder responses; they're noise from the
//  same underlying event, most likely tied to the nFAULT retry
//  bursts also observed during these windows). WAIT below checks this
//  before allowing a DETECTING transition, without otherwise changing
//  the state machine's shape.
// ─────────────────────────────────────────────────────────────
enum class AckState : uint8_t {
    IDLE       = 0,  // Not in service mode
    SETTLING   = 1,  // Brief delay after a pulse ends, before capturing a new baseline
    WAIT       = 2,  // Waiting for ACK voltage rise
    DETECTING  = 3,  // ACK detected, measuring duration
    SUCCESS    = 4,  // Valid ACK (correct duration) — notifyAck() called
    FAIL       = 5,  // Invalid ACK (too short or too long)
};

// ─────────────────────────────────────────────────────────────
//  DccCurrentMonitor — ACK detector for one H-bridge's SENSE line
// ─────────────────────────────────────────────────────────────
class DccCurrentMonitor {
public:
    // pin:             ADC-capable GPIO reading this bridge's SENSE line
    // eepromKeyPrefix:  short identifier ("dcc"/"sm"/"cde") used to
    //                   build this bridge's EEPROM key
    //                   ("<prefix>.ack_mv") and in log messages
    // defaultMv:        fallback ACK-detection threshold in millivolts
    //                    if not yet configured (67 for the DCC/DRV8874
    //                    bridge, 149 for the SM/CDE DRV8876 bridges)
    // servesServiceMode: true only for the ONE instance whose bridge
    //                    is actually routed to the track during a
    //                    genuine service-mode session (currently
    //                    always the SM/programming-track bridge — see
    //                    DccHal's output-routing comments). The other
    //                    two instances' bridges are electrically
    //                    disabled during that window (HW_DCC_ACTIVE/
    //                    HW_CDE_ACTIVE both LOW), so running their ACK
    //                    state machine would only see near-zero
    //                    readings and produce confusing "ACK-WAIT"-style
    //                    log lines with no electrical meaning — loop()
    //                    below skips _checkAck() entirely for those.
    DccCurrentMonitor(uint8_t pin, const char* eepromKeyPrefix, uint16_t defaultMv,
                       bool servesServiceMode = false)
        : _pin(pin), _keyPrefix(eepromKeyPrefix), _defaultMv(defaultMv)
        , _servesServiceMode(servesServiceMode) {}

    // Configures the RP2350 ADC peripheral for 12-bit resolution (its
    // native/maximum resolution). Safe to call across all three
    // instances — analogReadResolution() is a global peripheral
    // setting, not per-pin, so the last call wins, but every instance
    // wants 12-bit anyway.
    void begin() { analogReadResolution(12); }

    // Main update function — call once per instance from DccHal::loop().
    // Always updates the raw ADC reading (lastAdcValue(), for web
    // display); the ACK detection state machine itself only actually
    // runs while in service mode AND for the instance constructed with
    // servesServiceMode=true — see the constructor's comment above.
    void loop();

    // Returns the last raw ADC reading (0-4095)
    uint16_t lastAdcValue()    const { return _lastAdc; }

    // Returns the current state of the ACK detection state machine
    AckState ackState()        const { return _ackState; }

    // Reads this bridge's configured ACK threshold live from EEPROM
    // (key "<prefix>.ack_mv"), defaulting to _defaultMv if not yet
    // configured.
    uint16_t thresholdMv() const {
        char key[24];
        snprintf(key, sizeof(key), "%s.ack_mv", _keyPrefix);
        return eepromStore().getUint16(key, _defaultMv);
    }

    // Short identifier for this bridge ("dcc"/"sm"/"cde").
    const char* keyPrefix() const { return _keyPrefix; }

private:
    uint8_t     _pin;
    const char* _keyPrefix;
    uint16_t    _defaultMv;
    bool        _servesServiceMode; // True only for the SM instance — see constructor's comment
    uint16_t    _lastAdc      = 0;     // Most recent ADC reading
    uint16_t    _baseAdc      = 0;     // Baseline reading at start of service mode packet
    bool        _wasInService = false; // True if we were in service mode last loop
    AckState    _ackState     = AckState::IDLE;
    uint32_t    _ackStartMs   = 0;     // Timestamp when ACK voltage rise was detected
    uint32_t    _settleSinceMs = 0;    // Timestamp SETTLING was entered
    uint32_t    _lastSuccessMs = 0;    // Timestamp of the last ACK-SUCCESS (0 = none yet), used by the refractory-period check in WAIT
    uint32_t    _riseCandidateMs = 0;  // Timestamp of the FIRST above-threshold sample in the current debounce run (used as the true pulse start, not the later debounce-confirmed moment)
    uint32_t    _fallCandidateMs = 0;  // Timestamp of the FIRST below-threshold sample in the current debounce run (used as the true pulse end)
    uint8_t     _riseDebounce  = 0;    // Consecutive above-threshold samples seen so far while in WAIT
    uint8_t     _fallDebounce  = 0;    // Consecutive below-threshold samples seen so far while in DETECTING

    // Converts a millivolt value to the corresponding raw ADC count.
    static uint16_t _mvToAdc(uint16_t mv) {
        return (uint16_t)(((uint32_t)mv * DCC_ADC_MAX) / DCC_ADC_VREF_MV);
    }

    // Runs the ACK detection state machine during service mode.
    // Calls DccHal::notifyAck() on successful ACK detection.
    void _checkAck(uint16_t adc);
};
