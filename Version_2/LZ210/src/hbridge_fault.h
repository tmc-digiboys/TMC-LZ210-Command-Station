#pragma once
// ═══════════════════════════════════════════════════════════════
//  hbridge_fault.h  —  low-active H-bridge nFAULT pin monitoring
//
//  v2 hardware has three onboard H-bridges (DCC main, SM/programming
//  track, CDE), each with its own nFAULT output (HW_DCC_FAULT/
//  HW_SM_FAULT/HW_CDE_FAULT). Per the DRV8874/DRV8876 datasheets
//  (Texas Instruments — both chips share the same DRV887x nFAULT
//  behaviour), this pin is an open-drain output with an external
//  pull-up: HIGH under normal operation, pulled LOW by the chip
//  itself to indicate a fault (overcurrent, overtemperature, charge
//  pump/supply undervoltage). "n" is TI's standard prefix for
//  active-low signals — confirmed directly from the datasheet text:
//  "the nFAULT pin driven low" on a fault condition, with an
//  external "nFAULT Pullup resistor" holding it high otherwise.
//
//  HBridgeFault (below) is a small, reusable class — instantiated
//  once per H-bridge by DccHal, the same "one class, several
//  instances" pattern used elsewhere in this project (see
//  DccCurrentMonitor for the equivalent on the SENSE lines) — rather
//  than writing three nearly identical blocks of fault-checking code.
//  It only detects and debounces the raw nFAULT signal; the decision
//  of whether a fault event should actually escalate to a
//  station-wide power-off lives in shouldEscalate() below, and is
//  acted on by DccHal — see its comment for why an isolated fault
//  blip is expected and normal on this hardware, and only repeated
//  faults warrant real action.
// ═══════════════════════════════════════════════════════════════
#include <Arduino.h>

// ─────────────────────────────────────────────────────────────
//  HBridgeFault — debounced, edge-triggered monitor for one
//  H-bridge's low-active nFAULT pin.
//
//  A single, isolated nFAULT edge is not necessarily a genuine,
//  ongoing problem: the DRV887x chips have their own automatic-retry
//  overcurrent response (IMODE=GND on this board — confirmed by Rob
//  against the datasheet's Table 7-6 "IMODE Functions": fixed
//  off-time, automatic retry), which briefly disables the outputs and
//  self-recovers WITHOUT needing an external reset. A brief, isolated
//  nFAULT blip is expected and harmless in this configuration — for
//  example, the inrush current a decoder's input capacitor draws when
//  a loco first makes rail contact, or the back-EMF/inductive
//  kickback as a motor's magnetic field collapses when it stops, both
//  confirmed present in practice (Rob: faults observed both when a
//  loco is simply moved by hand on the rails, and right as another
//  loco comes to a stop) — neither is a genuine, sustained fault.
//
//  What DOES indicate a genuine problem is faults recurring
//  repeatedly within a short window (the chip's own automatic-retry
//  isn't managing to recover, or a real short/overload is
//  continuously present) — see shouldEscalate() below, which is what
//  DccHal now actually acts on (station-wide power-off), while check()
//  alone is only used for logging every individual blip.
// ─────────────────────────────────────────────────────────────
class HBridgeFault {
public:
    // pin:  the nFAULT GPIO for this bridge (HW_DCC_FAULT/
    //       HW_SM_FAULT/HW_CDE_FAULT)
    // name: short display name for logging ("DCC"/"SM"/"CDE")
    HBridgeFault(uint8_t pin, const char* name)
        : _pin(pin), _name(name) {}

    // Configures the pin as an input with an internal pull-up as a
    // safety backstop (the board already has its own external
    // pull-up per the datasheet's recommended circuit, but an
    // internal one costs nothing and protects against ever reading a
    // floating/undefined level as a false fault if that external
    // pull-up is ever missing or damaged).
    void begin() { pinMode(_pin, INPUT_PULLUP); }

    // Call every loop() iteration. Returns true exactly once per
    // fault event (edge-triggered on the transition into the fault
    // state), not continuously for as long as the pin stays low.
    // Debounced with a small number of consecutive low readings to
    // reject a single-sample glitch, though the chip's own nFAULT
    // flag is expected to already be clean. Every true return is also
    // recorded into the recent-fault history used by shouldEscalate()
    // below.
    bool check() {
        bool faultNow = (digitalRead(_pin) == LOW);
        if (!faultNow) { _wasLow = false; _lowCount = 0; return false; }
        if (_wasLow) return false;          // already reported, still ongoing
        if (++_lowCount < 3) return false;  // debounce: 3 consecutive lows
        _wasLow = true;
        _recordFault();
        return true;
    }

    // Call after check() returns true, to decide whether this fault
    // event should escalate to a full station-wide power-off (true),
    // or whether it is likely just an isolated, self-recovering blip
    // that the chip's own automatic-retry will already be handling
    // (false — no action needed beyond logging the individual event).
    // Escalates once windowCountMs milliseconds contain at least
    // retryCount recorded faults (both configurable via EEPROM —
    // see DccHal::loop() for the actual keys/defaults).
    //
    // NOTE: with IMODE=GND (fixed off-time, automatic retry — see the
    // class comment above), a genuinely persistent fault does NOT
    // necessarily hold nFAULT continuously low: the chip itself
    // cycles fault→brief pause→retry→fault again for as long as the
    // underlying problem persists. Depending on how that cycle's
    // timing lines up with check()'s debounce and this method's
    // window, a genuine ongoing short could in principle still fail
    // to accumulate enough recorded edges to reach retryCount within
    // windowMs (confirmed by Rob: a real short stopped escalating
    // after this method was introduced, even with retryCount=3, its
    // original default). checkDuration() below is a second,
    // independent signal — call BOTH and escalate on either.
    bool shouldEscalate(uint32_t windowMs, uint8_t retryCount) const {
        uint32_t now = millis();
        uint8_t count = 0;
        for (uint8_t i = 0; i < HBRIDGE_FAULT_HISTORY; i++) {
            if (_history[i] != 0 && (now - _history[i]) <= windowMs) count++;
        }
        return count >= retryCount;
    }

    // Call every loop() iteration (independently of check() — this
    // reads the raw pin directly, not the debounced/edge-tracked
    // state) to detect a fault that has been continuously present for
    // at least thresholdMs. Same pattern as _checkCdeShort()'s
    // continuous-low-duration measurement on the CDE E-signal
    // (dcc_hal.cpp) — a proven approach in this codebase — added here
    // as a second signal alongside shouldEscalate() specifically
    // because that count/window approach alone was confirmed (Rob,
    // empirically) to miss a genuine, persistent short under
    // automatic-retry: some fault conditions (UVLO, thermal shutdown,
    // or a retry cycle time that doesn't reliably clear between
    // debounced samples) hold nFAULT low for a stretch that this
    // catches directly, regardless of how the edge/count side behaves.
    // thresholdMs is EEPROM-configurable (see DccHal::loop()) —
    // Rob may need to tune this empirically (LSA/scope) against the
    // DRV887x's actual retry-cycle timing on this board.
    //
    // clearMs — recovery-confirm debounce, same reasoning as CDE_CLEAR_US
    // (dcc_hal.h): confirmed via TraceLog (Rob) that a genuine, sustained
    // fault condition can present as a dense STORM of brief nFAULT blips
    // (a ~2-second run of individually-debounced "fault edge" events was
    // observed with NO escalation at all, while a much shorter, ~4-edge
    // burst elsewhere DID escalate) — the original, unconditional
    // "if (!faultNow) resetToZero" meant any single HIGH sample caught
    // between two closely-spaced blips wiped out all accumulated
    // duration, so a storm of brief blips could paradoxically go
    // undetected here even though it represents a WORSE underlying
    // condition than an isolated blip. Only treats the pin as genuinely
    // recovered — resetting the duration window — once it has read HIGH
    // continuously for at least clearMs, mirroring exactly how
    // _checkCdeShort() was fixed for the same failure mode.
    bool checkDuration(uint32_t thresholdMs, uint32_t clearMs = 10) {
        bool faultNow = (digitalRead(_pin) == LOW);
        if (!faultNow) {
            if (_durationSinceMs == 0) return false;  // not in a fault window anyway
            if (_recoverSinceMs == 0) { _recoverSinceMs = millis(); return false; }
            if ((millis() - _recoverSinceMs) >= clearMs) {
                _durationSinceMs = 0;
                _recoverSinceMs  = 0;
            }
            return false;
        }
        _recoverSinceMs = 0;  // still faulted — cancel any pending recovery
        if (_durationSinceMs == 0) { _durationSinceMs = millis(); return false; }
        return (millis() - _durationSinceMs) >= thresholdMs;
    }

    // True if the pin currently reads as faulted (level, not edge —
    // for status display, e.g. the web interface).
    bool isFaulted() const { return digitalRead(_pin) == LOW; }

    // Short display name for this bridge ("DCC"/"SM"/"CDE").
    const char* name() const { return _name; }

private:
    uint8_t     _pin;
    const char* _name;
    bool        _wasLow   = false;
    uint8_t     _lowCount = 0;

    // Small ring buffer of recent fault timestamps, used by
    // shouldEscalate() above to detect repeated faults within a
    // short window. Entries older than any realistic window are
    // simply ignored by shouldEscalate()'s own age check, so this
    // never needs explicit pruning.
    static constexpr uint8_t HBRIDGE_FAULT_HISTORY = 5;
    uint32_t _history[HBRIDGE_FAULT_HISTORY] = {0};
    uint8_t  _historyHead = 0;

    // Timestamp the pin was first observed continuously low, used by
    // checkDuration() above. 0 means "not currently in a low stretch".
    uint32_t _durationSinceMs = 0;
    // Timestamp the pin was most recently observed HIGH while already
    // in a fault window (checkDuration()'s clearMs debounce). 0 means
    // "not currently tracking a pending recovery".
    uint32_t _recoverSinceMs  = 0;

    void _recordFault() {
        _history[_historyHead] = millis();
        _historyHead = (_historyHead + 1) % HBRIDGE_FAULT_HISTORY;
    }
};
