// ═══════════════════════════════════════════════════════════════
//  dcc_current_monitor.cpp  —  decoder ACK detection via H-bridge
//  analogue SENSE lines
//
//  See dcc_current_monitor.h for the full class-level explanation of
//  why this is now instantiated three times (once per v2 H-bridge),
//  why short-circuit detection moved to HBridgeFault (hbridge_fault.h),
//  and why the threshold is configured in millivolts.
// ═══════════════════════════════════════════════════════════════
#include "dcc_current_monitor.h"
#include "dcc_hal.h"

// ─────────────────────────────────────────────────────────────
//  loop() — main update function, called once per instance from
//  DccHal::loop() on core0
//
//  Reads the ADC pin every call, then:
//  - While in service mode: runs the ACK detection state machine
//  - When leaving service mode: resets ACK state to IDLE
//
//  The "leaving service mode" transition is detected via the
//  _wasInService flag: it is only set true while _checkAck() is
//  actively being called (i.e. while gDccHal.isInServiceMode() is
//  true), so the very next loop() call after service mode ends will
//  see _wasInService==true but isInServiceMode()==false, triggering
//  exactly one reset of the ACK state machine back to IDLE (and
//  clearing the flag so this reset only happens once per service-mode
//  session, not on every subsequent loop() call).
//
//  NOTE: isInServiceMode() reflects a single, global service-mode
//  flag on the shared DCCPacketScheduler (there is only one scheduler
//  driving the DCC signal, even though three H-bridges physically
//  exist). Only the bridge actually routed to the track during a
//  session (HW_SM_ACTIVE, per DccHal's service-mode output routing —
//  see _setMainOutputs()/_exitServiceMode()) can see a meaningful ACK
//  pulse; the other two bridges are electrically disabled during that
//  window, so this instance's ACK state machine is gated on
//  _servesServiceMode as well — running it for the disabled bridges
//  would only see near-zero readings and produce confusing log lines
//  with no electrical meaning (see the constructor's comment).
//  ADC channel-switching settling: the RP2350 has a single physical
//  ADC core multiplexed across all input channels (VIN/12V/5V on
//  PowerMonitor, DCC/SM/CDE SENSE here — different pins, same
//  underlying hardware). PowerMonitor runs immediately after DccHal
//  in the core0 module registration order (see LZ210.ino), so at the
//  START of each core0 cycle, the ADC mux is still sitting on
//  whichever PowerMonitor channel it last sampled at the END of the
//  PREVIOUS cycle — if the very next analogRead() here doesn't allow
//  enough settling time, the reading can carry residual charge from
//  that entirely different, much higher-magnitude channel. This
//  matches what was observed in practice: a periodic, oddly
//  consistent elevated "baseline" (~1200-1260 raw counts, in the same
//  general order of magnitude as PowerMonitor's own raw VIN/12V/5V
//  readings — see PowerMonitor::loop()'s log output) appearing during
//  otherwise-clean SM ACK detection. A throwaway first read discards
//  the contaminated sample and lets the mux settle onto THIS pin
//  before the real, used reading is taken.
// ─────────────────────────────────────────────────────────────
void DccCurrentMonitor::loop() {
    (void)analogRead(_pin);              // throwaway — let the ADC mux settle
    _lastAdc = (uint16_t)analogRead(_pin);

    if (_servesServiceMode && gDccHal.isInServiceMode()) {
        _checkAck(_lastAdc);
        _wasInService = true;
    } else {
        // Leaving service mode (or never the instance responsible for
        // it) — reset the ACK state machine
        if (_wasInService) {
            _ackState     = AckState::IDLE;
            _wasInService = false;
        }
    }
}

// ─────────────────────────────────────────────────────────────
//  _checkAck() — decoder ACK detection during service mode
//
//  Implements a state machine per the NMRA DCC specification. Uses
//  this bridge's configured threshold (thresholdMv(), converted to a
//  raw ADC count) as the minimum significant rise above baseline.
//
//  IDLE / SUCCESS / FAIL → WAIT:
//    First service mode packet received (or the previous packet's
//    ACK cycle has already concluded). Capture the current ADC
//    reading as the new baseline for this packet.
//
//  WAIT → DETECTING:
//    Voltage rises more than the threshold above baseline — a
//    decoder has begun drawing extra current to acknowledge the
//    packet. Records the start timestamp so the pulse duration can be
//    measured.
//
//  DETECTING → SUCCESS:
//    Voltage falls back to within the threshold of baseline after
//    somewhere between DCC_ACK_MIN_MS and DCC_ACK_MAX_MS milliseconds
//    have elapsed — a validly-timed ACK pulse. Calls
//    DccHal::notifyAck() to inform the DCC scheduler that the current
//    CV read/write attempt was acknowledged.
//
//  DETECTING → FAIL:
//    Either the voltage falls back to baseline too quickly (elapsed
//    time still below DCC_ACK_MIN_MS when it drops — pulse too short
//    to be a genuine ACK), or the voltage stays elevated for longer
//    than DCC_ACK_MAX_MS without falling back (too long to be a
//    normal ACK).
// ─────────────────────────────────────────────────────────────
void DccCurrentMonitor::_checkAck(uint16_t adc) {
    uint16_t deltaAdc = _mvToAdc(thresholdMv());

    switch (_ackState) {

    case AckState::IDLE:
        // No preceding pulse to settle from — capture the baseline
        // immediately.
        _baseAdc  = adc;
        _ackState = AckState::WAIT;
        break;

    case AckState::SUCCESS:
    case AckState::FAIL:
        // A pulse just ended — start the brief settling delay before
        // capturing a new baseline, rather than capturing it
        // immediately (see AckState::SETTLING's comment in the header
        // for why).
        _settleSinceMs = millis();
        _ackState      = AckState::SETTLING;
        break;

    case AckState::SETTLING:
        if (millis() - _settleSinceMs >= DCC_ACK_SETTLE_MS) {
            _baseAdc  = adc;
            _ackState = AckState::WAIT;
        }
        break;

    case AckState::WAIT: {
        // Refractory period: ignore any rise for a while after a
        // genuine ACK-SUCCESS — see DCC_ACK_REFRACTORY_MS's comment
        // for why (Rob, via LSA: packets repeat roughly every 8ms, so
        // anything detected sooner than that after a valid SUCCESS
        // cannot be a second, genuine, independent response).
        if (_lastSuccessMs != 0 && (millis() - _lastSuccessMs) < DCC_ACK_REFRACTORY_MS) {
            break;
        }
        // Debounce the rise: require DCC_ACK_DEBOUNCE_SAMPLES
        // consecutive above-threshold samples before actually
        // transitioning to DETECTING — see that constant's comment
        // for why. The candidate's timestamp (first above-threshold
        // sample, not the later confirming one) becomes _ackStartMs,
        // so the eventual duration measurement isn't skewed by the
        // debounce delay itself.
        if (adc > (_baseAdc + deltaAdc)) {
            if (_riseDebounce == 0) _riseCandidateMs = millis();
            _riseDebounce++;
            if (_riseDebounce >= DCC_ACK_DEBOUNCE_SAMPLES) {
                _ackState     = AckState::DETECTING;
                _ackStartMs   = _riseCandidateMs;
                _riseDebounce = 0;
                _fallDebounce = 0;
            }
        } else {
            _riseDebounce = 0;  // dropped back below threshold — not a real rise, reset
        }
        break;
    }

    case AckState::DETECTING: {
        if (adc <= (_baseAdc + deltaAdc)) {
            // Debounce the fall the same way — require
            // DCC_ACK_DEBOUNCE_SAMPLES consecutive below-threshold
            // samples before treating the pulse as genuinely over,
            // using the FIRST below-threshold sample's timestamp (not
            // the confirming one) as the pulse's true end.
            if (_fallDebounce == 0) _fallCandidateMs = millis();
            _fallDebounce++;
            if (_fallDebounce >= DCC_ACK_DEBOUNCE_SAMPLES) {
                uint32_t duration = _fallCandidateMs - _ackStartMs;
                if (duration >= DCC_ACK_MIN_MS && duration <= DCC_ACK_MAX_MS) {
                    _ackState = AckState::SUCCESS;
                    _lastSuccessMs = millis();  // starts the refractory period (see WAIT above)
                    gDccHal.notifyAck();  // inform the scheduler of a valid ACK
                } else {
                    _ackState = AckState::FAIL;  // wrong duration
                }
                _fallDebounce = 0;
            }
        } else {
            _fallDebounce = 0;  // back above threshold — pulse still ongoing, reset
            uint32_t duration = millis() - _ackStartMs;
            if (duration > DCC_ACK_MAX_MS) {
                // Stayed high too long — not a valid ACK
                _ackState = AckState::FAIL;
            }
        }
        break;
    }
    }
}
