// ═══════════════════════════════════════════════════════════════
//  dcc_current_monitor.cpp  —  DCC current sensing for short circuit and ACK
//
//  Reads GP27 via the RP2350 ADC and runs two state machines:
//  1. Short circuit detection: fires if current exceeds threshold
//     for DCC_SHORT_CONFIRM consecutive samples after debounce.
//  2. ACK detection: detects the 4-8ms current pulse produced by a
//     decoder acknowledging a service mode (programming track) packet.
//
//  Both functions are currently disabled at hardware level because the
//  DRV8871 H-bridge is not suitable for DCC current sensing.
//  See dcc_current_monitor.h for detailed hardware notes.
// ═══════════════════════════════════════════════════════════════
#include "dcc_current_monitor.h"
#include "dcc_hal.h"
#include "event_bus.h"
#include "eeprom_store.h"
#include "led_status.h"

// Global instance used by DccHal
DccCurrentMonitor gDccCurrentMonitor;

// ─────────────────────────────────────────────────────────────
//  begin() — configure the ADC pin and short circuit threshold
//
//  adcPin:           GPIO pin to read (must be ADC-capable, e.g. GP27)
//  shortThresholdAdc: ADC value (0-4095) above which short circuit is flagged
//
//  Stores both parameters into the corresponding member fields
//  (_pin/_shortThresh), and sets ADC resolution to 12 bits (the
//  RP2350's native resolution, giving the full 0-4095 reading range
//  that DCC_ADC_MAX and the threshold value are expressed in).
// ─────────────────────────────────────────────────────────────
void DccCurrentMonitor::begin(uint8_t adcPin, uint16_t shortThresholdAdc) {
    _pin         = adcPin;
    _shortThresh = shortThresholdAdc;
    analogReadResolution(12);  // RP2350: 12-bit ADC (0-4095)
}

// ─────────────────────────────────────────────────────────────
//  loop() — main update function, called from DccHal::loop() on core0
//
//  Reads the ADC pin every call, then:
//  - Always checks for short circuit (_checkShortCircuit)
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
// ─────────────────────────────────────────────────────────────
void DccCurrentMonitor::loop() {
    _lastAdc = (uint16_t)analogRead(_pin);

    _checkShortCircuit(_lastAdc);

    if (gDccHal.isInServiceMode()) {
        _checkAck(_lastAdc);
        _wasInService = true;
    } else {
        // Leaving service mode — reset the ACK state machine
        if (_wasInService) {
            _ackState     = AckState::IDLE;
            _wasInService = false;
        }
    }
}

// ─────────────────────────────────────────────────────────────
//  _checkShortCircuit() — short circuit detection
//
//  Disabled at compile time when DCC_SHORT_DISABLED is defined: the
//  function immediately discards its argument and returns, so none of
//  the logic below ever executes on the current hardware revision
//  (see the header's hardware note for why).
//
//  When enabled:
//  - Suppressed during ACK detection (ACK also raises current, which
//    would otherwise be misread as a short circuit)
//  - Suppressed when track power is off (resets _inShort, _shortCount
//    and _powerOnMs so the debounce/confirmation logic starts fresh
//    the next time power is turned back on)
//  - Debounced: waits DCC_SHORT_DEBOUNCE_MS after power-on before
//    checking at all (allows H-bridge inrush current to settle,
//    avoiding a false trigger right at power-on)
//  - Confirmed: requires DCC_SHORT_CONFIRM consecutive samples above
//    the threshold before triggering (filters single-sample spikes,
//    e.g. from switching noise); any single sample back below
//    threshold resets the counter, so the samples must be genuinely
//    consecutive
//  - On trigger: turns off track power immediately via
//    gDccHal.setPower(false), pulses the short-circuit status LED,
//    and publishes a SHORT_CIRCUIT event on the EventBus so other
//    modules can react (e.g. informing connected clients)
// ─────────────────────────────────────────────────────────────
void DccCurrentMonitor::_checkShortCircuit(uint16_t adc) {
#ifdef DCC_SHORT_DISABLED
    (void)adc;  // hardware not ready — short circuit detection disabled
    return;
#endif
    // Suppress during ACK detection — ACK also raises current
    if (_ackState == AckState::DETECTING) return;

    // When power is off: reset state so recovery is possible
    if (!gDccHal.getPower()) {
        _inShort    = false;
        _shortCount = 0;
        _powerOnMs  = 0;
        return;
    }

    // Wait DCC_SHORT_DEBOUNCE_MS after power-on for current to settle
    if (_powerOnMs == 0) _powerOnMs = millis();
    if (millis() - _powerOnMs < DCC_SHORT_DEBOUNCE_MS) return;

    if (adc >= _shortThresh) {
        if (!_inShort) {
            _shortCount++;
            if (_shortCount >= DCC_SHORT_CONFIRM) {
                _inShort    = true;
                _shortCount = 0;
                gDccHal.setPower(false);      // turn off track power immediately
                gLeds.onShortCircuit();        // fast-blink the status LED
                EventBus::instance().publish(
                    Ev::shortCircuit(ModuleId::DCC_HAL));
            }
        }
    } else {
        // Current back to normal — reset the confirmation counter
        _shortCount = 0;
        _inShort    = false;
    }
}

// ─────────────────────────────────────────────────────────────
//  _checkAck() — decoder ACK detection during service mode
//
//  Implements a state machine per the NMRA DCC specification:
//
//  IDLE / SUCCESS / FAIL → WAIT:
//    First service mode packet received (or the previous packet's
//    ACK cycle has already concluded). Capture the current ADC
//    reading as the new baseline current for this packet.
//
//  WAIT → DETECTING:
//    Current rises more than DCC_ACK_DELTA above baseline — a
//    decoder has begun drawing extra current to acknowledge the
//    packet. Records the start timestamp so the pulse duration can be
//    measured.
//
//  DETECTING → SUCCESS:
//    Current falls back to within DCC_ACK_DELTA of baseline after
//    somewhere between DCC_ACK_MIN_MS and DCC_ACK_MAX_MS milliseconds
//    have elapsed — a validly-timed ACK pulse. Calls
//    DccHal::notifyAck() to inform the DCC scheduler that the current
//    CV read/write attempt was acknowledged.
//
//  DETECTING → FAIL:
//    Either the current falls back to baseline too quickly (elapsed
//    time still below DCC_ACK_MIN_MS when it drops — pulse too short
//    to be a genuine ACK), or the current stays elevated for longer
//    than DCC_ACK_MAX_MS without falling back (too long to be a
//    normal ACK, and could indicate a short circuit instead).
// ─────────────────────────────────────────────────────────────
void DccCurrentMonitor::_checkAck(uint16_t adc) {
    switch (_ackState) {

    case AckState::IDLE:
    case AckState::SUCCESS:
    case AckState::FAIL:
        // Capture the baseline current for this service mode packet
        _baseAdc  = adc;
        _ackState = AckState::WAIT;
        break;

    case AckState::WAIT:
        // Wait for current to rise (ACK start)
        if (adc > (_baseAdc + DCC_ACK_DELTA)) {
            _ackState   = AckState::DETECTING;
            _ackStartMs = millis();
        }
        break;

    case AckState::DETECTING: {
        uint32_t duration = millis() - _ackStartMs;
        if (adc <= (_baseAdc + DCC_ACK_DELTA)) {
            // Current has fallen — evaluate the pulse duration
            if (duration >= DCC_ACK_MIN_MS && duration <= DCC_ACK_MAX_MS) {
                _ackState = AckState::SUCCESS;
                gDccHal.notifyAck();  // inform the scheduler of a valid ACK
            } else {
                _ackState = AckState::FAIL;  // wrong duration
            }
        } else if (duration > DCC_ACK_MAX_MS) {
            // Stayed high too long — not a valid ACK (possibly a short circuit)
            _ackState = AckState::FAIL;
        }
        break;
    }
    }
}
