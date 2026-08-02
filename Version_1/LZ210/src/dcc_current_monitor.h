#pragma once
// ═══════════════════════════════════════════════════════════════
//  DccCurrentMonitor — DCC current sensing for short circuit and ACK
//
//  Reads the ADC pin (GP27) and distinguishes between:
//  - Short circuit: current exceeds dcc.shortma threshold
//  - ACK:           current rises briefly (4-8ms) during service mode
//
//  Runs on core0, called from DccHal::loop().
//
//  HARDWARE NOTE:
//  Currently disabled (DCC_SHORT_DISABLED) because the DRV8871
//  H-bridge is not suitable for DCC current sensing:
//  - Its short circuit protection triggers within microseconds,
//    preventing normal DCC operation with capacitive loads.
//  - The 100mΩ shunt generates only ~6mV for a 60mA ACK pulse,
//    which is too small for reliable ADC measurement.
//  Both short circuit detection and ACK detection require a
//  different H-bridge or an external current sensing circuit.
//  See dcc_current_monitor.cpp for the full implementation.
// ═══════════════════════════════════════════════════════════════
#include <Arduino.h>
#include "module_arch.h"

// ACK detection constants (per NMRA DCC specification)
#define DCC_ACK_MIN_MS        4    // Minimum valid ACK duration (ms)
#define DCC_ACK_MAX_MS        8    // Maximum valid ACK duration (ms)
#define DCC_ACK_DELTA         30   // Minimum ADC rise above baseline to detect ACK
#define DCC_SHORT_CHECK_MS    1    // Short circuit check interval (ms)
#define DCC_SHORT_DISABLED         // Short circuit detection disabled — hardware not ready
#define DCC_SHORT_DEBOUNCE_MS 500  // Wait after power-on before enabling short circuit check
#define DCC_SHORT_CONFIRM     5    // Consecutive samples above threshold to confirm short circuit
#define DCC_ADC_MAX          4095  // RP2350 ADC full scale (12-bit)

// ─────────────────────────────────────────────────────────────
//  AckState — state machine for decoder ACK detection
//
//  Transitions each service-mode packet cycle through IDLE/SUCCESS/
//  FAIL → WAIT (new baseline captured) → DETECTING (current rise
//  seen) → SUCCESS or FAIL (duration evaluated once current falls
//  back to baseline, or times out) — see _checkAck() in the .cpp for
//  the full transition logic.
// ─────────────────────────────────────────────────────────────
enum class AckState : uint8_t {
    IDLE       = 0,  // Not in service mode
    WAIT       = 1,  // Waiting for ACK current rise
    DETECTING  = 2,  // ACK detected, measuring duration
    SUCCESS    = 3,  // Valid ACK (correct duration) — notifyAck() called
    FAIL       = 4,  // Invalid ACK (too short or too long)
};

// ─────────────────────────────────────────────────────────────
//  DccCurrentMonitor — ADC-based current monitor
// ─────────────────────────────────────────────────────────────
class DccCurrentMonitor {
public:
    // begin() — records the ADC pin and short-circuit threshold to
    // use, and configures the RP2350 ADC peripheral for 12-bit
    // resolution (its native/maximum resolution, giving readings in
    // the range 0-4095 as referenced by DCC_ADC_MAX and the default
    // _shortThresh value).
    // adcPin: GPIO pin number (must be ADC-capable)
    // shortThresholdAdc: ADC value (0-4095) above which short circuit is flagged
    void begin(uint8_t adcPin, uint16_t shortThresholdAdc);

    // Main update function — call from core0 loop (DccHal::loop).
    // Reads ADC, checks for short circuit and ACK as appropriate.
    void loop();

    // Returns the last raw ADC reading (0-4095)
    uint16_t lastAdcValue()    const { return _lastAdc; }

    // Returns true if a short circuit is currently detected
    bool     inShortCircuit()  const { return _inShort; }

    // Returns the current state of the ACK detection state machine
    AckState ackState()        const { return _ackState; }

private:
    uint8_t  _pin          = 0;
    uint16_t _shortThresh  = 1020;  // ADC threshold for short circuit
    uint16_t _lastAdc      = 0;     // Most recent ADC reading
    uint16_t _baseAdc      = 0;     // Baseline current at start of service mode packet
    bool     _inShort      = false; // True if short circuit is currently active
    bool     _wasInService = false; // True if we were in service mode last loop
    AckState _ackState     = AckState::IDLE;
    uint32_t _ackStartMs   = 0;    // Timestamp when ACK current rise was detected
    uint32_t _lastShortMs  = 0;    // Timestamp of last short circuit check
    uint32_t _powerOnMs    = 0;    // Timestamp of last power-on (for debounce)
    uint8_t  _shortCount   = 0;    // Consecutive samples above threshold

    // Checks for short circuit condition and triggers shutdown if confirmed.
    // Disabled when DCC_SHORT_DISABLED is defined.
    void _checkShortCircuit(uint16_t adc);

    // Runs the ACK detection state machine during service mode.
    // Calls DccHal::notifyAck() on successful ACK detection.
    void _checkAck(uint16_t adc);
};

extern DccCurrentMonitor gDccCurrentMonitor;
