#pragma once
#include <Arduino.h>

// ═══════════════════════════════════════════════════════════════
//  nfault_source.h — interrupt-timed nFAULT edge capture, one per
//  H-bridge
//
//  Replaces the capture half of the old HBridgeFault class. It
//  records ONE thing, exactly: the micros() timestamp of every
//  high-to-low edge on the nFAULT pin, from an interrupt on CHANGE.
//  All decisions about what an edge means live in HBridgeGuard
//  (hbridge_guard.h) — this class has no policy at all.
//
//  Why timestamps instead of polling: per the hardware designer
//  (Aiko Pras, nFault.md), an ITRIP fault repeats every 58 or 100us
//  (one DCC half-bit) and an OCP fault every ~2ms. Telling those
//  apart, and honouring windows like "ignore everything for 1.5ms,
//  then watch for 20ms", needs microsecond-accurate event times that
//  do not depend on how fast loop() happens to run.
//
//  Only falling edges are queued: the designer's state machine reacts
//  to "high-to-low edges" only, and the nFAULT level may still be low
//  from the previous event when monitoring resumes.
//
//  Statistics kept here (web interface only, no decisions hang on
//  them): rawCount() = every falling edge, and a smoothed duty
//  (fraction of time nFAULT is low) from the rising edges. The duty
//  figure is what showed 0.3% for a healthy loco and ~48% for the
//  one that would not run, so it stays as a diagnostic.
//
//  Threading: DccHal is a core0 module and attachInterrupt() fires on
//  the core that registered it, so the ISR and the consumer run on the
//  same core. The ring-buffer indices are single bytes and volatile.
// ═══════════════════════════════════════════════════════════════
class NFaultSource {
public:
    NFaultSource(uint8_t pin, const char* name) : _pin(pin), _name(name) {}

    const char* name() const { return _name; }

    // Input with an internal pull-up as a backstop (the board has its
    // own external pull-up; this protects against a missing one).
    void begin() { pinMode(_pin, INPUT_PULLUP); _isLow = digitalRead(_pin) == LOW; _fallUs = micros(); }

    // Plain function pointer: attachInterrupt() cannot take a bound
    // member function, so DccHal provides one static trampoline per
    // bridge that calls isrEdge().
    void attachIsr(void (*isr)()) {
        attachInterrupt(digitalPinToInterrupt(_pin), isr, CHANGE);
    }

    // Live pin level (not an event) — for the web status column.
    bool isLow() const { return digitalRead(_pin) == LOW; }

    // ── ISR side ──────────────────────────────────────────────
    // Minimal and ISR-safe: micros(), digitalRead(), a ring push.
    void isrEdge() {
        uint32_t now = micros();
        bool low = digitalRead(_pin) == LOW;
        if (low) {
            // Already low = we missed the rising edge in between. Typical
            // for OCP: the DRV8874 releases nFAULT after ~1.92ms for its
            // auto-retry and, if the fault persists, pulls it low again
            // within a few us — shorter than this interrupt's latency, so
            // by the time the "rise" interrupt runs the pin is low again
            // (Rob, Oct 2026: OCP episodes showed "nFAULT still low").
            // Close the previous pulse here; its length then includes that
            // brief high gap (a few us too long, nothing more).
            if (_isLow) pushPulse(_fallUs, now - _fallUs);
            _isLow  = true;
            _fallUs = now;
            _rawCount++;
            uint8_t next = (uint8_t)((_head + 1) % RING);
            if (next == _tail) { _dropped++; return; }  // consumer not keeping up
            _ring[_head] = now;
            _head = next;
            return;
        }
        // High while we already thought it was high: a whole low pulse
        // came and went within the interrupt latency. Its start time is
        // unknown, so record nothing (previously this was misread as a
        // rising edge with a bogus length since some older fall). The
        // edge pattern the guard sees is unchanged by this.
        if (!_isLow) return;
        _isLow = false;
        // Rising edge: the low period just ended. Duty statistic, and
        // the pulse itself (fall time + how long nFAULT was low) for
        // HBridgeGuard's per-episode pulse log.
        pushPulse(_fallUs, now - _fallUs);
    }

    void pushPulse(uint32_t fallUs, uint32_t lowUs) {
        uint8_t pn = (uint8_t)((_pHead + 1) % PRING);
        if (pn != _pTail) {
            _pFall[_pHead] = fallUs;
            _pLow[_pHead]  = lowUs;
            _pHead = pn;
        }
        uint32_t d = lowUs;
        if (d > DUTY_CAP_US) d = DUTY_CAP_US;
        _lowUsWindow += d;
    }

    // ── completed low pulses (loop context) ───────────────────
    // Oldest completed pulse without removing it: fall time and how
    // long nFAULT stayed low (us). false when none is queued.
    bool peekPulse(uint32_t& fallUs, uint32_t& lowUs) const {
        if (_pTail == _pHead) return false;
        fallUs = _pFall[_pTail];
        lowUs  = _pLow[_pTail];
        return true;
    }
    void dropPulse() { if (_pTail != _pHead) _pTail = (uint8_t)((_pTail + 1) % PRING); }

    // ── consumer side (loop context) ──────────────────────────
    // Oldest queued falling-edge timestamp, or false when empty.
    bool popFall(uint32_t& tUs) {
        if (_tail == _head) return false;
        tUs   = _ring[_tail];
        _tail = (uint8_t)((_tail + 1) % RING);
        return true;
    }

    // Discard everything queued — used while track power is off and
    // at power-on, so stale edges never reach the state machine.
    void clear() { _tail = _head; _pTail = _pHead; }

    // ── statistics ────────────────────────────────────────────
    uint32_t rawCount() const { return _rawCount; }
    uint32_t dropped()  const { return _dropped; }
    void     resetCounts() { _rawCount = 0; _dropped = 0; }

    // Smoothed duty in permille (0-1000), window 100ms, ~4 windows of
    // smoothing. Call regularly from loop().
    void updateDuty(uint32_t nowUs) {
        if ((uint32_t)(nowUs - _windowStartUs) < DUTY_WINDOW_US) return;
        uint32_t lowUs = _lowUsWindow;
        _lowUsWindow   = 0;
        _windowStartUs = nowUs;
        uint32_t raw = (lowUs * 1000UL) / DUTY_WINDOW_US;
        if (raw > 1000) raw = 1000;
        _dutyAcc = _dutyAcc - (_dutyAcc >> 2) + (raw >> 2);
    }
    uint16_t dutyPermille() const { return (uint16_t)_dutyAcc; }
    void     resetDuty()          { _dutyAcc = 0; _lowUsWindow = 0; }

private:
    // A single low period credited to the duty statistic is capped:
    // a noise misread of tens of thousands of us would otherwise
    // dominate a whole 100ms window (seen on the bench). Generously
    // above the ~2ms OCP retry, so no real event is affected.
    static constexpr uint32_t DUTY_CAP_US    = 10000;
    static constexpr uint32_t DUTY_WINDOW_US = 100000;
    static constexpr uint8_t  RING           = 64;
    static constexpr uint8_t  PRING          = 32;   // completed pulses

    uint8_t     _pin;
    const char* _name;

    volatile uint32_t _ring[RING] = {0};
    volatile uint8_t  _head       = 0;
    volatile uint8_t  _tail       = 0;
    volatile uint32_t _fallUs     = 0;
    volatile bool     _isLow      = false;   // level as last seen by isrEdge()
    volatile uint32_t _rawCount   = 0;
    volatile uint32_t _pFall[PRING] = {0};
    volatile uint32_t _pLow[PRING]  = {0};
    volatile uint8_t  _pHead      = 0;
    volatile uint8_t  _pTail      = 0;
    volatile uint32_t _dropped    = 0;

    volatile uint32_t _lowUsWindow   = 0;
    uint32_t          _windowStartUs = 0;
    uint32_t          _dutyAcc       = 0;
};
