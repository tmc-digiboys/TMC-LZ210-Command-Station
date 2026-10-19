#pragma once
#include <Arduino.h>

// ═══════════════════════════════════════════════════════════════
//  power_integrator.h — sustained-overload detection via a leaky
//  integrator of SENSE² (approximating DRV887x power dissipation)
//
//  Algorithm as specified by the hardware designer (Aiko Pras), for
//  the DCC/SM bridges specifically after their RIPROPI was changed
//  (3.3kOhm parallel to the stock 2.49kOhm) to raise ITRIP from
//  ~2.9A to ~5A: that higher trip level lets the DRV8874 ride out a
//  short inrush/chop burst without OCP intervening — but at 5A the
//  chip's own dissipation exceeds 5W, reaching thermal shutdown
//  within roughly 100ms if sustained. So continuous current must
//  stay under ~2.5-3A even though brief peaks up to ~5A are fine —
//  a distinction nFAULT/OCP alone cannot make, since it only reports
//  "over ITRIP right now", not "how much power has this chip been
//  dissipating over time".
//
//  Confirmed necessary in practice, not just in theory (Rob, scope
//  measurement on the bad loco after the resistor change): nFAULT
//  pulses as a steady ~54-60us square wave — continuous, repeated
//  chopping, not a single edge and not a stuck-low latch. Each
//  individual pulse is correctly filtered out as ordinary chopping
//  (see HBridgeFault's own comment) — but the chip was chopping
//  continuously for seconds, which is exactly the sustained-high-
//  average-current situation this class exists to catch. The
//  existing checkDuration() backstop happened to still escalate
//  after several seconds in that specific trace, but only by
//  accident of polling timing (never seeing a long enough gap to
//  call the pin "recovered") — not from any principled
//  power/current criterion, and not something to rely on.
//
//  Deliberately power-based (squared), not current-based: the
//  designer's own reasoning is that DRV887x heating is driven by
//  power dissipation, not current directly, so squaring the SENSE
//  voltage (proportional to current) before integrating gives a
//  quantity that tracks dissipated power — a few genuinely brief
//  high-current peaks (inrush, a chop burst) contribute little to
//  the time-averaged value and decay back out, while a sustained
//  elevated current keeps pushing the average up.
//
//  sample() is expected to be called roughly every 1ms, driven from
//  DccHal::loop()'s own millis()-based timing (see its own comment) —
//  not a hardware timer. That means the actual interval is "as close
//  to 1ms as loop() happens to manage", not a hard real-time
//  guarantee; expect some jitter, bounded by how long the slowest
//  thing in loop() ever takes.
// ═══════════════════════════════════════════════════════════════
class PowerIntegrator {
public:
    // Call roughly every 1ms with the latest SENSE reading (mV, e.g.
    // DccCurrentMonitor::lastMv()). shift controls the leaky
    // integrator's time constant: at ~1ms sampling, the time constant
    // is roughly 2^shift milliseconds (e.g. shift=7 -> ~128ms, in the
    // "a few hundred ms" range the designer described). Larger shift
    // = slower to respond, more tolerant of longer bursts; smaller =
    // faster to trip, less tolerant. Classic leaky-bucket / exponential
    // moving average, done in plain unsigned integer math (no float,
    // no division): acc = acc - (acc >> shift) + (sample^2 >> shift).
    // In steady state (a sustained, unchanging SENSE reading held long
    // enough), acc converges to sample^2 exactly — which is what
    // tripped() below compares against.
    void sample(uint16_t senseMv, uint8_t shift) {
        uint32_t sq = (uint32_t)senseMv * (uint32_t)senseMv;
        _acc = _acc - (_acc >> shift) + (sq >> shift);
    }

    // True once the integrated value has risen to or above the given
    // target continuous-current-equivalent SENSE voltage (mV),
    // squared for the comparison (see sample()'s own comment on why
    // this is power/squared rather than a plain voltage/current
    // comparison).
    bool tripped(uint16_t targetMv) const {
        uint32_t targetSq = (uint32_t)targetMv * (uint32_t)targetMv;
        return _acc >= targetSq;
    }

    // Current integrated value, as an equivalent SENSE voltage (mV) —
    // i.e. sqrt(acc) — for the web interface / diagnostics, so this
    // can be watched directly in the same units as the trip threshold
    // rather than as a squared, less legible raw accumulator value.
    uint16_t equivalentMv() const {
        uint32_t v = _acc;
        if (v == 0) return 0;
        // Plain integer sqrt (Newton's method) — no sqrt()/float
        // needed for a uint32_t this size; a handful of iterations is
        // more than enough to converge for values in this range.
        uint32_t x = v, y = (x + 1) / 2;
        while (y < x) { x = y; y = (x + v / x) / 2; }
        return (uint16_t)x;
    }

    // Resets to zero — called on a genuine power-on (DccHal::setPower()),
    // so a previous session's integrated value never carries across a
    // power cycle, and after escalating (so the next power-on starts
    // clean rather than already primed near the trip point).
    void reset() { _acc = 0; }

private:
    uint32_t _acc = 0;  // leaky-integrated SENSE-mV-squared
};
