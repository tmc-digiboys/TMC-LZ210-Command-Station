#pragma once
#include <Arduino.h>

// ═══════════════════════════════════════════════════════════════
//  power_estimator.h — time-averaged dissipation of a DRV887x
//
//  Replaces PowerIntegrator. Same idea, as specified by the hardware
//  designer (nFault.md, "Aanbeveling"): estimate the power dissipated
//  in the chip as I^2 x RDS(on), averaged over time to model its
//  thermal inertia, and cut DCC before it overheats. What changed:
//
//   - The result is in milliwatts, not squared millivolts, so the
//     limit means the same thing on both chips. A 3A average is 1.8W
//     in a DRV8874 (200 mOhm) but 6.3W in a DRV8876 (700 mOhm); a
//     fixed mV threshold cannot express that.
//   - The input is a PEAK over the last millisecond, not one sample.
//     The designer is explicit that the ADC signal is spiky and the
//     software has to pick the highest value of a burst.
//   - HBridgeGuard only feeds it while the bridge is NORMAL; in the
//     fault states the SENSE value is the aftermath of an output the
//     chip has already switched off and says nothing about load.
//
//  Leaky integrator in plain unsigned integer math, with 8 extra
//  fractional bits so the shift does not truncate small values:
//      acc = acc - (acc >> shift) + (p >> shift)
//  where acc and p are in 1/256 mW. Steady state converges to p.
//  The time constant is about 2^shift milliseconds at 1ms steps.
// ═══════════════════════════════════════════════════════════════
class PowerEstimator {
public:
    // Feed `steps` milliseconds of the same peak reading.
    //   peakMv      highest SENSE (mV) seen in that time
    //   scaleMvA    SENSE mV per ampere (AIPROPI x R_IPROPI)
    //   rdsMohm     RDS(on), HS+LS, in milliohm
    //   shift       time constant, 2^shift ms (clamped 1..12)
    void integrate(uint16_t peakMv, uint32_t scaleMvA, uint32_t rdsMohm,
                   uint8_t shift, uint8_t steps) {
        if (scaleMvA == 0 || steps == 0) return;
        if (shift < 1)  shift = 1;
        if (shift > 12) shift = 12;

        // mA = mV * 1000 / (mV per A);  mW = mA^2 * mOhm / 1e6
        uint64_t ma = (uint64_t)peakMv * 1000ULL / scaleMvA;
        uint64_t mw = ma * ma * (uint64_t)rdsMohm / 1000000ULL;
        if (mw > 1000000ULL) mw = 1000000ULL;
        uint32_t p = (uint32_t)mw << 8;

        for (uint8_t i = 0; i < steps; i++) {
            _acc = _acc - (_acc >> shift) + (p >> shift);
        }
    }

    // Current estimate in mW.
    uint32_t equivalentMw() const { return _acc >> 8; }

    void reset() { _acc = 0; }

private:
    uint32_t _acc = 0;  // 1/256 mW
};
