#pragma once
// ═══════════════════════════════════════════════════════════════
//  power_monitor.h  —  onboard power-rail voltage monitoring
//
//  v2 hardware has three ADC-capable pins, each reading the voltage
//  across the lower resistor of a two-resistor voltage divider that
//  scales down one of the board's own power rails (VIN, 12V, 5V) to
//  a range the RP2350's ADC (0-3.3V) can measure directly.
//
//  Divider networks (from the schematic):
//    VIN rail: 47kΩ (series) + 4.7kΩ (measured across) — ADC reads
//              ~1.6V when VIN is at its nominal/spec voltage
//    12V rail: 22kΩ (series) + 4.7kΩ (measured across) — ADC reads
//              ~2.1V at a nominal 12V rail
//    5V  rail: 10kΩ (series) + 4.7kΩ (measured across) — ADC reads
//              ~1.5V at a nominal 5V rail
//
//  PowerRailMonitor (below) reconstructs the actual rail voltage from
//  the measured (across-the-lower-resistor) voltage using the exact
//  resistor values, rather than a single calibration point — this is
//  more precise since it's derived directly from the physical divider
//  ratio: actual = measured × (R_series + R_measure) / R_measure.
//  Small deviations from the nominal reference values above are
//  expected and normal, reflecting real-world resistor tolerance
//  (typically ±5%) rather than indicating a fault.
//
//  Checked once at boot (begin(), logged to Serial2) and continuously
//  thereafter (loop(), once per instance from PowerMonitor::loop()),
//  with both the measured (across-resistor) and reconstructed (full
//  rail) voltages readable live via the web interface.
// ═══════════════════════════════════════════════════════════════
#include <Arduino.h>
#include "module_arch.h"
#include "hardware_config.h"

#define PWR_ADC_MAX      4095  // RP2350 ADC full scale (12-bit)
#define PWR_ADC_VREF_MV  3300  // RP2350 ADC reference voltage (mV)

// ─────────────────────────────────────────────────────────────
//  PowerRailMonitor — reads one ADC-capable pin (the voltage across
//  the lower resistor of a divider) and reconstructs the actual rail
//  voltage from the exact resistor values.
// ─────────────────────────────────────────────────────────────
class PowerRailMonitor {
public:
    // pin:          ADC-capable GPIO reading this rail's divider
    // rSeriesOhm:   the series (upper) resistor value, in ohms
    // rMeasureOhm:  the resistor the ADC actually measures across
    //               (the lower resistor, in ohms)
    // nominalMv:    this rail's nominal/expected voltage, in mV —
    //               purely informational, shown alongside the
    //               reconstructed reading on the web page for
    //               comparison; not used in the calculation itself
    // name:         short display name for the web interface ("VIN",
    //               "12V", "5V")
    PowerRailMonitor(uint8_t pin, uint32_t rSeriesOhm, uint32_t rMeasureOhm,
                      uint16_t nominalMv, const char* name)
        : _pin(pin), _rSeriesOhm(rSeriesOhm), _rMeasureOhm(rMeasureOhm)
        , _nominalMv(nominalMv), _name(name) {}

    // No pin-specific setup needed — analogReadResolution(12) is a
    // global peripheral setting; set once here (harmless if called
    // multiple times across the three instances). Also takes and
    // logs the first reading immediately, so the rail voltages are
    // checked and visible in the boot log right at power-up.
    void begin() {
        analogReadResolution(12);
        loop();
        Serial2.printf("PowerMonitor: %s gemeten=%umV rail=%umV (nominaal %umV)\n",
                       _name, _measuredMv, _railMv, _nominalMv);
    }

    // Call every loop() iteration. Reads the ADC, converts to
    // millivolts (the voltage actually across the lower resistor),
    // then reconstructs the full rail voltage from the exact divider
    // ratio.
    void loop() {
        uint16_t raw = (uint16_t)analogRead(_pin);
        _measuredMv = (uint16_t)(((uint32_t)raw * PWR_ADC_VREF_MV) / PWR_ADC_MAX);
        _railMv = (uint16_t)(((uint32_t)_measuredMv * (_rSeriesOhm + _rMeasureOhm))
                              / _rMeasureOhm);
    }

    // Voltage actually measured across the lower (measure) resistor,
    // in millivolts — i.e. the raw ADC reading converted to mV,
    // before divider reconstruction.
    uint16_t measuredMv() const { return _measuredMv; }

    // Reconstructed actual rail voltage, in millivolts.
    uint16_t railMv() const { return _railMv; }

    // This rail's nominal/expected voltage, in millivolts (informational).
    uint16_t nominalMv() const { return _nominalMv; }

    // Short display name ("VIN", "12V", "5V").
    const char* name() const { return _name; }

private:
    uint8_t     _pin;
    uint32_t    _rSeriesOhm, _rMeasureOhm;
    uint16_t    _nominalMv;
    const char* _name;
    uint16_t    _measuredMv = 0;
    uint16_t    _railMv     = 0;
};

// ─────────────────────────────────────────────────────────────
//  PowerMonitor — HardwareModule owning the three v2 power-rail
//  monitors (VIN, 12V, 5V). Registered via ModuleRegistry like any
//  other module; runs on core0 alongside DccHal.
// ─────────────────────────────────────────────────────────────
class PowerMonitor : public HardwareModule {
public:
    PowerMonitor()
        : HardwareModule("PowerMonitor", ModuleId::POWER_MONITOR, ModuleCore::CORE0) {}

    void begin() override {
        _vin.begin();
        _v12.begin();
        _v5.begin();
    }

    void loop() override {
        _vin.loop();
        _v12.loop();
        _v5.loop();
    }

    void usedPins(const uint8_t*& pins, uint8_t& count) const override {
        static uint8_t p[3] = { HW_PWR_VIN_PIN, HW_PWR_12V_PIN, HW_PWR_5V_PIN };
        pins = p; count = 3;
    }

    const PowerRailMonitor& vin() const { return _vin; }
    const PowerRailMonitor& v12() const { return _v12; }
    const PowerRailMonitor& v5()  const { return _v5;  }

private:
    // VIN rail: 47kΩ series + 4.7kΩ measure, nominal ADC ~1.6V
    PowerRailMonitor _vin{HW_PWR_VIN_PIN, 47000, 4700,
                          (uint16_t)(1600UL * (47000+4700) / 4700), "VIN"};
    // 12V rail: 22kΩ series + 4.7kΩ measure, nominal ADC ~2.1V
    PowerRailMonitor _v12{HW_PWR_12V_PIN, 22000, 4700,
                          (uint16_t)(2100UL * (22000+4700) / 4700), "12V"};
    // 5V rail: 10kΩ series + 4.7kΩ measure, nominal ADC ~1.5V
    PowerRailMonitor _v5 {HW_PWR_5V_PIN,  10000, 4700,
                          (uint16_t)(1500UL * (10000+4700) / 4700), "5V"};
};

extern PowerMonitor gPowerMonitor;
