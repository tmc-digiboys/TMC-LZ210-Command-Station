#pragma once
#include <Arduino.h>
#include "hardware_config.h"

// ─────────────────────────────────────────────────────────────
//  LED pin assignments — see hardware_config.h
//
//  LED_PIN_STATUS uses the board's built-in LED (LED_BUILTIN, the
//  standard Arduino macro — on this Olimex board it resolves to the
//  Pico SDK's PICO_DEFAULT_LED_PIN, GPIO25) rather than a dedicated
//  HW_LED_STATUS_PIN GPIO, which no longer exists on v2 hardware.
// ─────────────────────────────────────────────────────────────
#define LED_PIN_STATUS   LED_BUILTIN
#define LED_PIN_W5500    HW_LED_W5500_PIN
#define LED_PIN_DCCBUS   HW_LED_DCCBUS_PIN
#define LED_PIN_RSBUS    HW_LED_RSBUS_PIN
#define LED_PIN_LNBUS    HW_LED_LNBUS_PIN
#define LED_PIN_XPNBUS   HW_LED_XPNBUS_PIN

// ─────────────────────────────────────────────────────────────
//  LedMode — operating mode for a single LED
// ─────────────────────────────────────────────────────────────
enum class LedMode : uint8_t {
    LED_OFF,    // Always off
    LED_ON,     // Always on
    LED_BLINK,  // Blink at fixed interval (set via setBlinkMs)
    LED_PULSE,  // Single short pulse, then returns to LED_OFF
};

// ─────────────────────────────────────────────────────────────
//  Led — single LED controller
//
//  Supports continuous on/off, blinking at a configurable rate,
//  and single-shot pulse mode for activity indicators.
// ─────────────────────────────────────────────────────────────
class Led {
public:
    // Led() — constructs the controller for one LED. `pin` is the
    // GPIO number driving it; `blinkMs` is the default blink half-
    // period (for LED_BLINK) and pulse duration (for pulse()),
    // adjustable later via setBlinkMs().
    Led(uint8_t pin, uint32_t blinkMs = 100)
        : _pin(pin), _blinkMs(blinkMs) {}

    // begin() — configures the GPIO pin as an output and drives it
    // low, so the LED starts in a known, off state. Must be called
    // once during setup before any other method is used.
    void begin() {
        pinMode(_pin, OUTPUT);
        digitalWrite(_pin, LOW);
    }

    // setBlinkMs() — changes the blink half-period / pulse duration
    // (in milliseconds) used by LED_BLINK mode and by pulse().
    void setBlinkMs(uint32_t ms) { _blinkMs = ms; }

    // setMode() — switches this LED to a new mode. LED_ON immediately
    // drives the pin high; LED_OFF immediately drives it low.
    // LED_BLINK does not change the pin state right away — it starts
    // toggling the pin at the configured _blinkMs interval the next
    // time loop() runs. In every case, resets the internal timing
    // reference (_lastMs) to the current time, so a freshly selected
    // blink interval starts counting from now rather than from
    // whatever timing was in effect before.
    void setMode(LedMode mode) {
        _mode = mode;
        if (mode == LedMode::LED_ON)  digitalWrite(_pin, HIGH);
        if (mode == LedMode::LED_OFF) digitalWrite(_pin, LOW);
        _lastMs = millis();
    }

    // pulse() — triggers a single short pulse (used as an activity
    // indicator, e.g. "a DCC packet was just sent" or "a byte of
    // Ethernet traffic just occurred"). Does nothing if the LED is
    // currently in LED_ON mode (used for a steady power indicator,
    // where a transient pulse would be meaningless/confusing).
    // Otherwise immediately drives the pin high and records when the
    // pulse should end (_blinkMs milliseconds from now); loop() is
    // responsible for turning the pin back off once that time has
    // elapsed. A new pulse() call while one is already in progress
    // simply restarts the timer, extending the visible pulse.
    void pulse() {
        if (_mode == LedMode::LED_ON) return;
        _pulsing  = true;
        _pulseEnd = millis() + _blinkMs;
        digitalWrite(_pin, HIGH);
    }

    // loop() — must be called repeatedly from the main loop to
    // advance this LED's time-based behaviour. In LED_BLINK mode,
    // toggles the pin and flips the internal _state flag every time
    // _blinkMs has elapsed since the last toggle. Independently of
    // the current mode, if a pulse() is in progress and its end time
    // has been reached, ends the pulse: the pin is restored to HIGH
    // if the underlying mode is LED_ON (a pulse briefly happening
    // during a technically-impossible LED_ON state is defensive here,
    // since pulse() itself already refuses to start one in that case)
    // or LOW otherwise (the normal case: pulse ends, LED returns to
    // off).
    void loop() {
        uint32_t now = millis();
        switch (_mode) {
            case LedMode::LED_BLINK:
                if (now - _lastMs >= _blinkMs) {
                    _state  = !_state;
                    _lastMs = now;
                    digitalWrite(_pin, _state ? HIGH : LOW);
                }
                break;
            default:
                break;
        }
        // End pulse if duration has elapsed
        if (_pulsing && now >= _pulseEnd) {
            _pulsing = false;
            digitalWrite(_pin, _mode == LedMode::LED_ON ? HIGH : LOW);
        }
    }

private:
    uint8_t  _pin;
    uint32_t _blinkMs;
    LedMode  _mode     = LedMode::LED_OFF;
    bool     _state    = false;
    bool     _pulsing  = false;
    uint32_t _lastMs   = 0;
    uint32_t _pulseEnd = 0;
};

// ─────────────────────────────────────────────────────────────
//  LedStatus — controller for all six status LEDs
//
//  status:  track power (solid on = powered, fast blink = short
//           circuit) — uses the board's built-in LED
//  w5500:   Ethernet traffic pulse
//  dccbus:  DCC bus activity pulse
//  rsbus:   RS-Bus activity pulse
//  lnbus:   LocoNet bus activity pulse
//  xpnbus:  XpressNet bus activity pulse (any transport: LAN, USB,
//           RS485, or Z21's tunnelled XpressNet commands)
// ─────────────────────────────────────────────────────────────
class LedStatus {
public:
    Led status {LED_PIN_STATUS, 500};   // Power LED (slow blink default)
    Led w5500  {LED_PIN_W5500,  80};    // Ethernet traffic pulse
    Led dccbus {LED_PIN_DCCBUS, 80};    // DCC traffic pulse
    Led rsbus  {LED_PIN_RSBUS,  80};    // RS-Bus traffic pulse
    Led lnbus  {LED_PIN_LNBUS,  80};    // LocoNet traffic pulse
    Led xpnbus {LED_PIN_XPNBUS, 80};    // XpressNet traffic pulse

    // begin() — initialises all six LED pins as outputs, driving
    // them low. Call once during setup, after the individual Led
    // members have already been constructed with their pin numbers
    // (via the member-initialiser default arguments above).
    void begin() {
        status.begin();
        w5500.begin();
        dccbus.begin();
        rsbus.begin();
        lnbus.begin();
        xpnbus.begin();
    }

    // loop() — advances the time-based blink/pulse state of every
    // LED by delegating to each one's own loop(). Must be called on
    // every iteration of the main loop for blinking and pulsing to
    // work correctly.
    void loop() {
        status.loop();
        w5500.loop();
        dccbus.loop();
        rsbus.loop();
        lnbus.loop();
        xpnbus.loop();
    }

    // ── Event-driven LED updates ──────────────────────────

    // onPowerOn() — call when track power is switched on. Resets the
    // status LED's blink interval to the normal 500ms value (undoing
    // any short-circuit fast-blink setting) and switches it to solid
    // LED_ON.
    void onPowerOn()         { status.setBlinkMs(500); status.setMode(LedMode::LED_ON);    }

    // onPowerOff() — call when track power is switched off. Turns the
    // status LED off entirely.
    void onPowerOff()        { status.setMode(LedMode::LED_OFF); }

    // onShortCircuit() — call when a short circuit has been detected.
    // Switches the status LED to a fast 100ms blink, visually
    // distinguishing this condition from normal "power on" (solid) or
    // "power off" (dark). Remains in this state until onPowerOn() (or
    // onPowerOnClear()) is called.
    void onShortCircuit()    { status.setBlinkMs(100); status.setMode(LedMode::LED_BLINK); }

    // onPowerOnClear() — call to restore the normal "power on" LED
    // state after a short circuit condition has cleared. Functionally
    // identical to onPowerOn(); kept as a separate, more descriptively
    // named entry point for that specific call site.
    void onPowerOnClear()    { status.setBlinkMs(500); status.setMode(LedMode::LED_ON);    }

    // onEthernetTraffic() — call whenever Ethernet activity occurs
    // (e.g. a TCP/UDP packet was sent or received), triggering a
    // brief pulse on the w5500 LED.
    void onEthernetTraffic() { w5500.pulse();  }

    // onDccTraffic() — call whenever a DCC packet is sent onto the
    // track, triggering a brief pulse on the dccbus LED.
    void onDccTraffic()      { dccbus.pulse(); }

    // onRsBusTraffic() — call whenever RS-Bus feedback activity
    // occurs, triggering a brief pulse on the rsbus LED.
    void onRsBusTraffic()    { rsbus.pulse();  }

    // onLnBusTraffic() — call whenever activity occurs on the
    // LocoNet bus (see LocoNetModule::loop()), triggering a brief
    // pulse on the lnbus LED.
    void onLnBusTraffic()    { lnbus.pulse();  }

    // onXpnBusTraffic() — call whenever a frame is processed by the
    // shared XpressNetHandler (see XpressNetHandler::process()),
    // regardless of which transport it arrived through (LenzLAN,
    // LenzUSB, XpressNet RS485, or Z21's tunnelled XpressNet
    // commands), triggering a brief pulse on the xpnbus LED.
    void onXpnBusTraffic()   { xpnbus.pulse(); }
};

extern LedStatus gLeds;
