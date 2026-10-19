#pragma once
#include <Arduino.h>
#include "hardware/watchdog.h"
#include "hardware/structs/watchdog.h"
#if __has_include("hardware/structs/powman.h")
#include "hardware/structs/powman.h"            // RP2350: chip reset reason
#define CRASHDIAG_HAVE_POWMAN 1
#endif

// ═══════════════════════════════════════════════════════════════
//  crash_diag.h — why did the command station restart, and where
//  was it when it did?
//
//  Background (Rob, Oct 2026, 5-loco run): the station rebooted by
//  itself. Before that, no loco responded to Rocrail any more — the
//  trace log simply stopped, consistent with core1 (all networking)
//  hanging while core0 kept refreshing DCC with the last speeds, until
//  the watchdog reset the board ~12s later. Nothing in the log said
//  where core1 was stuck, or even whether it was the watchdog at all
//  rather than a supply dip during the collision that followed.
//
//  Two cheap diagnostics:
//
//  1. Breadcrumbs. Both cores write "where am I" (a stage code and,
//     inside the module loops, the module's registry index) into the
//     RP2350 watchdog scratch registers. Those registers survive a
//     watchdog reset (not a power-on or brown-out), so after a
//     watchdog reboot setup() can report where each core last was.
//     A register write is a single store — cheap enough for every
//     module call.
//
//  2. Reset reason. The RP2350 records what caused the last chip
//     reset (power-on, brown-out, RUN pin / reset button, glitch
//     detector, watchdog, ...). Logged once at boot, raw and decoded.
//
//  Scratch register use (0..3 only; the SDK/bootrom use 4..7):
//    scratch[0]  MAGIC | flags     (flags: see Flag)
//    scratch[1]  core0 crumb       stage<<8 | module index (0xFF = none)
//    scratch[2]  core1 crumb       idem
//    scratch[3]  uptime in ms, written by core0 every loop()
// ═══════════════════════════════════════════════════════════════
namespace CrashDiag {

constexpr uint32_t MAGIC      = 0x4C5A0000u;   // "LZ" in the upper half
constexpr uint32_t MAGIC_MASK = 0xFFFF0000u;

enum Stage : uint8_t {
    ST_NONE          = 0,
    ST_LOOP_TOP      = 1,   // start of loop()/loop1()
    ST_EVENTBUS      = 2,   // EventBus::poll()
    ST_BONJOUR       = 3,   // EthernetBonjour.run()
    ST_MODULE        = 4,   // inside a module's loop() (module index valid)
    ST_ETH_MAINTAIN  = 5,   // Ethernet.maintain() (DHCP)
    ST_CMDBUS        = 6,   // CommandBus::poll()
    ST_LEDS          = 7,   // gLeds.loop()
    ST_LOOP_END      = 8,   // iteration finished
};

enum Flag : uint32_t {
    F_RESET_REQUESTED = 0x0001,   // deliberate restart (web / XpressNet reset)
    F_CORE1_STALL     = 0x0002,   // core0 cut track power because core1 stalled
};

inline void crumb(uint8_t core, uint8_t stage, uint8_t module = 0xFF) {
    watchdog_hw->scratch[1 + (core & 1)] = ((uint32_t)stage << 8) | module;
}
inline void setFlag(uint32_t f)     { watchdog_hw->scratch[0] = MAGIC | ((watchdog_hw->scratch[0] & ~MAGIC_MASK) | f); }
inline void setUptime(uint32_t ms)  { watchdog_hw->scratch[3] = ms; }
inline uint32_t core1Crumb()        { return watchdog_hw->scratch[2]; }

inline const char* stageName(uint8_t s) {
    switch (s) {
        case ST_LOOP_TOP:     return "loop start";
        case ST_EVENTBUS:     return "EventBus.poll";
        case ST_BONJOUR:      return "mDNS (Bonjour.run)";
        case ST_MODULE:       return "module loop";
        case ST_ETH_MAINTAIN: return "Ethernet.maintain (DHCP)";
        case ST_CMDBUS:       return "CommandBus.poll";
        case ST_LEDS:         return "LEDs";
        case ST_LOOP_END:     return "loop end";
        default:              return "-";
    }
}

// Formats one crumb as "<stage>[ '<module name>']" into buf.
// nameOf(index) resolves a registry index to a module name (or nullptr).
inline void describeCrumb(uint32_t c, char* buf, size_t len,
                          const char* (*nameOf)(uint8_t)) {
    uint8_t stage  = (c >> 8) & 0xFF;
    uint8_t module = c & 0xFF;
    const char* mn = (stage == ST_MODULE && nameOf) ? nameOf(module) : nullptr;
    if (mn) snprintf(buf, len, "%s '%s'", stageName(stage), mn);
    else    snprintf(buf, len, "%s", stageName(stage));
}

// Call once, early in setup(), BEFORE anything writes a crumb and
// before watchdog_enable(). Logs the reset reason and — after a
// watchdog reset — where both cores were. Then clears the scratch
// registers for this run. `out` is any Print (traceSerial).
inline void reportAndReset(Print& out, const char* (*nameOf)(uint8_t)) {
    // ── Chip reset reason ───────────────────────────────────
#ifdef CRASHDIAG_HAVE_POWMAN
    uint32_t cr = powman_hw->chip_reset;
    char why[160]; why[0] = '\0';
    auto add = [&](const char* s) {
        size_t n = strlen(why);
        snprintf(why + n, sizeof(why) - n, "%s%s", n ? ", " : "", s);
    };
#ifdef POWMAN_CHIP_RESET_HAD_POR_BITS
    if (cr & POWMAN_CHIP_RESET_HAD_POR_BITS)               add("power-on");
#endif
#ifdef POWMAN_CHIP_RESET_HAD_BOR_BITS
    if (cr & POWMAN_CHIP_RESET_HAD_BOR_BITS)               add("BROWN-OUT (supply dip)");
#endif
#ifdef POWMAN_CHIP_RESET_HAD_RUN_LOW_BITS
    if (cr & POWMAN_CHIP_RESET_HAD_RUN_LOW_BITS)           add("RUN pin / reset button");
#endif
#ifdef POWMAN_CHIP_RESET_HAD_GLITCH_DETECT_BITS
    if (cr & POWMAN_CHIP_RESET_HAD_GLITCH_DETECT_BITS)     add("GLITCH detector (voltage/clock disturbance)");
#endif
#ifdef POWMAN_CHIP_RESET_HAD_WATCHDOG_RESET_RSM_BITS
    if (cr & POWMAN_CHIP_RESET_HAD_WATCHDOG_RESET_RSM_BITS)    add("watchdog (rsm)");
#endif
#ifdef POWMAN_CHIP_RESET_HAD_WATCHDOG_RESET_SWCORE_BITS
    if (cr & POWMAN_CHIP_RESET_HAD_WATCHDOG_RESET_SWCORE_BITS) add("watchdog (swcore)");
#endif
#ifdef POWMAN_CHIP_RESET_HAD_SWCORE_PD_BITS
    if (cr & POWMAN_CHIP_RESET_HAD_SWCORE_PD_BITS)         add("switched-core power-down");
#endif
#ifdef POWMAN_CHIP_RESET_HAD_DP_RESET_REQ_BITS
    if (cr & POWMAN_CHIP_RESET_HAD_DP_RESET_REQ_BITS)      add("debugger");
#endif
#ifdef POWMAN_CHIP_RESET_HAD_RESCUE_BITS
    if (cr & POWMAN_CHIP_RESET_HAD_RESCUE_BITS)            add("rescue");
#endif
    out.printf("RESET: chip_reset=0x%08lX (%s)\r\n", (unsigned long)cr, why[0] ? why : "?");
#endif

    bool wdTimeout = watchdog_enable_caused_reboot();
    bool wdAny     = watchdog_caused_reboot();
    uint32_t s0 = watchdog_hw->scratch[0];
    bool valid  = (s0 & MAGIC_MASK) == MAGIC;

    if (!wdAny) {
        out.printf("RESET: not a watchdog reset\r\n");
    } else if (valid && (s0 & F_RESET_REQUESTED)) {
        out.printf("RESET: requested restart (web interface / XpressNet reset)\r\n");
    } else if (wdTimeout && valid) {
        char c0[64], c1[64];
        describeCrumb(watchdog_hw->scratch[1], c0, sizeof(c0), nameOf);
        describeCrumb(watchdog_hw->scratch[2], c1, sizeof(c1), nameOf);
        out.printf("RESET: WATCHDOG TIMEOUT after %lu s uptime%s\r\n",
                   (unsigned long)(watchdog_hw->scratch[3] / 1000),
                   (s0 & F_CORE1_STALL) ? " (track power had been cut: core1 stalled)" : "");
        out.printf("RESET:   core0 was at: %s\r\n", c0);
        out.printf("RESET:   core1 was at: %s\r\n", c1);
    } else {
        out.printf("RESET: watchdog reset (%s), no breadcrumbs\r\n",
                   wdTimeout ? "timeout" : "software reboot");
    }

    watchdog_hw->scratch[0] = MAGIC;
    watchdog_hw->scratch[1] = 0;
    watchdog_hw->scratch[2] = 0;
    watchdog_hw->scratch[3] = 0;
}

} // namespace CrashDiag
