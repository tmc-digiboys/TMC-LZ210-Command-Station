#pragma once
#include <Arduino.h>
#include "nfault_source.h"
#include "power_estimator.h"
#include "trace_log.h"

// ═══════════════════════════════════════════════════════════════
//  hbridge_guard.h — the nFAULT state machine, one per H-bridge
//
//  Implements the state diagram from the hardware designer
//  (Aiko Pras, nFault.md / drv8874_09_state_machine.png). It replaces
//  what had grown into seven independent paths that could each power
//  the station off, with ~37 settings that compensated for each
//  other's side effects. There is exactly one state at a time, and
//  exactly one way out to a power-off: SHORT.
//
//      NORMAL ──nFAULT──▶ IGNORE ──1.5ms──▶ WATCH ──new nFAULT──▶ INRUSH
//         ▲  ▲                                │                      │
//         │  └──────── no nFAULT in 20ms ─────┘                   35ms
//         │                                                          ▼
//         └────────── no nFAULT for 5ms ──────────────────────── VERIFY
//                                                                    │
//      SHORT ◀────────────────────── new nFAULT ─────────────────────┘
//      SHORT ◀── power estimate over limit (from NORMAL)
//
//   NORMAL   DCC on; SENSE is peak-held and fed to the power estimate.
//   IGNORE   1.5ms. Edges are only counted: the output may still be
//            switched off by the chip, and a micro short may be gone.
//   WATCH    20ms. No new nFAULT = micro short, back to NORMAL, no
//            action. A new nFAULT = a real fault, go to INRUSH.
//   INRUSH   enterInrushMode(): short pulses too brief to trip OCP let
//            a loco's capacitor charge. It must be on the track at
//            least 20ms to do anything, and per the library it only
//            appears once the packet in flight is finished (up to
//            ~13ms), hence 35ms in total. Edges are ignored.
//   VERIFY   leaveInrushMode(), normal DCC again, 5ms. Any nFAULT =
//            the cause did not go away = SHORT. Optionally (Params::
//            verifyEdges = 2, setting nf.verify_edges) a single stray
//            edge is tolerated: on a healthy loco, a lone micro short
//            landing inside those 5ms cut track power for a second while
//            the inrush curve and edge count looked like every other
//            harmless episode (TraceLog, Rob, 2 of 19 inrush runs). A
//            real fault repeats within ~0.1ms (ITRIP) or ~2ms (OCP), so
//            requiring a second edge delays a true SHORT by that much,
//            not more. Default 1 = the designer's rule, unchanged.
//   SHORT    the host cuts DCC; restarting is the host's decision.
//
//  OCP, ITRIP and micro short are told apart only for statistics, from
//  the pattern of edges (nFault.md "Onderscheid tussen OCP- en
//  ITRIP-fouten"): several edges inside the ignore window = ITRIP (it
//  repeats every 58/100us); the next edge ~1.5-3ms later = OCP (2ms
//  auto-retry); none = micro short. No decision depends on it.
//
//  INRUSH IN THIS VERSION IS TIME-BASED. Per the designer a falling
//  SENSE envelope means a charging capacitor and a flat one means a
//  real short, and that should shorten or end INRUSH. This version
//  only MEASURES that: SENSE is peak-held per window (a lead period
//  covering the packet in flight, then windows of `measUs`) and the
//  curve is logged when INRUSH ends. Nothing is decided from it yet —
//  the thresholds need to come from real curves, not a guess.
//
//  The machine decides on edge TIMESTAMPS from NFaultSource, never on
//  how often loop() happens to call it.
//
//  PULSE LENGTHS (statistics only). NFaultSource also reports every
//  completed low pulse (how long nFAULT stayed low). The pulses of an
//  episode (first 8) are appended to its classification line in the
//  trace log, e.g. "... micro short, no repeat within 20ms -> normal,
//  low us: 1928", and min/avg/max per class are shown on the web page.
//  Note what this measures: how long the CHIP held nFAULT low, not how
//  long the short lasted. After an OCP the DRV8874 keeps nFAULT low
//  until its ~2ms auto-retry however brief the cause was; an ITRIP
//  chop is tens of us. So the length mainly tells WHICH mechanism in
//  the chip acted — a check on the edge-pattern classification.
// ═══════════════════════════════════════════════════════════════
class HBridgeGuard {
public:
    // CamelCase on purpose: the DCC library headers #define SHORT, ON, OFF,
    // ESTOP and SERVICE, and a macro silently destroys an all-caps enumerator
    // (the compiler then reports a missing closing bracket before a numeric
    // constant), depending on which header happens to be included first.
    enum class State : uint8_t { Normal, Ignore, Watch, Inrush, Verify, Shorted };

    // Timing and power parameters. Cheap to build; DccHal refreshes it
    // from EEPROM a few times per second.
    struct Params {
        uint32_t ignoreUs;      // 1500
        uint32_t watchUs;       // 20000
        uint32_t inrushUs;      // 35000, enterInrushMode() to leave
        uint32_t verifyUs;      // 5000
        uint32_t measUs;        // inrush curve window, e.g. 2000
        bool     inrushAllowed; // false for CDE, and for the bridge not driving the track
        // Power estimate (DCC/SM). limitMw == 0 disables it.
        uint32_t scaleMvA;      // SENSE mV per ampere
        uint32_t rdsMohm;       // RDS(on) HS+LS
        uint32_t limitMw;       // trip level
        uint8_t  shift;         // time constant, 2^shift ms
        // How many nFAULT edges inside VERIFY make it a SHORT. 1 is the
        // designer's rule (any edge); 0 is treated as 1. With 2, a single
        // stray edge is tolerated and VERIFY is extended once by verifyUs,
        // within which a second edge must follow — see onEdge().
        // Trailing field on purpose: older positional initialisers keep working.
        uint8_t  verifyEdges;
        // Optional, off by default (0 = the designer's rule: ANY second
        // nFAULT in WATCH starts an inrush). With skipOther = 1, a second
        // edge that is neither ITRIP-like (several edges inside IGNORE)
        // nor OCP-like (~2ms) — i.e. an isolated PAIR of single edges
        // 3..20ms apart — does NOT start an inrush while the layout is
        // running. Why (TraceLog from Rob's layout, 79 inrush runs while
        // running): 36 of them were exactly such pairs, every one ended in
        // "verify ok" with a median of 0 edges during the whole inrush —
        // a wheel bouncing over a frog, not a sustained load — yet each
        // replaced the DCC signal by the 12.5% pattern for ~45ms.
        // Still an inrush as designed: ITRIP/OCP-like repeats, anything
        // within ponGraceUs after power-on (the layout really is charging
        // then: median 153 chopped edges), and a CLUSTER of pairs
        // (burstN within burstUs) which is not isolated any more.
        uint8_t  skipOther;
        uint8_t  burstN;        // 0 = 4
        uint32_t ponGraceUs;    // 0 = 3 s
        uint32_t burstUs;       // 0 = 250 ms
        // Optional, off by default (0 = designer's rule). With skipItrip = 1 a
        // repeat that classifies as ITRIP (several edges in IGNORE = the chip
        // chopping cycle by cycle, below OCP) does NOT start an inrush while
        // running: the DCC signal keeps flowing. Rob's layout: running-time
        // ITRIP bursts are short, show no big SENSE values and 20% of the
        // inrush runs they caused ended in a 1 s SHORT. OCP-like repeats and
        // anything within ponGraceUs after power-on still start an inrush.
        // Safety net: when skipped ITRIP episodes chain (each within watchUs of
        // the previous) for longer than itripMaxUs (0 = 50 ms), the chain is
        // treated as a sustained load and the inrush starts after all.
        uint8_t  skipItrip;
        uint32_t itripMaxUs;
    };

    // What the guard needs from its host. Plain function pointers so
    // this header does not depend on DccHal.
    struct Hooks {
        void (*enterInrush)(void* ctx)                    = nullptr;
        void (*leaveInrush)(void* ctx)                    = nullptr;
        void (*shortCircuit)(void* ctx, const char* why)  = nullptr;
        void* ctx                                         = nullptr;
    };

    void begin(const char* name, const Hooks& hooks) { _name = name; _h = hooks; }

    // Fresh start: power-on. Statistics are kept.
    void reset(uint32_t nowUs) {
        _now = nowUs;
        _resetUs = nowUs;            // power-on moment, for the "layout is charging" grace
        _pairCount = 0; _pairHead = 0;
        _chainN = 0;
        _state = State::Normal;
        _ignoredEdges = 0;
        _curveN = 0;
        _pwr.reset();
        _pwrLastUs = nowUs;
        _peakMv = 0;
        _samples = 0;
    }

    void resetStats() {
        _episodes = _nMicro = _nItrip = _nOcp = _nOther = 0;
        _inrushCount = _verifyOk = _shortCount = _verifyStray = _nSkipped = _nItripSkipped = _nItripEscalated = 0;
        _pwrMaxMw = 0;
        _reason[0] = '\0';
        for (auto& l : _low) l = LowStats{};
        _lastPulses[0] = '\0';
        _microMinGapMs = 0; _microFast = 0; _microBurstMax = 0;
        _microN = 0; _microHead = 0;
    }

    // Call every loop() iteration with the freshest SENSE reading
    // (mV). Cheap: just keeps the maximum. The ADC is only
    // MEANINGFUL in NORMAL and INRUSH; in the other states the value
    // is ignored (the designer: "ADC not read").
    void feedSense(uint16_t mv) {
        if (mv > _peakMv) _peakMv = mv;
        if (_samples < 0xFFFF) _samples++;
    }

    // Call every loop() iteration while track power is on.
    void update(NFaultSource& src, const Params& p, uint32_t nowUs) {
        _now    = nowUs;
        _measUs = p.measUs;
        uint32_t t;
        while (src.popFall(t)) {
            takePulses(src, t);     // pulses that ended before this edge
            advance(t, p);
            onEdge(t, p);
        }
        takePulses(src, nowUs);
        advance(nowUs, p);
        stage(p);
    }

    // ── read-only view for the web interface ──────────────────
    State       state() const { return _state; }
    const char* stateName() const { return nameOf(_state); }
    bool        inrushActive() const { return _state == State::Inrush; }
    uint32_t    episodes()    const { return _episodes; }
    uint32_t    nMicro()      const { return _nMicro; }
    uint32_t    nItrip()      const { return _nItrip; }
    uint32_t    nOcp()        const { return _nOcp; }
    uint32_t    nOther()      const { return _nOther; }
    uint32_t    inrushCount() const { return _inrushCount; }
    uint32_t    verifyOk()    const { return _verifyOk; }
    uint32_t    shortCount()  const { return _shortCount; }
    uint32_t    verifyStray() const { return _verifyStray; }   // VERIFYs that passed despite a tolerated edge
    uint32_t    skippedItrip() const { return _nItripSkipped; }   // ITRIP repeats that did not start an inrush
    uint32_t    itripEscalated() const { return _nItripEscalated; } // ... chains that did after all
    uint32_t    skippedPairs() const { return _nSkipped; }     // "other" pairs that did not start an inrush
    const char* lastReason()  const { return _reason; }
    uint32_t    powerMw()     const { return _pwr.equivalentMw(); }
    // Highest estimate reached since the last stats reset. The estimate
    // itself is a ~128ms average that sits near zero whenever you happen
    // to look; this is the number that shows how close a loco really got
    // to the limit.
    uint32_t    powerMaxMw()  const { return _pwrMaxMw; }

    // nFAULT low-pulse lengths (us) per classification, since the last
    // stats reset. Index: 0 micro short, 1 ITRIP, 2 OCP, 3 other.
    struct LowStats {
        uint32_t n = 0, minUs = 0, maxUs = 0;
        uint64_t sumUs = 0;
        uint32_t avgUs() const { return n ? (uint32_t)(sumUs / n) : 0; }
    };
    const LowStats& lowStats(uint8_t cls) const { return _low[cls < 4 ? cls : 0]; }
    // Classification + pulse lengths of the most recent classified episode.
    const char* lastPulses() const { return _lastPulses; }

    // Micro shorts in time, since the last stats reset (web page).
    //   microMinGapMs   shortest time between two consecutive micro shorts (0 = fewer than two)
    //   microFast       micro shorts that came within MICRO_FAST_MS of the previous one
    //   microBurstMax   most micro shorts seen within any MICRO_FAST_MS window
    static constexpr uint32_t MICRO_FAST_MS = 1000;
    uint32_t    microMinGapMs()  const { return _microMinGapMs; }
    uint32_t    microFast()      const { return _microFast; }
    uint8_t     microBurstMax()  const { return _microBurstMax; }

    static const char* nameOf(State s) {
        switch (s) {
            case State::Normal: return "NORMAL";
            case State::Ignore: return "IGNORE";
            case State::Watch:  return "WATCH";
            case State::Inrush: return "INRUSH";
            case State::Verify: return "VERIFY";
            case State::Shorted:  return "SHORT";
        }
        return "?";
    }

private:
    // Packet in flight when INRUSH is requested: the DMA transfer
    // cannot be interrupted, so the pattern only appears after it
    // (DCCHardware_RP.inc). The designer puts the longest packet at
    // ~13ms. SENSE is not measured during this lead.
    static constexpr uint32_t DCC_MAX_PACKET_US = 13000;
    // One inrush "packet" (25 x 40us). After leaveInrushMode() the one
    // in flight still finishes, so VERIFY does not start counting
    // edges until it is gone (my assumption, not in the designer's
    // text: otherwise the tail of our own pattern could fail VERIFY).
    static constexpr uint32_t INRUSH_PACKET_US  = 1000;
    // Statistics only: nFAULT-to-nFAULT interval that identifies OCP
    // (2ms auto-retry, with margin).
    static constexpr uint32_t OCP_MIN_US = 1500;
    static constexpr uint32_t OCP_MAX_US = 3000;
    static constexpr uint8_t  MAX_WINDOWS = 48;

    static bool reached(uint32_t t, uint32_t deadline) {
        return (int32_t)(t - deadline) >= 0;   // safe across the 71-minute micros() wrap
    }

    // ── time-driven transitions, evaluated at logical time t ──
    void advance(uint32_t t, const Params& p) {
        for (;;) {
            switch (_state) {
            case State::Ignore: {
                uint32_t dl = _t0 + p.ignoreUs;
                if (!reached(t, dl)) return;
                _state      = State::Watch;
                _watchStart = dl;
                break;                       // re-evaluate: WATCH may be over too
            }
            case State::Watch: {
                uint32_t dl = _watchStart + p.watchUs;
                if (!reached(t, dl)) return;
                const char* cls = classify(0, false);
                if (_cls == Cls::Micro) noteMicroTiming();
                const char* pl  = notePulses(cls);
                traceLog().logf(TraceLevel::INFO, TraceSource::HBRIDGE,
                                "%s %s, no repeat within %lums -> normal%s",
                                _name, cls, (unsigned long)(p.watchUs / 1000), pl);
                setNormal();
                return;
            }
            case State::Inrush: {
                uint32_t dl = _inrushStart + p.inrushUs;
                if (!reached(t, dl)) return;
                finishInrush();
                break;                       // now VERIFY
            }
            case State::Verify: {
                uint32_t dl = _verifyBase + p.verifyUs;
                if (!reached(t, dl)) return;
                _verifyOk++;
                if (_verifyEdges) {
                    _verifyStray++;
                    traceLog().logf(TraceLevel::INFO, TraceSource::HBRIDGE,
                                    "%s verify ok after %u stray edge(s) -> normal",
                                    _name, (unsigned)_verifyEdges);
                } else {
                    traceLog().logf(TraceLevel::INFO, TraceSource::HBRIDGE,
                                    "%s verify ok -> normal", _name);
                }
                setNormal();
                return;
            }
            default:
                return;
            }
        }
    }

    // ── one falling edge, at its own timestamp ────────────────
    void onEdge(uint32_t t, const Params& p) {
        switch (_state) {
        case State::Normal:
            _t0           = t;
            _ignoredEdges = 0;
            _pulseN       = 0;
            _pulseMore    = 0;
            _state        = State::Ignore;
            _episodes++;
            {
                // Time since the previous episode of this bridge, in ms
                // (millis, so long gaps are right too; micros wraps).
                uint32_t ms = millis();
                if (_episodes > 1) {
                    uint32_t gap = ms - _lastEpisodeMs;
                    traceLog().logf(TraceLevel::INFO, TraceSource::HBRIDGE,
                                    "%s nFAULT, episode %lu (+%lu.%lus)", _name, (unsigned long)_episodes,
                                    (unsigned long)(gap / 1000), (unsigned long)((gap % 1000) / 100));
                } else {
                    traceLog().logf(TraceLevel::INFO, TraceSource::HBRIDGE,
                                    "%s nFAULT, episode %lu", _name, (unsigned long)_episodes);
                }
                _lastEpisodeMs = ms;
                _episodeMs     = ms;
            }
            break;
        case State::Ignore:
            if (_ignoredEdges < 0xFFFF) _ignoredEdges++;
            break;
        case State::Watch: {
            uint32_t dt = t - _t0;
            const char* cls = classify(dt, true);
            if (p.inrushAllowed && p.skipOther && _cls == Cls::Other && isolatedPair(t, p)) {
                _nSkipped++;
                traceLog().logf(TraceLevel::INFO, TraceSource::HBRIDGE,
                                "%s other, repeat after %luus -> no inrush (running, isolated pair)%s",
                                _name, (unsigned long)dt, notePulses(cls));
                setNormal();
                break;
            }
            if (p.inrushAllowed && p.skipItrip && _cls == Cls::Itrip && itripSkippable(t, p)) {
                _nItripSkipped++;
                traceLog().logf(TraceLevel::INFO, TraceSource::HBRIDGE,
                                "%s itrip, %u edges in ignore, repeat after %luus -> no inrush (running)%s",
                                _name, (unsigned)_ignoredEdges, (unsigned long)dt, notePulses(cls));
                setNormal();
                break;
            }
            if (p.inrushAllowed) enterInrush(cls, dt);
            else                 enterVerifyDirect(cls, dt);
            break;
        }
        case State::Inrush:
            if (_inrushEdges < 0xFFFF) _inrushEdges++;
            if (_winEdges < 255)       _winEdges++;     // for the per-window curve log
            break;
        case State::Verify: {
            // Edges before _verifyStart belong to our own inrush tail.
            if (!reached(t, _verifyStart)) break;
            if (_verifyEdges < 0xFFFF) _verifyEdges++;
            if (_verifyEdges == 1) _verifyFirstUs = t - _verifyStart;
            uint8_t need = p.verifyEdges ? p.verifyEdges : 1;
            if (_verifyEdges >= need) {
                char why[56];
                snprintf(why, sizeof(why), "nFAULT in verify: %u edge(s), 1st after %luus",
                         (unsigned)_verifyEdges, (unsigned long)_verifyFirstUs);
                goShort(why);
            } else {
                // Tolerated for now. A real fault repeats at once (ITRIP every
                // 58/100us, OCP every ~2ms), so give it one more verify window
                // to show itself; a stray micro short does not repeat.
                _verifyBase = t;
            }
            break;
        }
        default:
            break;
        }
    }

    // ── continuous work that is not an edge ───────────────────
    void stage(const Params& p) {
        if (_state == State::Normal) {
            powerStage(p);
        } else if (_state == State::Inrush) {
            if (!p.inrushAllowed) {
                // The bridge that drives the track changed (service mode
                // started or ended) while we were in INRUSH.
                if (_h.leaveInrush) _h.leaveInrush(_h.ctx);
                traceLog().logf(TraceLevel::WARNING, TraceSource::HBRIDGE,
                                "%s inrush abandoned: bridge no longer drives the track", _name);
                setNormal();
                return;
            }
            uint32_t dl = (_winIndex == 0) ? (_inrushStart + DCC_MAX_PACKET_US)
                                           : (_winStart + p.measUs);
            if (reached(_now, dl)) closeWindow();
        }
    }

    void powerStage(const Params& p) {
        if (p.limitMw == 0) {
            _peakMv    = 0;
            _pwrLastUs = _now;
            return;
        }
        uint32_t el = _now - _pwrLastUs;
        if (el < 1000) return;
        uint32_t ms    = el / 1000;
        uint8_t  steps = ms > 16 ? 16 : (uint8_t)ms;
        _pwr.integrate(_peakMv, p.scaleMvA, p.rdsMohm, p.shift, steps);
        _pwrLastUs += ms * 1000;
        _peakMv     = 0;
        uint32_t mw = _pwr.equivalentMw();
        if (mw > _pwrMaxMw) _pwrMaxMw = mw;   // peak hold, for the web page (a snapshot says little)
        if (mw >= p.limitMw) {
            char why[48];
            snprintf(why, sizeof(why), "sustained power %lu mW >= %lu mW",
                     (unsigned long)mw, (unsigned long)p.limitMw);
            goShort(why);
        }
    }

    // ── statistics-only classification ────────────────────────
    enum class Cls : uint8_t { Micro, Itrip, Ocp, Other };
    Cls _cls = Cls::Micro;

    const char* classify(uint32_t dtUs, bool hasWatchEdge) {
        if (_ignoredEdges >= 2) { _cls = Cls::Itrip; _nItrip++; return "ITRIP"; }
        if (hasWatchEdge) {
            if (dtUs >= OCP_MIN_US && dtUs <= OCP_MAX_US) { _cls = Cls::Ocp; _nOcp++; return "OCP"; }
            _cls = Cls::Other; _nOther++; return "other";
        }
        _cls = Cls::Micro;
        _nMicro++;
        return "micro short";
    }

    // Is this "other" pair an isolated one, i.e. safe to leave alone?
    bool isolatedPair(uint32_t t, const Params& p) {
        uint32_t grace = p.ponGraceUs ? p.ponGraceUs : 3000000UL;
        if ((uint32_t)(t - _resetUs) < grace) return false;     // layout still charging
        uint8_t  n = p.burstN ? p.burstN : 4;
        if (n > PAIR_RING) n = PAIR_RING;
        uint32_t w = p.burstUs ? p.burstUs : 250000UL;
        _pairTs[_pairHead] = t;
        _pairHead = (uint8_t)((_pairHead + 1) % PAIR_RING);
        if (_pairCount < PAIR_RING) _pairCount++;
        uint8_t cnt = 0;
        for (uint8_t i = 0; i < _pairCount; i++)
            if ((uint32_t)(t - _pairTs[i]) <= w) cnt++;
        if (cnt >= n) { _pairCount = 0; _pairHead = 0; return false; }   // a cluster
        return true;
    }

    // true = skip this ITRIP repeat. false = power-on grace, or a chain of
    // skipped ITRIP episodes that lasts longer than itripMaxUs (sustained load).
    bool itripSkippable(uint32_t t, const Params& p) {
        uint32_t grace = p.ponGraceUs ? p.ponGraceUs : 3000000UL;
        if ((uint32_t)(t - _resetUs) < grace) return false;
        uint32_t maxUs = p.itripMaxUs ? p.itripMaxUs : 50000UL;
        uint32_t gap   = p.watchUs ? p.watchUs : 20000UL;
        if (_chainN == 0 || (uint32_t)(t - _chainLastUs) > gap) {
            _chainN = 0;
            _chainStart = _t0;
        }
        _chainN++;
        _chainLastUs = t;
        if ((uint32_t)(t - _chainStart) >= maxUs) {
            _chainN = 0;
            _nItripEscalated++;
            traceLog().logf(TraceLevel::WARNING, TraceSource::HBRIDGE,
                            "%s itrip chain of %luus -> sustained, inrush after all",
                            _name, (unsigned long)(t - _chainStart));
            return false;
        }
        return true;
    }

    // ── INRUSH ────────────────────────────────────────────────
    void enterInrush(const char* cls, uint32_t dtUs) {
        _inrushCount++;
        _state       = State::Inrush;
        _inrushStart = _now;
        _inrushEdges = 0;
        _curveN      = 0;
        _winIndex    = 0;
        _winStart    = _now;
        _peakMv      = 0;
        _samples     = 0;
        _leadSamples = 0;
        _winEdges    = 0;
        traceLog().logf(TraceLevel::INFO, TraceSource::HBRIDGE,
                        "%s %s, repeat after %luus (%u edges while ignoring) -> inrush%s",
                        _name, cls, (unsigned long)dtUs, (unsigned)_ignoredEdges, notePulses(cls));
        if (_h.enterInrush) _h.enterInrush(_h.ctx);
    }

    // Close the current measurement window: record its peak.
    // ── micro shorts in time (statistics only) ───────────────
    // Uses the episode's start time. Keeps the last MICRO_RING start
    // times to count how many fell within MICRO_FAST_MS.
    void noteMicroTiming() {
        uint32_t ms = _episodeMs;
        if (_microN) {
            uint8_t  prev = (uint8_t)((_microHead + MICRO_RING - 1) % MICRO_RING);
            uint32_t gap  = ms - _microTs[prev];
            if (_microMinGapMs == 0 || gap < _microMinGapMs) _microMinGapMs = gap ? gap : 1;
            if (gap < MICRO_FAST_MS) _microFast++;
        }
        _microTs[_microHead] = ms;
        _microHead = (uint8_t)((_microHead + 1) % MICRO_RING);
        if (_microN < MICRO_RING) _microN++;
        uint8_t inWin = 0;
        for (uint8_t i = 0; i < _microN; i++)
            if ((uint32_t)(ms - _microTs[i]) < MICRO_FAST_MS) inWin++;
        if (inWin > _microBurstMax) _microBurstMax = inWin;
    }

    // ── nFAULT pulse lengths (statistics only) ───────────────
    // Take every completed pulse that ended at or before t (its rising
    // edge), so a pulse is attributed before the state machine moves on
    // past that moment. Pulses of the current episode are kept; pulses
    // that belong to no episode (an inrush/verify tail, NORMAL) are
    // dropped — only the classification window is of interest.
    void takePulses(NFaultSource& src, uint32_t t) {
        uint32_t f, low;
        while (src.peekPulse(f, low)) {
            if (!reached(t, f + low)) return;      // ends after t: later
            src.dropPulse();
            bool inEpisode = (_state == State::Ignore || _state == State::Watch)
                             && reached(f, _t0);
            if (!inEpisode) continue;
            if (_pulseN < PULSE_MAX) _pulses[_pulseN++] = low;
            else if (_pulseMore < 0xFFFF) _pulseMore++;
        }
    }

    // Called once per classified episode: adds its pulses to the
    // statistics of its class and returns ", low us: a b c (+n)" for the
    // log line (empty if nFAULT was still low / no pulse completed).
    const char* notePulses(const char* clsName) {
        LowStats& st = _low[(uint8_t)_cls < 4 ? (uint8_t)_cls : 0];
        int n = 0;
        _pulseBuf[0] = '\0';
        if (_pulseN) n = snprintf(_pulseBuf, sizeof(_pulseBuf), ", low us:");
        for (uint8_t i = 0; i < _pulseN && n > 0 && n < (int)sizeof(_pulseBuf) - 8; i++) {
            uint32_t v = _pulses[i];
            if (st.n == 0 || v < st.minUs) st.minUs = v;
            if (v > st.maxUs) st.maxUs = v;
            st.sumUs += v; st.n++;
            n += snprintf(_pulseBuf + n, sizeof(_pulseBuf) - n, " %lu", (unsigned long)v);
        }
        if (_pulseMore && n > 0 && n < (int)sizeof(_pulseBuf) - 8)
            snprintf(_pulseBuf + n, sizeof(_pulseBuf) - n, " +%u", (unsigned)_pulseMore);
        if (!_pulseN) snprintf(_pulseBuf, sizeof(_pulseBuf), ", nFAULT still low");
        snprintf(_lastPulses, sizeof(_lastPulses), "%s%s", clsName, _pulseBuf);
        _pulseN = 0;            // counted once, even if logged again later
        _pulseMore = 0;
        return _pulseBuf;
    }

    void closeWindow() {
        if (_curveN < MAX_WINDOWS) {
            _curve[_curveN]      = _peakMv;
            _curveEdges[_curveN] = _winEdges;
            _curveN++;
        }
        if (_winIndex == 0) _leadSamples = _samples;
        _winEdges = 0;
        _peakMv   = 0;
        _samples  = 0;
        _winStart = _now;
        _winIndex++;
    }

    void finishInrush() {
        if (_samples > 0 || _curveN == 0) closeWindow();   // flush the last window
        if (_h.leaveInrush) _h.leaveInrush(_h.ctx);
        logCurve();
        _state       = State::Verify;
        _verifyStart = _now + INRUSH_PACKET_US;
        _verifyBase  = _verifyStart;
        _verifyEdges = 0;
    }

    // Measurement only — nothing is decided from these values yet.
    // Reading the values: SENSE saturates where the chip chops. At ITRIP
    // V(IPROPI) equals VREF, which sits right at the top of the ADC range
    // (a reading of ~3230mV, identical every time, is that ceiling), so
    // anything above ITRIP is invisible here — 5A and 15A both read full
    // scale. A value without an edge count next to it had no nFAULT edge
    // in that window; one WITH an edge count coincided with the chip
    // actually tripping, which is how a real ITRIP-level reading is told
    // from an ADC artefact.
    // curve[0] is the lead period (packet in flight), the rest are
    // windows of _measUs. One line per ~10 values to stay inside the
    // trace line length.
    void logCurve() {
        traceLog().logf(TraceLevel::INFO, TraceSource::HBRIDGE,
                        "%s inrush over after %lums: lead=%umV (%u samples, %u edges), %u nFAULT edges during",
                        _name, (unsigned long)((_now - _inrushStart) / 1000),
                        (unsigned)(_curveN ? _curve[0] : 0), (unsigned)_leadSamples,
                        (unsigned)(_curveN ? _curveEdges[0] : 0), (unsigned)_inrushEdges);
        uint8_t i = 1;
        while (i < _curveN) {
            char buf[TRACE_LOG_MSG_MAX];
            int n = snprintf(buf, sizeof(buf), "%s curve, peak mV[/nFAULT edges] per %luus window:",
                             _name, (unsigned long)_measUs);
            uint8_t cnt = 0;
            while (i < _curveN && cnt < 10 && n > 0 && n < (int)sizeof(buf) - 12) {
                if (_curveEdges[i])
                    n += snprintf(buf + n, sizeof(buf) - n, " %u/%u", (unsigned)_curve[i], (unsigned)_curveEdges[i]);
                else
                    n += snprintf(buf + n, sizeof(buf) - n, " %u", (unsigned)_curve[i]);
                i++; cnt++;
            }
            traceLog().logf(TraceLevel::INFO, TraceSource::HBRIDGE, "%s", buf);
        }
    }

    // A bridge that cannot do inrush (CDE, or the one not driving the
    // track) skips it and goes straight to the verification.
    void enterVerifyDirect(const char* cls, uint32_t dtUs) {
        _state       = State::Verify;
        _verifyStart = _now;
        _verifyBase  = _verifyStart;
        _verifyEdges = 0;
        traceLog().logf(TraceLevel::INFO, TraceSource::HBRIDGE,
                        "%s %s, repeat after %luus -> verify (no inrush on this bridge)%s",
                        _name, cls, (unsigned long)dtUs, notePulses(cls));
    }

    void setNormal() {
        _state     = State::Normal;
        _pwrLastUs = _now;
        _peakMv    = 0;
        _samples   = 0;
    }

    void goShort(const char* why) {
        if (_state == State::Inrush && _h.leaveInrush) _h.leaveInrush(_h.ctx);
        snprintf(_reason, sizeof(_reason), "%s", why);
        _state = State::Shorted;
        _shortCount++;
        traceLog().logf(TraceLevel::WARNING, TraceSource::HBRIDGE,
                        "%s -> SHORT: %s", _name, _reason);
        if (_h.shortCircuit) _h.shortCircuit(_h.ctx, _reason);
    }

    // ── state ─────────────────────────────────────────────────
    const char*    _name = "";
    Hooks          _h{};
    State          _state = State::Normal;
    uint32_t       _now = 0;
    uint32_t       _measUs = 2000;

    uint32_t       _t0 = 0;            // first edge of the episode
    uint32_t       _watchStart = 0;
    uint32_t       _inrushStart = 0;
    uint32_t       _verifyStart = 0;
    static constexpr uint8_t PAIR_RING = 8;
    uint32_t       _resetUs = 0;
    uint32_t       _pairTs[PAIR_RING] = {0};
    uint8_t        _pairHead = 0, _pairCount = 0;
    uint32_t       _nSkipped = 0;
    uint32_t       _nItripSkipped = 0, _nItripEscalated = 0;
    uint32_t       _chainStart = 0, _chainLastUs = 0;
    uint8_t        _chainN = 0;
    uint32_t       _verifyBase = 0;      // VERIFY ends verifyUs after this (moved by a tolerated edge)
    uint16_t       _verifyEdges = 0;     // edges seen in the current VERIFY
    uint32_t       _verifyFirstUs = 0;   // when the first one came, after VERIFY started
    uint16_t       _ignoredEdges = 0;
    uint16_t       _inrushEdges = 0;

    // inrush curve
    uint16_t       _curve[MAX_WINDOWS] = {0};
    uint8_t        _curveEdges[MAX_WINDOWS] = {0};  // nFAULT falling edges seen in each window
    uint8_t        _winEdges = 0;
    uint8_t        _curveN = 0;
    uint8_t        _winIndex = 0;
    uint32_t       _winStart = 0;
    uint16_t       _samples = 0;
    uint16_t       _leadSamples = 0;
    uint16_t       _peakMv = 0;

    // power estimate
    PowerEstimator _pwr;
    uint32_t       _pwrLastUs = 0;
    uint32_t       _pwrMaxMw = 0;

    // statistics
    uint32_t       _episodes = 0, _nMicro = 0, _nItrip = 0, _nOcp = 0, _nOther = 0;
    uint32_t       _inrushCount = 0, _verifyOk = 0, _shortCount = 0, _verifyStray = 0;
    char           _reason[56] = "";

    // nFAULT pulse lengths
    static constexpr uint8_t PULSE_MAX = 8;
    uint32_t       _pulses[PULSE_MAX] = {0};
    uint8_t        _pulseN = 0;
    uint16_t       _pulseMore = 0;
    char           _pulseBuf[64] = "";
    char           _lastPulses[80] = "";
    LowStats       _low[4];

    // episode timing / micro shorts in time
    uint32_t       _lastEpisodeMs = 0;
    uint32_t       _episodeMs = 0;
    static constexpr uint8_t MICRO_RING = 16;
    uint32_t       _microTs[MICRO_RING] = {0};
    uint8_t        _microHead = 0, _microN = 0;
    uint32_t       _microMinGapMs = 0, _microFast = 0;
    uint8_t        _microBurstMax = 0;
};
