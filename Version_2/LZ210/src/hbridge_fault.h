#pragma once
// ═══════════════════════════════════════════════════════════════
//  hbridge_fault.h  —  low-active H-bridge nFAULT pin monitoring
//
//  v2 hardware has three onboard H-bridges (DCC main, SM/programming
//  track, CDE), each with its own nFAULT output (HW_DCC_FAULT/
//  HW_SM_FAULT/HW_CDE_FAULT). Per the DRV8874/DRV8876 datasheets
//  (Texas Instruments — both chips share the same DRV887x nFAULT
//  behaviour), this pin is an open-drain output with an external
//  pull-up: HIGH under normal operation, pulled LOW by the chip
//  itself to indicate a fault (overcurrent, overtemperature, charge
//  pump/supply undervoltage). "n" is TI's standard prefix for
//  active-low signals — confirmed directly from the datasheet text:
//  "the nFAULT pin driven low" on a fault condition, with an
//  external "nFAULT Pullup resistor" holding it high otherwise.
//
//  HBridgeFault (below) is a small, reusable class — instantiated
//  once per H-bridge by DccHal, the same "one class, several
//  instances" pattern used elsewhere in this project (see
//  DccCurrentMonitor for the equivalent on the SENSE lines) — rather
//  than writing three nearly identical blocks of fault-checking code.
//  It only detects the raw nFAULT signal; the decision of whether a
//  fault event should actually escalate to a station-wide power-off
//  lives in shouldEscalate() below, and is acted on by DccHal — see
//  its comment for why an isolated fault blip is expected and normal
//  on this hardware, and only repeated faults warrant real action.
//
//  INTERRUPT-DRIVEN, NOT POLLED (Rob, schematic review): nFAULT on
//  this hardware is confirmed Quad-Level 2 (RIMODE = 20kOhm to GND —
//  visible on the LZ210-TMC schematic, "DCC-Out" sheet), which per
//  the DRV8874/DRV8876 datasheet's Table 7-6 means Cycle-By-Cycle
//  current chopping, not the fixed-off-time automatic-retry this
//  class originally assumed. That matters because nFAULT now also
//  pulses for every ordinary chopping event — normal current
//  limiting, e.g. while a loco accelerates — not only for a genuine
//  overcurrent retry. The two are electrically distinguishable only
//  by HOW LONG the pin stays low: per both chips' electrical
//  characteristics tables (identical on this point), a chop is
//  tOFF = 25us typical, while a genuine OCP retry is tRETRY = 2ms
//  typical — a ~80x difference. Catching a 25us pulse reliably is not
//  achievable by polling digitalRead() from loop() (core0 also runs
//  DCC/RS-Bus/command-and-event work, with no guaranteed upper bound
//  on the gap between iterations), so this class now uses
//  attachInterrupt() on CHANGE and times each low period with
//  micros() in the ISR itself — see isrEdge() below.
//
//  DCC and SM drive an individual loco's decoder directly, so they
//  see this per-loco chatter and FILTER chop-class events out
//  entirely (see check()'s filterChops parameter) — they are the
//  driver doing its job, not a fault. CDE drives a booster network,
//  not a loco, so it never sees that per-loco chop pattern in the
//  first place (confirmed, Rob) — a low reading there is already
//  meaningful, so CDE does not filter, and the same duration split
//  is used instead to help quantify what kind of condition it is
//  (an isolated event versus a repeating ~2ms retry pattern, which
//  would suggest a structurally overloaded output rather than a
//  one-off blip).
//
//  DUTY-CYCLE TRACKING (updateDuty()/dutyPermille() below) — added
//  after a first attempt at sustained-overload detection (squaring
//  and leaky-integrating the SENSE/ADC reading, per the hardware
//  designer's own power-dissipation reasoning) turned out unusable in
//  practice: confirmed on the bench (Rob), a loco already known bad
//  (continuous nFAULT chopping on the scope, ~55-60us square wave,
//  seconds long) produced a LOWER integrated SENSE reading at the
//  moment it escalated (307mV) than an ordinary HEALTHY loco's normal
//  peak readings during plain driving (369mV) — i.e. no threshold
//  could separate the two cases. Root cause: sampling the ADC only
//  ~once per ms against a ~55-60us chop period is badly aliased —
//  each sample is essentially a coin flip between the chop's on/off
//  phase, not a meaningful power measurement. Rather than sampling
//  the ADC faster (considered and rejected — see DccHal's own
//  comment on why doing ADC reads from this ISR specifically was
//  judged too risky, a single shared ADC peripheral with no
//  interrupt-safety guarantee in the Arduino framework), duty-cycle
//  tracking reuses the exact, already-interrupt-timed low-period
//  durations this class captures for chop/OCP classification anyway
//  — no ADC involved at all, so no aliasing is possible by
//  construction.
// ═══════════════════════════════════════════════════════════════
#include <Arduino.h>

// ─────────────────────────────────────────────────────────────
//  HBridgeFault — debounced, edge-triggered monitor for one
//  H-bridge's low-active nFAULT pin.
//
//  A single, isolated nFAULT edge is not necessarily a genuine,
//  ongoing problem. On DCC/SM most of what trips nFAULT is the
//  DRV887x's own Cycle-By-Cycle current chopping (confirmed
//  Quad-Level 2, see the top-of-file comment) — routine current
//  limiting, filtered out entirely below. What remains after that
//  filter still has its own automatic-retry behaviour for a genuine
//  overcurrent event (tRETRY = 2ms typical), which briefly disables
//  the outputs and self-recovers WITHOUT needing an external reset —
//  so even a filtered, "real" fault event is often still just the
//  inrush current a decoder's input capacitor draws when a loco first
//  makes rail contact, or the back-EMF/inductive kickback as a
//  motor's magnetic field collapses when it stops, both confirmed
//  present in practice (Rob: faults observed both when a loco is
//  simply moved by hand on the rails, and right as another loco comes
//  to a stop) — neither is a genuine, sustained fault.
//
//  What DOES indicate a genuine problem is faults recurring
//  repeatedly within a short window (the chip's own automatic-retry
//  isn't managing to recover, or a real short/overload is
//  continuously present) — see shouldEscalate() below, which is what
//  DccHal now actually acts on (station-wide power-off), while check()
//  alone is only used for logging every individual blip.
// ─────────────────────────────────────────────────────────────
class HBridgeFault {
public:
    // pin:  the nFAULT GPIO for this bridge (HW_DCC_FAULT/
    //       HW_SM_FAULT/HW_CDE_FAULT)
    // name: short display name for logging ("DCC"/"SM"/"CDE")
    HBridgeFault(uint8_t pin, const char* name)
        : _pin(pin), _name(name) {}

    // Configures the pin as an input with an internal pull-up as a
    // safety backstop (the board already has its own external
    // pull-up per the datasheet's recommended circuit, but an
    // internal one costs nothing and protects against ever reading a
    // floating/undefined level as a false fault if that external
    // pull-up is ever missing or damaged). Does NOT attach the
    // interrupt — call attachIsr() separately once, after begin(),
    // with a free function specific to this instance (see
    // DccHal::begin() for the three call sites — one ISR trampoline
    // function per bridge, since attachInterrupt() needs a plain
    // function pointer, not a bound member function).
    void begin() { pinMode(_pin, INPUT_PULLUP); }

    void attachIsr(void (*isr)()) {
        attachInterrupt(digitalPinToInterrupt(_pin), isr, CHANGE);
    }

    // Called from this bridge's ISR trampoline on EVERY change of the
    // nFAULT pin (both edges). Deliberately minimal and ISR-safe:
    // only micros() (interrupt-safe on RP2350/Arduino-Pico) and a
    // lock-free push into a small ring buffer — no TraceLog, no
    // EEPROM, no millis()-based logic, all of which belong in
    // check() below instead, called from loop() on the same core this
    // interrupt fires on (DccHal is a core0 module — see its own
    // architecture comment — so no cross-core buffer access here).
    //
    // On a FALLING edge, just remembers when. On a RISING edge, the
    // low period that just ended is now fully known — pushes its
    // duration (microseconds) into the ring buffer for check() to
    // classify later. This means a fault is only reported once it has
    // already CLEARED, not at the moment it began — a deliberate
    // trade-off: classifying chop (~25us) vs a genuine OCP retry
    // (~2ms) requires knowing the full duration, which isn't known
    // until the pin goes back high. checkDuration() below is
    // unaffected by this — it polls the live pin directly every
    // loop() iteration, so a fault that never clears at all (no rising
    // edge ever fires) is still caught by it regardless of what this
    // ISR sees.
    void isrEdge() {
        uint32_t now = micros();
        if (digitalRead(_pin) == LOW) {
            _fallUs = now;
            return;
        }
        uint32_t durationUs = now - _fallUs;
        // Sanity cap, duty-tracking only (NOT the ring buffer below —
        // chop/OCP classification still sees the full, uncapped
        // duration). Suspected cause of a reproducible, suspiciously
        // consistent ~25% duty reading after only a brief, harmless
        // inrush blip (Rob, TraceLog, three separate occurrences
        // converging on 25.0-25.4%): electrical noise/bounce on this
        // fast-switching pin around an edge can make digitalRead()
        // here see a different level than the one that actually
        // triggered this ISR call, computing durationUs against a
        // stale _fallUs — a single such misread, tens of thousands of
        // us, is enough to dominate an entire 100ms duty window on its
        // own (0 - 0 + (1000>>2) = exactly 25.0%, matching what was
        // observed). Capped well above the known-legitimate tRETRY
        // scale (~2ms) so no genuine OCP-retry duration is affected.
        uint32_t dutyUs = durationUs;
        if (dutyUs > DUTY_SANITY_CAP_US) dutyUs = DUTY_SANITY_CAP_US;
        _lowUsWindow += dutyUs;  // see updateDuty()'s own comment
        uint8_t next = (uint8_t)((_edgeHead + 1) % EDGE_BUF_SIZE);
        if (next == _edgeTail) { _edgeDropped++; return; }  // consumer not keeping up — drop, don't block
        _edgeBuf[_edgeHead] = durationUs;
        _edgeHead = next;
    }

    // Duty-cycle tracking — see the file-level comment's own note on
    // why this exists (SENSE-based sustained-power detection turned
    // out unusable: aliasing against a ~55-60us chop with ~1ms
    // sampling made a confirmed-bad loco read LOWER than normal
    // peaks on a healthy one). This instead uses the exact,
    // interrupt-timed low-period durations already captured above —
    // no ADC, no aliasing, no new hardware access at all — to track
    // what FRACTION of time nFAULT has actually been low, which
    // should scale directly with how hard the driver is being pushed
    // regardless of chop frequency.
    //
    // Call every loop() iteration (cheap — just a timestamp compare
    // most calls); only does real work once windowUs has elapsed.
    // Each window's raw duty (low-time / windowUs, in permille,
    // 0-1000 = 0.0%-100.0%) is fed through a leaky integrator across
    // windows (same leaky-bucket technique as PowerIntegrator used,
    // minus the SENSE/ADC dependency that made that one unreliable):
    // acc = acc - (acc>>shift) + (rawPermille>>shift). shift controls
    // how many windows of elevated duty it takes to trip — e.g.
    // shift=2 means roughly 4 windows' worth of persistence needed,
    // so one single elevated window (a brief, legitimate burst)
    // doesn't trip it on its own.
    // Returns true exactly when a window just closed and dutyPermille()
    // was freshly updated this call — false otherwise (nothing to
    // report yet). Lets a caller log/react only on actual updates
    // instead of every loop() iteration.
    bool updateDuty(uint32_t windowUs, uint8_t shift) {
        uint32_t now = micros();
        if (now - _windowStartUs < windowUs) return false;
        uint32_t lowUs = _lowUsWindow;
        _lowUsWindow   = 0;
        _windowStartUs = now;
        uint32_t rawPermille = (lowUs * 1000UL) / windowUs;
        if (rawPermille > 1000) rawPermille = 1000;  // clamp — a low period spanning the window boundary could otherwise overshoot slightly
        _dutyAcc = _dutyAcc - (_dutyAcc >> shift) + (rawPermille >> shift);
        return true;
    }

    // Current smoothed duty cycle, in permille (0-1000 = 0.0%-100.0%)
    // — for the web interface and for DccHal's own trip comparison.
    uint16_t dutyPermille() const { return (uint16_t)_dutyAcc; }

    // Resets duty tracking to zero — called on a genuine power-on
    // (DccHal::setPower()), same reasoning as resetDuration(): a
    // previous session's accumulated duty should never carry across
    // a power cycle, and after escalating on it, so the next
    // power-on starts clean rather than already primed near the
    // trip point.
    void resetDuty() { _dutyAcc = 0; _lowUsWindow = 0; }

    // Call every loop() iteration. Drains ISR-captured completed
    // low-periods and classifies each: chopUs is the boundary between
    // a chop pulse (tOFF, ~25us typical) and a genuine OCP retry
    // (tRETRY, ~2ms typical) — see the class comment above. A
    // duration at or below chopUs is a chop.
    //
    // filterChops = true (DCC/SM): chop-class events are silently
    // discarded entirely — not counted anywhere, not recorded, not
    // logged — since they are the driver doing its job, not a fault.
    // filterChops = false (CDE): every event counts, chop-class
    // included, since CDE never sees per-loco chop in the first place
    // (see class comment) — a low reading there is already
    // meaningful.
    //
    // Returns true if at least one qualifying (non-filtered) event was
    // drained this call, and records each into the recent-fault
    // history used by shouldEscalate() below. chopUs is passed in by
    // the caller (DccHal reads it from EEPROM, "dcc.fault_chop_us")
    // — this class deliberately does no EEPROM lookups itself.
    //
    // faultCountRaw() (below) counts every drained event regardless of
    // filterChops — including chop-class ones even when they're
    // filtered out of faultCount() — so the web interface can still
    // show how electrically busy a DCC/SM line really is, chop
    // included, even though none of that reaches the fault/escalation
    // logic.
    bool check(uint32_t chopUs, bool filterChops) {
        bool any = false;
        while (_edgeTail != _edgeHead) {
            uint32_t durationUs = _edgeBuf[_edgeTail];
            _edgeTail = (uint8_t)((_edgeTail + 1) % EDGE_BUF_SIZE);
            _faultCountRaw++;
            if (filterChops && durationUs <= chopUs) continue;  // routine chopping, not a fault
            _recordFault();
            any = true;
        }
        return any;
    }

    // Call after check() returns true, to decide whether this fault
    // event should escalate to a full station-wide power-off (true),
    // or whether it is likely just an isolated, self-recovering blip
    // that the chip's own automatic-retry will already be handling
    // (false — no action needed beyond logging the individual event).
    // Escalates once windowCountMs milliseconds contain at least
    // retryCount recorded faults (both configurable via EEPROM —
    // see DccHal::loop() for the actual keys/defaults).
    //
    // NOTE: with IMODE=GND (fixed off-time, automatic retry — see the
    // class comment above), a genuinely persistent fault does NOT
    // necessarily hold nFAULT continuously low: the chip itself
    // cycles fault→brief pause→retry→fault again for as long as the
    // underlying problem persists. Depending on how that cycle's
    // timing lines up with check()'s debounce and this method's
    // window, a genuine ongoing short could in principle still fail
    // to accumulate enough recorded edges to reach retryCount within
    // windowMs (confirmed by Rob: a real short stopped escalating
    // after this method was introduced, even with retryCount=3, its
    // original default). checkDuration() below is a second,
    // independent signal — call BOTH and escalate on either.
    //
    // ONLY THE NEWEST HBRIDGE_ESCALATE_HISTORY (5) EDGES ARE LOOKED AT,
    // exactly as when the whole history was 5 entries long. That means
    // a retryCount above 5 can never fire — a quirk of the original
    // code, and existing settings (Rob: retry count 20, stays on) rely
    // on it. Widening this to the full history changed that behaviour
    // behind his back (a storm of 20 edges in 400ms then powered off),
    // so it is deliberately kept as it was. The web page labels the
    // limit. Use stormWithin() below for counts above 5.
    bool shouldEscalate(uint32_t windowMs, uint8_t retryCount) const {
        uint32_t now = millis();
        uint8_t count = 0;
        for (uint8_t k = 0; k < HBRIDGE_ESCALATE_HISTORY; k++) {
            uint8_t idx = (uint8_t)((_historyHead + HBRIDGE_FAULT_HISTORY - 1 - k) % HBRIDGE_FAULT_HISTORY);
            if (_history[idx] != 0 && (now - _history[idx]) <= windowMs) count++;
        }
        return count >= retryCount;
    }

    // "Fault storm" test for DccHal::_updateInrush()'s low-sense rule:
    // true if at least `count` recorded edges fall within the last
    // windowMs. Uses the FULL history (up to HBRIDGE_FAULT_HISTORY
    // entries), so counts up to 32 work — unlike shouldEscalate().
    bool stormWithin(uint32_t windowMs, uint8_t count) const {
        uint32_t now = millis();
        uint8_t n = 0;
        for (uint8_t i = 0; i < HBRIDGE_FAULT_HISTORY; i++) {
            if (_history[i] != 0 && (now - _history[i]) <= windowMs) n++;
        }
        return n >= count;
    }

    // Call every loop() iteration (independently of check() — this
    // reads the raw pin directly, not the debounced/edge-tracked
    // state) to detect a fault that has been continuously present for
    // at least thresholdMs. Same pattern as _checkCdeShort()'s
    // continuous-low-duration measurement on the CDE E-signal
    // (dcc_hal.cpp) — a proven approach in this codebase — added here
    // as a second signal alongside shouldEscalate() specifically
    // because that count/window approach alone was confirmed (Rob,
    // empirically) to miss a genuine, persistent short under
    // automatic-retry: some fault conditions (UVLO, thermal shutdown,
    // or a retry cycle time that doesn't reliably clear between
    // debounced samples) hold nFAULT low for a stretch that this
    // catches directly, regardless of how the edge/count side behaves.
    // thresholdMs is EEPROM-configurable (see DccHal::loop()) —
    // Rob may need to tune this empirically (LSA/scope) against the
    // DRV887x's actual retry-cycle timing on this board.
    //
    // clearMs — recovery-confirm debounce, same reasoning as CDE_CLEAR_US
    // (dcc_hal.h): confirmed via TraceLog (Rob) that a genuine, sustained
    // fault condition can present as a dense STORM of brief nFAULT blips
    // (a ~2-second run of individually-debounced "fault edge" events was
    // observed with NO escalation at all, while a much shorter, ~4-edge
    // burst elsewhere DID escalate) — the original, unconditional
    // "if (!faultNow) resetToZero" meant any single HIGH sample caught
    // between two closely-spaced blips wiped out all accumulated
    // duration, so a storm of brief blips could paradoxically go
    // undetected here even though it represents a WORSE underlying
    // condition than an isolated blip. Only treats the pin as genuinely
    // recovered — resetting the duration window — once it has read HIGH
    // continuously for at least clearMs, mirroring exactly how
    // _checkCdeShort() was fixed for the same failure mode.
    bool checkDuration(uint32_t thresholdMs, uint32_t clearMs = 10) {
        bool faultNow = (digitalRead(_pin) == LOW);
        if (!faultNow) {
            if (_durationSinceMs == 0) return false;  // not in a fault window anyway
            if (_recoverSinceMs == 0) { _recoverSinceMs = millis(); return false; }
            if ((millis() - _recoverSinceMs) >= clearMs) {
                _durationSinceMs = 0;
                _recoverSinceMs  = 0;
            }
            return false;
        }
        _recoverSinceMs = 0;  // still faulted — cancel any pending recovery
        if (_durationSinceMs == 0) { _durationSinceMs = millis(); return false; }
        return (millis() - _durationSinceMs) >= thresholdMs;
    }

    // True if the pin currently reads as faulted (level, not edge —
    // for status display, e.g. the web interface).
    bool isFaulted() const { return digitalRead(_pin) == LOW; }

    // Clears checkDuration()'s own timestamp state. Needed by
    // DccHal::_updateInrush(): while inrush handling owns this bridge,
    // checkDuration() is skipped for it (see _updateInrush()'s own
    // comment) — but checkDuration() still runs normally in the brief
    // gaps BETWEEN successive inrush attempts (e.g. a loosely-seated
    // loco bouncing on the rails, faulting several times in quick
    // succession, each individually resolved). If one of those gap
    // samples happens to catch the pin still low, it silently starts
    // (or keeps alive) a duration window that then keeps accumulating
    // across the WHOLE series once inrush hands control back —
    // confirmed, Rob (TraceLog): five separate ~2-6ms inrush-resolved
    // faults within about 50ms, each with a low, safe SENSE reading,
    // still tripped checkDuration()'s escalation right after the last
    // one resolved. Called by _updateInrush() both when a fresh
    // attempt starts (clears any such contamination from before) and
    // when one resolves (so the next independent checkDuration() call
    // starts from a clean baseline, not one contaminated by this
    // attempt's own brief low periods).
    void resetDuration() { _durationSinceMs = 0; _recoverSinceMs = 0; }

    // Short display name for this bridge ("DCC"/"SM"/"CDE").
    const char* name() const { return _name; }

    // Cumulative counts of nFAULT edges seen since boot (or since the
    // last resetFaultCounts() call) — for the web interface's
    // H-bridge status table (Rob: wanted visibility into how often
    // each bridge faults over time, not just its current state; also
    // wanted the raw count specifically, not just the debounced one —
    // see check()'s own comment for why). Purely in-RAM: starts at 0
    // on every boot/reset — no EEPROM persistence, by design (asked
    // for explicitly: counts across the board's uptime, resettable
    // via a button, not meant to survive a power cycle).
    //
    // faultCountRaw(): every completed low-period the ISR captured,
    // chop-class included even when filterChops discards it from
    // everything else.
    // faultCount(): only the ones that passed the chop filter (the
    // same signal shouldEscalate() is built from) — NOT gated on
    // whether that particular edge went on to escalate, so an
    // isolated, harmless one still increments this too. For CDE
    // (filterChops=false), the two are identical.
    uint32_t faultCountRaw() const { return _faultCountRaw; }
    uint32_t faultCount()    const { return _faultCount; }
    void     resetFaultCounts() { _faultCount = 0; _faultCountRaw = 0; }

    // Diagnostic: how many completed low-periods the ISR had to drop
    // because check() wasn't draining the ring buffer fast enough.
    // Should stay at 0 in practice — loop() runs far more often than
    // EDGE_BUF_SIZE fault events should ever queue up between calls.
    uint32_t edgeDropped() const { return _edgeDropped; }

private:
    uint8_t     _pin;
    const char* _name;
    uint32_t    _faultCount    = 0;
    uint32_t    _faultCountRaw = 0;

    // ISR-captured completed low-periods (microseconds), drained by
    // check(). volatile: written from isrEdge() (an interrupt
    // context), read from check() (ordinary loop() context) — without
    // this, the compiler would be free to cache stale values across
    // that boundary. Sized generously (32, matching
    // HBRIDGE_FAULT_HISTORY below) — chop events are frequent during
    // ordinary chopping. Ring-buffer indices are single bytes, so
    // read/write is atomic on this platform without needing a
    // critical section.
    // See isrEdge()'s own comment — a single ISR-computed low-period
    // credited toward duty tracking is capped at this many
    // microseconds, generously above the legitimate tRETRY scale
    // (~2ms), to stop a suspected electrical-noise misread from
    // dominating an entire duty window on its own.
    static constexpr uint32_t DUTY_SANITY_CAP_US = 10000;
    static constexpr uint8_t EDGE_BUF_SIZE = 32;
    volatile uint32_t _edgeBuf[EDGE_BUF_SIZE] = {0};
    volatile uint8_t  _edgeHead = 0;
    volatile uint8_t  _edgeTail = 0;
    volatile uint32_t _fallUs   = 0;  // ISR-internal: timestamp of the most recent falling edge
    volatile uint32_t _edgeDropped = 0;

    // Duty-cycle tracking state — see updateDuty()'s own comment.
    // _lowUsWindow: accumulated low-microseconds within the CURRENT
    // window, written from the ISR (isrEdge()) and read/reset from
    // updateDuty() (ordinary loop() context) — volatile for the same
    // cross-context reason as the edge buffer above.
    volatile uint32_t _lowUsWindow = 0;
    uint32_t _windowStartUs = 0;  // when the current window began (micros())
    uint32_t _dutyAcc       = 0;  // leaky-integrated duty, permille (0-1000)

    // Small ring buffer of recent fault timestamps, used by
    // shouldEscalate() above to detect repeated faults within a
    // short window. Entries older than any realistic window are
    // simply ignored by shouldEscalate()'s own age check, so this
    // never needs explicit pruning.
    // 32 entries, used by stormWithin() (the low-sense fault-storm rule
    // in DccHal::_updateInrush()). shouldEscalate() only looks at the
    // newest 5 of them on purpose — see its comment. Costs 32*4 bytes
    // per bridge; entries older than any realistic window are ignored
    // by the age check anyway.
    static constexpr uint8_t HBRIDGE_FAULT_HISTORY = 32;
    // How many of the newest entries shouldEscalate() considers — the
    // original history size, kept so its behaviour did not change.
    static constexpr uint8_t HBRIDGE_ESCALATE_HISTORY = 5;
    uint32_t _history[HBRIDGE_FAULT_HISTORY] = {0};
    uint8_t  _historyHead = 0;

    // Timestamp the pin was first observed continuously low, used by
    // checkDuration() above. 0 means "not currently in a low stretch".
    uint32_t _durationSinceMs = 0;
    // Timestamp the pin was most recently observed HIGH while already
    // in a fault window (checkDuration()'s clearMs debounce). 0 means
    // "not currently tracking a pending recovery".
    uint32_t _recoverSinceMs  = 0;

    void _recordFault() {
        _faultCount++;
        _history[_historyHead] = millis();
        _historyHead = (_historyHead + 1) % HBRIDGE_FAULT_HISTORY;
    }
};
