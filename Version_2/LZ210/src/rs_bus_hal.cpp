// ═══════════════════════════════════════════════════════════════
//  rs_bus_hal.cpp  —  RS-Bus feedback HAL
//
//  Wraps the RSbusMaster PIO library to provide RS-Bus feedback
//  module data to the rest of the system via the EventBus.
//
//  The RS-Bus is a half-duplex bus connecting up to 128 feedback
//  encoder modules (e.g. Lenz LR101). Each module has 8 inputs
//  divided into two 4-bit nibbles.
//
//  Operation:
//  - loop() is called every 10ms (throttled)
//  - getRsFeedbacks() is called ONCE per tick (not in a loop — we
//    already poll every 10ms anyway, and the RS-Bus's own physical
//    cycle time (~33ms per 130-pulse round) is slower regardless)
//  - Delta filtering: only nibbles that actually changed are
//    forwarded. Confirmed via logfile analysis (tmc-baan): without
//    filtering, a few continuously-reporting modules flood the
//    CLIENT (its own FIFO/Auto-Accessory queues filled up), not our
//    own CPU
//  - Current-state queries (isSeen/getNibble) read directly from
//    rsBus.fbState[] in the library itself — no own copy of that
//  - Changed pairs are bundled into one FEEDBACK_BULK event so
//    LenzLAN can send them efficiently as 0x42/0x44/.../0x46 packets
//    (header = 0x40 + 2×number of pairs in that packet)
//
//  Module addressing — NOTE, two different numbering schemes:
//    RS-Bus hardware (RSbusMaster, internal to this file): 1-128
//    XpressNet/LAN protocol (§2.15, outward-facing):        0-127
//    RS-Bus module 1 = protocol address 0, module 128 = protocol address 127.
//    All code that communicates outward (LenzLAN/USB dump, XpressNet
//    0x42 response) must therefore apply -1/+1 when converting
//    between these two addressing schemes.
//
//  RS-Bus wire format (RsEntry.data, from the RSbusMaster library):
//    bit 4    = nibble selector (0=low, 1=high)
//    bits 6-5 = decoder type (meaning not yet confirmed)
//    bits 3-0 = nibble value
//    (RSbusMaster.h's own header comment says [6]=nibble — that is a
//    documentation error in the library; the ISR in RSbusMaster.cpp
//    itself uses bit4, and that is the source of truth for what we
//    actually receive)
//
//  FEEDBACK_BULK event format — this is NOT a copy of the internal
//  RSbusMaster format above, but follows §2.15 "Schaltinformation"
//  as prescribed by §2.4.5 ("ADR_x und DAT_x haben das Format wie
//  unter 2.15 beschrieben"):
//    fbBulkAddr[]: protocol address (0-127, i.e. RS-Bus address -1),
//                  NO nibble bit (bit7 always 0)
//    fbBulkData[]: ITNZ — I=0 (not applicable for Rückmeldebaustein),
//                  TT=10 (Kennung "Rückmeldebaustein"), N=nibble
//                  selector (bit4), ZZZZ=nibble value (bits 3-0)
// ═══════════════════════════════════════════════════════════════

#include "rs_bus_hal.h"
#include "trace_log.h"
#include "event_bus.h"
#include "led_status.h"
#include "eeprom_store.h"
#include <cstring>

// Global instance used by all modules
RsBusHal gRsBusHal;

// ─────────────────────────────────────────────────────────────
//  begin() — initialise the RS-Bus master PIO state machine
//
//  Clears the nibble cache (all modules to zero) and initialises the
//  RS-Bus master, but calls stop() because the command station
//  starts up in the power-off state. start() is only called once a
//  POWER_ON event arrives.
// ─────────────────────────────────────────────────────────────
void RsBusHal::begin() {
    memset(_type, 0, sizeof(_type));
    memset(_lastSent, 0, sizeof(_lastSent));

    rsBus.init(RS_BUS_TX_PIN, RS_BUS_RX_PIN, RS_BUS_PIO);
    rsBus.stop();   // the station starts in power-off: RS-Bus not yet active

    // EventBus: without this registration, onEvent() is never called,
    // so rsBus.start() would then never run after this initial stop().
    EventBus::instance().registerHandler(ModuleId::RS_BUS_HAL,
        [](const Event& ev){ gRsBusHal.onEvent(ev); });

    traceLog().logf(TraceLevel::INFO, TraceSource::RS_BUS,
                     "RS-Bus geinitialiseerd (gestopt, wacht op power-on)");
    traceLog().logf(TraceLevel::INFO, TraceSource::RS_BUS,
                     "TX=GP%d RX=GP%d PIO=%s",
                     RS_BUS_TX_PIN, RS_BUS_RX_PIN,
                     (RS_BUS_PIO == pio0) ? "pio0" :
                     (RS_BUS_PIO == pio1) ? "pio1" : "pio2");
}

// ─────────────────────────────────────────────────────────────
//  onEvent() — react to power-on/off events
//
//  POWER_ON:  restarts the RS-Bus master and resets the delta-
//             detection cache to a "never seen" sentinel (0xFF —
//             a value no real 4-bit nibble can ever have), so that
//             the very next time each module naturally reports in
//             via the normal loop()/getRsFeedbacks() polling cycle,
//             it is guaranteed to be treated as a change and
//             broadcast — see below for why this must NOT be done
//             as an immediate, synchronous dump here instead.
//  POWER_OFF: stops the RS-Bus master gracefully (waits for the end
//             of the current cycle).
// ─────────────────────────────────────────────────────────────
void RsBusHal::onEvent(const Event& ev) {
    if (ev.type == EvType::POWER_ON) {
        traceLog().logf(TraceLevel::INFO, TraceSource::RS_BUS, "power-on -> RS-Bus start");
        rsBus.start();
        _powered  = true;
        _lastRxMs = millis();  // start the watchdog window fresh from here

        // IMPORTANT: do NOT reset rsBus.fbState[] here. RS-Bus feedback
        // decoders are not tied to DCC track power (own supply, or
        // powered via the RS-Bus itself) — a DCC power cycle does not
        // reset them, so whatever the library still has recorded remains
        // the best information available (also read directly by the web
        // interface's live feedback table).
        //
        // Reset our own delta cache to a sentinel that can never match a
        // real nibble value (0-15), so that the first report received
        // for each address AFTER this point is always treated as
        // "changed" by the normal delta check in loop(), and therefore
        // gets broadcast. This must NOT be done as an immediate dump
        // built from fbState[] right here: at the moment POWER_ON fires,
        // rsBus.start() has only just been called and modules have not
        // had time to be polled yet — fbState[] is still effectively
        // empty, so an immediate dump finds nothing to send. The correct
        // fix lets the existing polling/broadcast path in loop() do the
        // announcing naturally, as each module actually reports in.
        memset(_lastSent, 0xFF, sizeof(_lastSent));
    } else if (ev.type == EvType::POWER_OFF) {
        traceLog().logf(TraceLevel::INFO, TraceSource::RS_BUS, "power-off -> RS-Bus stop");
        rsBus.stop();
        _powered = false;
    }
}

// ─────────────────────────────────────────────────────────────
//  loop() — poll RS-Bus and publish changed nibbles
//
//  Throttled to 10ms intervals to limit CPU load. The PIO buffers
//  RS-Bus data internally so no messages are missed.
//
//  Each poll cycle:
//  1. getRsFeedbacks() is called ONCE (not in a loop) and returns up
//     to 7 pairs. We already poll every 10ms regardless, so any
//     remainder is simply picked up on the next tick(s) — a loop
//     that drains everything in one go is not needed here (confirmed:
//     the RS-Bus's own physical cycle time, ~33ms for a full 130-pulse
//     round, is slower than our 10ms tick anyway).
//  2. Delta detection per nibble — only values that actually changed
//     are forwarded. Confirmed on the tmc-baan (logfile analysis)
//     that a number of modules keep continuously reporting "nothing
//     changed"; without filtering this floods the CLIENT (its own
//     FIFO/Auto-Accessory queues filled up as a result, causing
//     "queue is full" errors — not our own CPU/core0, as previously
//     assumed).
//  3. Changed nibbles are collected in local arrays (max 28 pairs)
//  4. A single FEEDBACK_BULK event is published with all changes
//
//  Pulses the RS-Bus traffic LED if any changes were detected.
// ─────────────────────────────────────────────────────────────
void RsBusHal::loop() {
    uint32_t now = millis();
    if (now - _lastPollMs < 10) return;  // throttle to 10ms
    _lastPollMs = now;

    uint8_t message[14];
    uint8_t length = 0;

    uint8_t changedAddr[28];
    uint8_t changedData[28];
    uint8_t changedCount = 0;

    // rsbus.delta_filter — configurable via the web interface (RS-Bus
    // panel). Newer feedback decoders already only report on a genuine
    // state change themselves; for those this filter is redundant.
    // Older decoders may report unconditionally on every poll, in which
    // case this filter prevents flooding connected clients. Read live
    // (not cached) so a change via the web interface takes effect on
    // the very next loop() call, without needing a restart.
    bool deltaFilter = eepromStore().getBool("rsbus.delta_filter", true);

    if (rsBus.getRsFeedbacks(message, length)) {
        _lastRxMs = now;  // any genuine poll result counts as "still alive",
                           // even a report that turns out unchanged — see the
                           // watchdog check below for why this matters
        for (uint8_t i = 0; i < length; i++) {
            uint8_t address = message[i * 2];      // RsEntry.address: pure address, 1-128
            uint8_t data     = message[i * 2 + 1];  // RsEntry.data: [6]=nibble,[5:4]=type,[3:0]=value

            // Decoded per RSbusMaster.cpp's ISR (fifoIsrHandler, line 204):
            //   `nb = (data >> 4) & 0x01` — this is the REAL source, not
            //   the header comment in RSbusMaster.h which says [6]=nibble.
            //   The two contradict each other; the ISR code is the source
            //   of truth here because it is literally what computes the
            //   value that ends up in the queue.
            //   bit 4    = nibble selector (0=low, 1=high)
            //   bits 6-5 = decoder type (00=Rückmeldedecoder, 01=Schaltdecoder
            //              with Rückmeldefunktion — presumably the same
            //              TT encoding as §2.15, not yet confirmed)
            //   bits 3-0 = nibble value
            uint8_t nibble  = (data >> 4) & 0x01;
            uint8_t type    = (data >> 5) & 0x03;
            uint8_t nibData = data & 0x0F;

            if (address < 1 || address > 128) continue;

            _type[address] = type;  // kept for inspection (see getType())
            // presence (isSeen) now follows from rsBus.fbState[address].count>0,
            // which the library already tracks itself — no own _seen[] needed anymore.

            bool changed = (_lastSent[address][nibble] != nibData);

            // Logged for EVERY report seen, changed or not, and tagged
            // with which outcome applies — needed to diagnose exactly
            // the kind of problem Rob hit on the tmc-baan: "too many
            // detectors" (a module reporting bogus/noisy data — visible
            // here as rapid changed=true churn for one address) versus
            // "no changes coming through" (reports arrive here but
            // never end up in a published BC, e.g. because deltaFilter
            // masks them, or none arrive here at all — distinguishing
            // those two failure modes from a bare "not working" report
            // needs exactly this per-report visibility).
            traceLog().logf(TraceLevel::DEBUG, TraceSource::RS_BUS,
                             "RX addr=%u nibble=%u type=%u dat=0x%X changed=%d filtered=%d",
                             address, nibble, type, nibData, changed,
                             deltaFilter && !changed);

            _lastSent[address][nibble] = nibData;  // keep the cache up to date either way,
                                                    // so re-enabling the filter later doesn't
                                                    // cause a spurious "changed" burst for
                                                    // values that were already reported while
                                                    // the filter was off
            if (deltaFilter && !changed) continue;  // unchanged, and filtering is enabled

            if (changedCount < 28) {
                // §2.15 "Schaltinformation": ADR is the pure module
                // address (0-127). Address offset: RS-Bus (hardware)
                // counts 1-128, the protocol counts 0-127.
                changedAddr[changedCount] = (address - 1) & 0x7F;
                // ITNZ byte: I=0, TT=type (from the library, not
                // hardcoded!), N=nibble, ZZZZ=nibData. TT indicates the
                // decoder type (00=Rückmeldebaustein, 01=Schaltempfänger
                // with feedback, etc.) — relevant to the client, so
                // simply passed through.
                changedData[changedCount] = ((type & 0x03) << 5)
                                          | (nibble ? 0x10 : 0x00)
                                          | (nibData & 0x0F);
                changedCount++;
            }
        }
    }

    if (changedCount == 0) {
        _checkWatchdog(now);
        return;
    }

    gLeds.onRsBusTraffic();

    traceLog().logf(TraceLevel::DEBUG, TraceSource::RS_BUS,
                     "BC publishing %u changed pair(s)", changedCount);

    // Publish all changes as a single FEEDBACK_BULK event
    Event ev = Ev::feedbackBulk(ModuleId::RS_BUS_HAL,
                                 changedAddr, changedData, changedCount);
    EventBus::instance().publish(ev);
}

// ─────────────────────────────────────────────────────────────
//  getSensorState() — read the cached state of one sensor input
//
//  Returns the last known state of a single input on a feedback module.
//  module: 1-128 (RS-Bus module address)
//  input:  0-7   (0-3 = nibble 0, 4-7 = nibble 1)
//  Returns true if the input is active (high), false if inactive or
//  if module or input is out of range.
// ─────────────────────────────────────────────────────────────
bool RsBusHal::getSensorState(uint8_t module, uint8_t input) const {
    if (module < 1 || module > 128 || input >= 8) return false;
    uint8_t nibble = input / 4;  // inputs 0-3 → nibble 0, inputs 4-7 → nibble 1
    uint8_t bit    = input % 4;  // bit position within the nibble
    return (rsBus.fbState[module].nibble[nibble] >> bit) & 1;
}

// ─────────────────────────────────────────────────────────────
//  clearAll() — reset our own bookkeeping and the library's cache
//
//  Clears _type (last-seen decoder type per address) and _lastSent
//  (last-broadcast nibble cache, used for delta detection). Also
//  zeroes the library's own rsBus.fbState[] table, so a factory
//  reset does not leave any stale library-side data (current values,
//  message counts, error counters) behind for modules that were
//  previously seen.
// ─────────────────────────────────────────────────────────────
void RsBusHal::clearAll() {
    memset(_type,     0, sizeof(_type));
    memset(_lastSent, 0, sizeof(_lastSent));
    memset((void*)rsBus.fbState, 0, sizeof(rsBus.fbState));  // also the library's own table
}

// ─────────────────────────────────────────────────────────────
//  _checkWatchdog() — recover from a stuck RSbusMaster PIO state
//  machine without requiring a full command-station reset.
//
//  Confirmed on the tmc-baan (Rob, log analysis): RS-Bus reception
//  stopped entirely mid-session — no genuine poll result of any kind
//  for over 2.5 minutes — with no POWER_OFF/ON event in between, while
//  XpressNet traffic on the very same core kept responding completely
//  normally throughout (ruling out a general core1/loop() hang). Only
//  a full command-station reset recovered it; toggling track power
//  alone did not (RsBusHal::onEvent()'s own POWER_ON/OFF handlers only
//  start()/stop() the library, they never re-init() the PIO program
//  itself). This points at the underlying RSbusMaster PIO state
//  machine itself getting stuck — plausibly from noise/a bad signal on
//  the physical RS-Bus line (Rob: moving the connection point and
//  disabling one detector made the problem go away for that session),
//  which is outside this codebase's own logic to prevent, but the
//  resulting stuck state can still be recovered from automatically
//  here rather than requiring Rob to power-cycle the whole centrale.
//
//  Only acts while track power is genuinely on (_powered) — RS-Bus
//  decoders can legitimately go quiet for extended periods otherwise
//  (e.g. an unoccupied layout section), so silence alone is only a
//  meaningful signal of a stuck state while trains could plausibly be
//  triggering feedback.
// ─────────────────────────────────────────────────────────────
void RsBusHal::_checkWatchdog(uint32_t now) {
    if (!_powered) return;

    // rsbus.watchdog_s — configurable via the web interface (RS-Bus
    // panel); 0 disables the watchdog entirely, for anyone who'd
    // rather investigate a stall themselves than have it silently
    // recovered.
    uint16_t timeoutS = eepromStore().getUint16("rsbus.watchdog_s", 15);
    if (timeoutS == 0) return;

    if (now - _lastRxMs < (uint32_t)timeoutS * 1000) return;

    traceLog().logf(TraceLevel::WARNING, TraceSource::RS_BUS,
                     "watchdog: no RS-Bus traffic for %us while powered — "
                     "re-initialising PIO", timeoutS);

    rsBus.stop();
    rsBus.init(RS_BUS_TX_PIN, RS_BUS_RX_PIN, RS_BUS_PIO);
    rsBus.start();
    _lastRxMs = now;  // restart the watchdog window from here, not from the
                       // stale silence — avoids re-triggering every loop()
                       // call if the underlying cause hasn't actually cleared
}
