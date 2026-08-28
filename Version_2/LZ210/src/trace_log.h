// ═══════════════════════════════════════════════════════════════
//  trace_log.h  —  TCP debug log stream (core1)
//
//  Generic, protocol-agnostic logging: any module (either core) can
//  call traceLog().logf(...) or traceLog().logBytes(...) to send an
//  arbitrary line of text or a raw byte dump to a single connected
//  TCP client, without needing its own dedicated Serial2.printf()
//  debug lines. Built to let the many, protocol-specific "TEMP
//  DIAGNOSTIC" prints added throughout the codebase during debugging
//  sessions be replaced with calls into one shared facility, rather
//  than re-inventing ad-hoc logging in every module.
//
//  Why not just use Serial2? Serial2 is good for quick, low-volume
//  debug output tethered to a USB cable, but doesn't scale to
//  streaming larger volumes of packet-trace data (RX/TX dumps across
//  several protocol modules at once), and requires physical access to
//  the board. This gives the same kind of visibility over the
//  network instead, toggleable at runtime via the web interface —
//  no reboot, no cable.
//
//  Enable/disable and port are EEPROM-configurable ("debug.tcp_log_
//  enable", "debug.tcp_log_port", web interface, default port 23456)
//  and read live each loop() (see _refreshConfig()), matching this
//  project's established pattern for other runtime-adjustable
//  settings (e.g. DccCurrentMonitor::thresholdMv(), Z21Lan's client
//  timeout) — flipping the switch off immediately closes any open
//  connection and stops listening, without a reboot.
//
//  Only ONE client is supported at a time (a debug tool, not a
//  production protocol) — a new connection replaces whatever was
//  previously connected, matching a "last viewer wins" model that
//  keeps the implementation simple.
//
//  ARCHITECTURE — cross-core safety:
//    Actual TCP I/O (EthernetServer/EthernetClient) only ever happens
//    on core1, inside loop() — exactly like every other Ethernet-
//    based module in this project (LenzLan, Z21Lan, ...). But log()/
//    logf()/logBytes() must be safely callable from EITHER core
//    (core0 modules like DccHal or RS-Bus need to log too), so they
//    NEVER touch the socket directly. Instead they format the message
//    and push it into a small lock-free ring buffer (the exact same
//    producer/consumer pattern as EventBus — see event_bus.h's own
//    comment for the full reasoning), and loop() (core1-only) drains
//    that buffer a few entries at a time and writes them to the
//    connected client, if any.
//
//    This also directly avoids the specific mistake made earlier
//    while debugging this project (a diagnostic ring-buffer dump that
//    tried to write its entire contents in one synchronous burst,
//    which measured as a multi-second stall of loop() and starved DCC
//    signal generation): log()/logf() are O(1), bounded, and never
//    block, even when a client is connected and slow to read — if the
//    ring buffer is full, the newest message is simply dropped rather
//    than blocking the calling module's own loop().
// ═══════════════════════════════════════════════════════════════
#pragma once
#include "module_arch.h"
#include <Ethernet.h>

// Ring buffer capacity and per-entry size. Sized generously enough to
// hold the entire boot sequence (setup()/setup1() alone produce
// several dozen lines) plus a bit of headroom for protocol traffic in
// the first seconds after boot, since messages are now always pushed
// regardless of whether logging is enabled/a client is connected yet
// (see logf()'s own comment) — a later-connecting viewer still gets
// the full startup transcript from the beginning, not just whatever
// happened after they connected. 128 entries * 160 bytes = 20KB, a
// small fraction of this board's available RAM. Individual messages
// longer than TRACE_LOG_MSG_MAX are truncated rather than allocated
// dynamically.
#define TRACE_LOG_RING_SIZE   128
#define TRACE_LOG_MSG_MAX     160
#define TRACE_LOG_PORT_DEFAULT 23456

class TraceLog : public ModuleBase {
public:
    TraceLog() : ModuleBase("TraceLog", ModuleId::TRACE_LOG, ModuleCore::CORE1) {}

    // begin() — core1. Does not open the listening socket itself;
    // that only happens once "debug.tcp_log_enable" is actually on,
    // handled lazily from loop() via _refreshConfig(), so an unused
    // TraceLog costs nothing beyond the empty ring buffer and one
    // idle EEPROM read per loop() cycle.
    void begin() override;

    // loop() — core1 only. Re-checks the EEPROM enable flag/port each
    // call (opening/closing the server socket if they changed),
    // accepts a new client if one is waiting (replacing any existing
    // one), and drains a bounded number of pending ring-buffer
    // entries to the connected client, if any.
    void loop() override;

    // logf() — cross-core-safe. Formats `fmt` (printf-style) prefixed
    // with `source` (a short tag identifying the calling module, e.g.
    // "Z21", "XN-RS485", "LNET") and pushes the result into the ring
    // buffer. No-op (cheap: one bool check) if logging is currently
    // disabled. Never blocks; silently drops the message if the ring
    // buffer is full.
    void logf(const char* source, const char* fmt, ...);

    // logBytes() — cross-core-safe convenience for the common "RX/TX
    // packet dump" case: formats source, direction ("RX"/"TX" or any
    // other short tag) and the given bytes as a space-separated hex
    // string, e.g. "[123456] Z21 RX: 53 00 01 A8 FA". Truncates the
    // byte list (rather than the whole line) if it would not fit in
    // TRACE_LOG_MSG_MAX.
    void logBytes(const char* source, const char* direction,
                  const uint8_t* data, uint8_t len);

    // True if a client is currently connected AND logging is enabled
    // — modules with an expensive-to-format message (e.g. one that
    // would otherwise require building a temporary string first) can
    // check this before doing that work at all.
    // Not const: EthernetClient's own connected()/operator bool() are
    // not const-qualified themselves (Ethernet.h), so this can't be
    // marked const either without either an -fpermissive workaround
    // or a `mutable _client` — simplest to just not claim const here,
    // since nothing about this call actually needs it (see webserver.cpp's
    // call site, which is already non-const context via traceLog()).
    bool active() { return _enabledCached && _client && _client.connected(); }

private:
    struct Entry {
        char text[TRACE_LOG_MSG_MAX];
    };

    // Lock-free ring buffer (any core writes via _push(), core1 reads
    // via _pop() from loop()) — same head/tail discipline as
    // EventBus::_push()/_pop(): the producer writes the slot's
    // contents fully before advancing _head, so a concurrent reader
    // never observes a partially written entry.
    Entry             _ring[TRACE_LOG_RING_SIZE];
    volatile uint8_t  _head = 0;
    volatile uint8_t  _tail = 0;

    bool     _enabledCached = false;
    uint16_t _portCached    = 0;

    EthernetServer* _server = nullptr;
    EthernetClient  _client;

    bool _push(const char* text);
    bool _pop(Entry& out);

    // _refreshConfig() — called once per loop(), core1 only. Reads
    // "debug.tcp_log_enable"/"debug.tcp_log_port" live from EEPROM;
    // opens/closes/re-binds the listening socket as needed when
    // either value has changed since the last call, and stops the
    // current client if logging was just turned off.
    void _refreshConfig();
};

extern TraceLog gTraceLog;
inline TraceLog& traceLog() { return gTraceLog; }

// ─────────────────────────────────────────────────────────────
//  TraceSerial — drop-in Serial2 replacement (same print()/
//  println()/printf() interface, via Print's own write()-based
//  implementation) that mirrors every line to TraceLog as well as the
//  real Serial2. Existing call sites are migrated by renaming
//  "Serial2." to "traceSerial." — no other code changes needed, since
//  Print supplies print/println/printf on top of just the two write()
//  overloads below.
//
//  Lines are accumulated until a '\n' is seen, then pushed as one
//  bare TraceLog entry (via logf("", "%s", line) — an empty source
//  suppresses TraceLog's own "[time] source:" prefix, since these
//  lines are already self-describing exactly as Serial2 shows them).
//
//  Cross-core note: write() can be called from either core (Serial2
//  itself is used from both throughout this project — e.g. dcc_hal.cpp
//  on core0, lenz_lan.cpp on core1). The line-accumulation buffer is
//  therefore kept PER CORE (indexed by get_core_num(), the same
//  pattern already used in command_bus.cpp) rather than protected by
//  a lock — consistent with this project's existing lock-free
//  approach (EventBus, TraceLog's own ring buffer) and avoiding any
//  risk of blocking a real-time core over a debug convenience.
// ─────────────────────────────────────────────────────────────
class TraceSerial : public Print {
public:
    size_t write(uint8_t c) override {
        Serial2.write(c);
        _accumulate(c);
        return 1;
    }
    size_t write(const uint8_t *buffer, size_t size) override {
        Serial2.write(buffer, size);
        for (size_t i = 0; i < size; i++) _accumulate(buffer[i]);
        return size;
    }

private:
    struct LineBuf {
        char    text[TRACE_LOG_MSG_MAX];
        uint8_t len = 0;
    };
    LineBuf _line[2];  // index 0 = core0, index 1 = core1

    void _accumulate(uint8_t c) {
        LineBuf& lb = _line[get_core_num() & 1];
        if (c == '\n') {
            lb.text[lb.len] = '\0';
            if (lb.len > 0) traceLog().logf("", "%s", lb.text);
            lb.len = 0;
        } else if (c != '\r') {
            if (lb.len < TRACE_LOG_MSG_MAX - 1) lb.text[lb.len++] = (char)c;
        }
    }
};

extern TraceSerial traceSerial;
