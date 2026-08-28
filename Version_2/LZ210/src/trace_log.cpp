// ═══════════════════════════════════════════════════════════════
//  trace_log.cpp  —  see trace_log.h for the full design rationale
// ═══════════════════════════════════════════════════════════════
#include "trace_log.h"
#include "eeprom_store.h"
#include <stdarg.h>

TraceLog gTraceLog;
TraceSerial traceSerial;

// ─────────────────────────────────────────────────────────────
//  begin() — core1. Deliberately does nothing beyond what
//  ModuleBase's own construction already set up: the listening
//  socket is opened lazily, the first time _refreshConfig() (called
//  from loop()) observes "debug.tcp_log_enable" is on. This mirrors
//  LenzLan/Z21Lan's own begin(), except THEIRS open unconditionally
//  since those protocols are always wanted, whereas TraceLog is a
//  debug-only, off-by-default facility.
// ─────────────────────────────────────────────────────────────
void TraceLog::begin() {
}

// ─────────────────────────────────────────────────────────────
//  loop() — core1 only.
// ─────────────────────────────────────────────────────────────
void TraceLog::loop() {
    _refreshConfig();
    if (!_enabledCached) return;

    // Accept a new client if one is waiting — replaces whatever was
    // previously connected (single-viewer model, see trace_log.h).
    if (_server) {
        EthernetClient incoming = _server->accept();
        if (incoming) {
            if (_client && _client.connected()) _client.stop();
            _client = incoming;
        }
    }

    if (!_client || !_client.connected()) return;

    // Drain a bounded number of pending entries per call, so a large
    // backlog (e.g. logging resumed after being off for a while)
    // cannot itself turn into a long, blocking burst — matching the
    // exact lesson from the earlier ring-buffer dump mistake this
    // project ran into (see trace_log.h's comment). Remaining entries
    // simply wait for the next loop() call.
    const uint8_t kMaxDrainPerCall = 4;
    Entry e;
    for (uint8_t i = 0; i < kMaxDrainPerCall; i++) {
        if (!_pop(e)) break;
        _client.print(e.text);
        _client.print("\r\n");
    }
}

// ─────────────────────────────────────────────────────────────
//  logf() — cross-core-safe (see trace_log.h)
//
//  Deliberately does NOT gate on _enabledCached before pushing: that
//  flag is only known once _refreshConfig() has run at least once
//  (from loop(), core1, after EEPROM/Ethernet are up), so gating here
//  would silently drop every message logged before that point —
//  including the earliest boot-time lines (setup()/setup1()), which
//  is exactly the information Rob wanted preserved even though a
//  viewer only connects later. Always pushing costs a few bytes of
//  formatting even while nothing is enabled/connected, which is an
//  acceptable price for a debug-only facility; only loop() (deciding
//  whether to open a socket, accept a client, or drain the buffer at
//  all) actually gates on _enabledCached.
// ─────────────────────────────────────────────────────────────
void TraceLog::logf(const char* source, const char* fmt, ...) {
    char msg[TRACE_LOG_MSG_MAX];
    int n = 0;
    if (source && source[0]) {
        n = snprintf(msg, sizeof(msg), "[%lu] %s: ", millis(), source);
        if (n < 0) n = 0;
        if ((size_t)n >= sizeof(msg)) n = sizeof(msg) - 1;
    }

    va_list args;
    va_start(args, fmt);
    vsnprintf(msg + n, sizeof(msg) - (size_t)n, fmt, args);
    va_end(args);

    _push(msg);
}

// ─────────────────────────────────────────────────────────────
//  logBytes() — cross-core-safe (see trace_log.h). Same reasoning as
//  logf() for not gating on _enabledCached here — see that function's
//  comment.
// ─────────────────────────────────────────────────────────────
void TraceLog::logBytes(const char* source, const char* direction,
                         const uint8_t* data, uint8_t len) {
    char msg[TRACE_LOG_MSG_MAX];
    int n = snprintf(msg, sizeof(msg), "[%lu] %s %s: ", millis(),
                      source ? source : "", direction ? direction : "");
    if (n < 0) n = 0;
    if ((size_t)n >= sizeof(msg)) n = sizeof(msg) - 1;

    // Each byte takes 3 chars ("XX "); stop before overflowing the
    // buffer rather than truncating mid-byte.
    uint8_t i = 0;
    for (; i < len; i++) {
        if ((size_t)n + 3 >= sizeof(msg)) break;
        n += snprintf(msg + n, sizeof(msg) - (size_t)n, "%02X ", data[i]);
    }
    if (i < len) {
        // Indicate truncation rather than silently dropping bytes
        // without any sign that the line was cut short.
        snprintf(msg + n, sizeof(msg) - (size_t)n, "...(%u more)", len - i);
    }

    _push(msg);
}

// ─────────────────────────────────────────────────────────────
//  _push()/_pop() — lock-free ring buffer, same discipline as
//  EventBus::_push()/_pop() (see event_bus.h/.cpp): the producer
//  (either core) writes the slot fully before advancing _head, and
//  the consumer (core1 only, from loop()) reads the slot before
//  advancing _tail, so neither side ever observes a partially
//  written or partially consumed entry. Returns false (dropping the
//  message) if the buffer is currently full, rather than blocking
//  the caller.
// ─────────────────────────────────────────────────────────────
bool TraceLog::_push(const char* text) {
    uint8_t nextHead = (uint8_t)((_head + 1) % TRACE_LOG_RING_SIZE);
    if (nextHead == _tail) return false;  // ring full — drop newest
    strncpy(_ring[_head].text, text, TRACE_LOG_MSG_MAX - 1);
    _ring[_head].text[TRACE_LOG_MSG_MAX - 1] = '\0';
    _head = nextHead;
    return true;
}

bool TraceLog::_pop(Entry& out) {
    if (_head == _tail) return false;  // empty
    out = _ring[_tail];
    _tail = (uint8_t)((_tail + 1) % TRACE_LOG_RING_SIZE);
    return true;
}

// ─────────────────────────────────────────────────────────────
//  _refreshConfig() — see trace_log.h's declaration comment
// ─────────────────────────────────────────────────────────────
void TraceLog::_refreshConfig() {
    bool     enabled = eepromStore().getBool("debug.tcp_log_enable", false);
    uint16_t port    = eepromStore().getUint16("debug.tcp_log_port", TRACE_LOG_PORT_DEFAULT);

    if (enabled == _enabledCached && port == _portCached) return;  // unchanged

    if (!enabled) {
        // Just turned off — close everything.
        if (_client) _client.stop();
        if (_server) { delete _server; _server = nullptr; }
        _enabledCached = false;
        _portCached    = port;
        return;
    }

    // Turning on, or the port changed while already on — (re)open.
    if (_client) _client.stop();
    if (_server) { delete _server; _server = nullptr; }
    _server = new EthernetServer(port);
    _server->begin();
    _enabledCached = true;
    _portCached    = port;
}
