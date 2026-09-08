// ═══════════════════════════════════════════════════════════════
//  trace_log.cpp  —  see trace_log.h for the full design rationale
// ═══════════════════════════════════════════════════════════════
#include "trace_log.h"
#include "eeprom_store.h"
#include <stdarg.h>

TraceLog gTraceLog;
TraceSerial traceSerial;

// ─────────────────────────────────────────────────────────────
//  traceSourceTag()/traceSourceEepromKey()/traceSourceLabel() — the
//  three small per-source lookup tables described in trace_log.h.
//  Kept as three parallel switch statements (rather than one array of
//  structs) so adding a new TraceSource only ever needs one line
//  added to each — a compiler warning on a missing switch case (all
//  three are exhaustive, no default) catches a forgotten entry rather
//  than silently returning something wrong for a new source.
// ─────────────────────────────────────────────────────────────
const char* traceSourceTag(TraceSource src) {
    switch (src) {
        case TraceSource::SYSTEM:   return "";
        case TraceSource::Z21:      return "Z21";
        case TraceSource::XN_LAN:   return "XN-LAN";
        case TraceSource::XN_USB:   return "XN-USB";
        case TraceSource::XN_RS485: return "XN-RS485";
        case TraceSource::XN:       return "XN";
        case TraceSource::LNET:     return "LNET";
        case TraceSource::HBRIDGE:  return "HBridge";
        case TraceSource::CDE:      return "CDE";
        case TraceSource::DCCHAL:   return "DccHal";
        case TraceSource::RS_BUS:   return "RS-Bus";
        case TraceSource::WEB:      return "Web";
        case TraceSource::_COUNT:   break;  // sentinel, never logged under
    }
    return "?";
}

const char* traceSourceEepromKey(TraceSource src) {
    switch (src) {
        case TraceSource::SYSTEM:   return "debug.log_src_system";
        case TraceSource::Z21:      return "debug.log_src_z21";
        case TraceSource::XN_LAN:   return "debug.log_src_xnlan";
        case TraceSource::XN_USB:   return "debug.log_src_xnusb";
        case TraceSource::XN_RS485: return "debug.log_src_xnrs485";
        case TraceSource::XN:       return "debug.log_src_xn";
        case TraceSource::LNET:     return "debug.log_src_lnet";
        case TraceSource::HBRIDGE:  return "debug.log_src_hbridge";
        case TraceSource::CDE:      return "debug.log_src_cde";
        case TraceSource::DCCHAL:   return "debug.log_src_dcchal";
        case TraceSource::RS_BUS:   return "debug.log_src_rsbus";
        case TraceSource::WEB:      return "debug.log_src_web";
        case TraceSource::_COUNT:   break;
    }
    return "debug.log_src_unknown";
}

const char* traceSourceLabel(TraceSource src) {
    switch (src) {
        case TraceSource::SYSTEM:   return "System (boot log, TraceSerial-mirrored lines)";
        case TraceSource::Z21:      return "Z21";
        case TraceSource::XN_LAN:   return "XpressNet (LAN)";
        case TraceSource::XN_USB:   return "XpressNet (USB)";
        case TraceSource::XN_RS485: return "XpressNet (RS485)";
        case TraceSource::XN:       return "XpressNet (shared handler)";
        case TraceSource::LNET:     return "LocoNet";
        case TraceSource::HBRIDGE:  return "H-bridge faults";
        case TraceSource::CDE:      return "CDE booster";
        case TraceSource::DCCHAL:   return "DccHal commands";
        case TraceSource::RS_BUS:   return "RS-Bus feedback";
        case TraceSource::WEB:      return "Web interface (HTTP)";
        case TraceSource::_COUNT:   break;
    }
    return "Unknown";
}

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
//  _passesFilter() — shared level+source check used by both
//  TraceSource-based overloads below, before any formatting begins.
//  Reads "debug.tcp_log_level" (minimum level to show, default INFO —
//  DEBUG-level messages are opt-in) and this source's own enable flag
//  (default true — every source shows by default, matching this
//  facility's pre-filtering behaviour of showing everything once
//  enabled) live from EEPROM each call, so a change via the web
//  interface takes effect immediately.
//
//  Deliberately does NOT ALSO gate on _enabledCached — see logf()'s
//  own comment for why messages are always pushed regardless of
//  whether a viewer is connected yet (preserving the full boot-time
//  transcript for a later-connecting client). Level/source filtering
//  is a genuinely separate concern from that: it controls which
//  messages are considered "wanted" at all, independent of whether
//  anyone happens to be watching right now.
// ─────────────────────────────────────────────────────────────
bool TraceLog::_passesFilter(TraceLevel level, TraceSource source) {
    // Index caching: EepromStore::_indexOf() is a linear search over
    // every registered parameter (~59 by this point in the project,
    // and growing) — fine for the occasional config-page read, but
    // this function runs on EVERY logf()/logBytes() call, including
    // the high-frequency ones covering routine protocol traffic
    // (LNET RX/TX, TCP RX/TX, ...). A running loco's steady stream of
    // LocoNet messages made this add up enough to measurably delay
    // core1's loop() cycle — including the web interface's own
    // request handling, which shares that same core1 loop() (Rob:
    // saving a setting became sluggish specifically while LocoNet
    // traffic was active). Each key's index is looked up once (lazily,
    // on first use) and cached here — a parameter's index never
    // changes after boot, since register*() calls only ever append to
    // the table (never reorder or remove), so this cache stays valid
    // for the entire session.

    static int8_t levelIdx = -2;   // -2 = not yet looked up, -1 = key not found, >=0 = cached index
    static int8_t sourceIdx[(uint8_t)TraceSource::_COUNT];
    static bool   sourceIdxInit = false;
    if (!sourceIdxInit) {
        for (uint8_t i = 0; i < (uint8_t)TraceSource::_COUNT; i++) sourceIdx[i] = -2;
        sourceIdxInit = true;
    }

    if (levelIdx == -2) levelIdx = eepromStore()._indexOf("debug.tcp_log_level");
    uint8_t minLevel = (uint8_t)TraceLevel::DEBUG;
    if (levelIdx >= 0) minLevel = eepromStore()._params[levelIdx].value.u8;
    if ((uint8_t)level < minLevel) return false;

    uint8_t si = (uint8_t)source;
    if (sourceIdx[si] == -2) sourceIdx[si] = eepromStore()._indexOf(traceSourceEepromKey(source));
    if (sourceIdx[si] < 0) return true;  // key not registered — default true, matching getBool()'s own default
    return eepromStore()._params[sourceIdx[si]].value.b;
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
//  viewer only connects later. Always pushing (once past the level/
//  source filter above) costs a few bytes of formatting even while
//  nothing is enabled/connected, which is an acceptable price for a
//  debug-only facility; only loop() (deciding whether to open a
//  socket, accept a client, or drain the buffer at all) actually
//  gates on _enabledCached.
// ─────────────────────────────────────────────────────────────
void TraceLog::logf(TraceLevel level, TraceSource source, const char* fmt, ...) {
    if (!_passesFilter(level, source)) return;

    const char* tag = traceSourceTag(source);
    char msg[TRACE_LOG_MSG_MAX];
    int n = 0;
    if (tag && tag[0]) {
        n = snprintf(msg, sizeof(msg), "[%lu] %s: ", millis(), tag);
        if (n < 0) n = 0;
        if ((size_t)n >= sizeof(msg)) n = sizeof(msg) - 1;
    }

    va_list args;
    va_start(args, fmt);
    vsnprintf(msg + n, sizeof(msg) - (size_t)n, fmt, args);
    va_end(args);

    _push(msg);
}

void TraceLog::logf(const char* source, const char* fmt, ...) {
    // Backward-compatible string-tag wrapper — see trace_log.h's own
    // comment on these overloads. Formats the message itself (rather
    // than forwarding to the TraceSource-based logf() above, which
    // isn't possible with a bare "..." parameter pack without a
    // second va_list round-trip) so existing call sites' free-text
    // tag is preserved exactly as before; only the level/SYSTEM-source
    // filter now additionally applies.
    if (!_passesFilter(TraceLevel::INFO, TraceSource::SYSTEM)) return;

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
void TraceLog::logBytes(TraceLevel level, TraceSource source, const char* direction,
                         const uint8_t* data, uint8_t len) {
    if (!_passesFilter(level, source)) return;

    char msg[TRACE_LOG_MSG_MAX];
    int n = snprintf(msg, sizeof(msg), "[%lu] %s %s: ", millis(),
                      traceSourceTag(source), direction ? direction : "");
    if (n < 0) n = 0;
    if ((size_t)n >= sizeof(msg)) n = sizeof(msg) - 1;

    uint8_t i = 0;
    for (; i < len; i++) {
        if ((size_t)n + 3 >= sizeof(msg)) break;
        n += snprintf(msg + n, sizeof(msg) - (size_t)n, "%02X ", data[i]);
    }
    if (i < len) {
        snprintf(msg + n, sizeof(msg) - (size_t)n, "...(%u more)", len - i);
    }

    _push(msg);
}

void TraceLog::logBytes(const char* source, const char* direction,
                         const uint8_t* data, uint8_t len) {
    // Backward-compatible string-tag wrapper — see trace_log.h's own
    // comment on these overloads.
    if (!_passesFilter(TraceLevel::INFO, TraceSource::SYSTEM)) return;

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
