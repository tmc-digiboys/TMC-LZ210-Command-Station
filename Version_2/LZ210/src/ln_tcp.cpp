// ═══════════════════════════════════════════════════════════════
//  ln_tcp.cpp  —  see ln_tcp.h for the full design rationale
// ═══════════════════════════════════════════════════════════════
#include "ln_tcp.h"
#include "trace_log.h"
#include <string.h>
#include <stdlib.h>

LnTcp gLnTcp;

// ─────────────────────────────────────────────────────────────
//  begin() — see ln_tcp.h's own comment
// ─────────────────────────────────────────────────────────────
void LnTcp::begin() {
    gLocoNet.bus().addConsumer(this);
}

// ─────────────────────────────────────────────────────────────
//  loop() — core1 only
// ─────────────────────────────────────────────────────────────
void LnTcp::loop() {
    _refreshConfig();
    if (!_enabledCached) return;

    _acceptNew();

    for (uint8_t i = 0; i < LN_TCP_MAX_CLIENTS; i++) {
        LnTcpClient& c = _clients[i];
        if (!c.active) continue;

        if (!c.tcp.connected()) {
            c.tcp.stop();
            c.active   = false;
            c.msgLen   = 0;
            c.lineLen  = 0;
            c.protocol = LnTcpProtocol::UNKNOWN;
            continue;
        }

        while (c.tcp.available()) {
            // Classify this connection's protocol from its very first
            // byte — see ln_tcp.h's own comment for why this is
            // reliable (bit 7 of a genuine LocoNet opcode is always
            // set; ASCII text's first byte never has it set).
            if (c.protocol == LnTcpProtocol::UNKNOWN) {
                int peeked = c.tcp.peek();
                if (peeked < 0) break;  // nothing actually available yet
                c.protocol = (peeked & 0x80) ? LnTcpProtocol::RAW : LnTcpProtocol::ASCII;
                traceLog().logf(TraceLevel::INFO, TraceSource::LNET,
                                 "TCP client %u classified as %s", i,
                                 c.protocol == LnTcpProtocol::RAW ? "RAW" : "ASCII");
            }

            if (c.protocol == LnTcpProtocol::RAW) {
                if (c.msgLen >= LN_TCP_MSG_BUF) {
                    // Should be unreachable (a genuine LocoNet message is
                    // never longer than LN_TCP_MSG_BUF), but guard against
                    // a misbehaving/malicious client anyway.
                    traceLog().logf(TraceLevel::WARNING, TraceSource::LNET,
                                     "TCP client %u sent oversized RAW message, resyncing", i);
                    c.msgLen = 0;
                }
                c.msgBuf[c.msgLen++] = (uint8_t)c.tcp.read();

                uint8_t want = _expectedMsgLen(c.msgBuf, c.msgLen);
                if (want == 0) continue;        // length not yet knowable (variable-length opcode)
                if (c.msgLen < want) continue;   // still accumulating

                traceLog().logBytes(TraceLevel::DEBUG, TraceSource::LNET, "TCP RX",
                                     c.msgBuf, c.msgLen);
                _injectAndRelay(c.msgBuf, c.msgLen);
                c.msgLen = 0;
            } else {
                char ch = (char)c.tcp.read();
                if (ch == '\r' || ch == '\n') {
                    if (c.lineLen > 0) {
                        c.lineBuf[c.lineLen] = '\0';
                        traceLog().logf(TraceLevel::DEBUG, TraceSource::LNET,
                                         "TCP RX: %s", c.lineBuf);
                        _processAsciiLine(c, c.lineBuf);
                        c.lineLen = 0;
                    }
                    continue;  // empty line (or second half of CRLF) — discard
                }
                if (c.lineLen < LN_TCP_LINE_BUF - 1) {
                    c.lineBuf[c.lineLen++] = ch;
                }
                // else: line too long — silently drop extra characters;
                // the eventual terminator flushes whatever fit, which
                // will fail to parse as a valid SEND and simply be
                // ignored.
            }
        }
    }
}

// ─────────────────────────────────────────────────────────────
//  onMessage() — LocoNetConsumer override, see ln_tcp.h
// ─────────────────────────────────────────────────────────────
LN_STATUS LnTcp::onMessage(const LnMsg& msg) {
    if (!_enabledCached) return LN_STATUS::LN_IDLE;
    for (uint8_t i = 0; i < LN_TCP_MAX_CLIENTS; i++) {
        LnTcpClient& c = _clients[i];
        if (!c.active || !c.tcp.connected()) continue;
        if (c.protocol == LnTcpProtocol::RAW)       _sendRaw(c, msg);
        else if (c.protocol == LnTcpProtocol::ASCII) _sendAsciiReceive(c, msg);
        // UNKNOWN: this client hasn't sent anything yet, so we don't
        // yet know which format it expects — skip it for now.
    }
    return LN_STATUS::LN_IDLE;
}

// ─────────────────────────────────────────────────────────────
//  _refreshConfig() — same pattern as TraceLog::_refreshConfig()
// ─────────────────────────────────────────────────────────────
void LnTcp::_refreshConfig() {
    bool     enabled = eepromStore().getBool("lntcp.enable", false);
    uint16_t port    = eepromStore().getUint16("lntcp.port", LN_TCP_PORT_DEFAULT);

    if (enabled == _enabledCached && port == _portCached) return;

    if (!enabled) {
        for (uint8_t i = 0; i < LN_TCP_MAX_CLIENTS; i++) {
            if (_clients[i].active) { _clients[i].tcp.stop(); _clients[i].active = false; }
        }
        if (_server) { delete _server; _server = nullptr; }
        _enabledCached = false;
        _portCached    = port;
        return;
    }

    if (_server) { delete _server; _server = nullptr; }
    _server = new EthernetServer(port);
    _server->begin();
    _enabledCached = true;
    _portCached    = port;
}

// ─────────────────────────────────────────────────────────────
//  _acceptNew()
// ─────────────────────────────────────────────────────────────
void LnTcp::_acceptNew() {
    if (!_server) return;
    EthernetClient incoming = _server->accept();
    if (!incoming) return;

    for (uint8_t i = 0; i < LN_TCP_MAX_CLIENTS; i++) {
        if (!_clients[i].active) {
            _clients[i].tcp      = incoming;
            _clients[i].active   = true;
            _clients[i].msgLen   = 0;
            _clients[i].lineLen  = 0;
            _clients[i].protocol = LnTcpProtocol::UNKNOWN;
            traceLog().logf(TraceLevel::INFO, TraceSource::LNET,
                             "TCP client connected (slot %u)", i);
            return;
        }
    }
    incoming.stop();
}

// ─────────────────────────────────────────────────────────────
//  _expectedMsgLen() — see ln_tcp.h's own comment
// ─────────────────────────────────────────────────────────────
uint8_t LnTcp::_expectedMsgLen(const uint8_t* buf, uint8_t haveLen) {
    if (haveLen < 1) return 0;
    uint8_t opc = buf[0];
    switch ((opc >> 5) & 0x03) {
        case 0: return 2;
        case 1: return 4;
        case 2: return 6;
        default:  // case 3: variable-length — second byte gives the total length
            if (haveLen < 2) return 0;  // not yet available
            return buf[1];
    }
}

// ─────────────────────────────────────────────────────────────
//  _processAsciiLine() — see ln_tcp.h's own comment
// ─────────────────────────────────────────────────────────────
void LnTcp::_processAsciiLine(LnTcpClient& c, const char* line) {
    (void)c;
    // Only "SEND <hex bytes>" is defined as a client→server token —
    // anything else is silently ignored.
    if (strncmp(line, "SEND", 4) != 0) return;
    const char* param = line + 4;
    while (*param == ' ') param++;

    uint8_t bytes[LN_TCP_MSG_BUF];
    uint8_t len = _parseHexBytes(param, bytes, sizeof(bytes));
    if (len == 0) return;  // malformed — silently ignored, matching the spec's permissive stance

    _injectAndRelay(bytes, len);
    // No "SENT OK/ERROR" reply is sent — that token belongs to the
    // ASCII protocol spec's optional acknowledgement mechanism, which
    // JMRI's own client does not appear to require (its tcpdump showed
    // no indication of waiting for one); the RECEIVE echo from
    // onMessage() above is sent regardless and appears sufficient.
}

// ─────────────────────────────────────────────────────────────
//  _parseHexBytes() — see ln_tcp.h's own comment
// ─────────────────────────────────────────────────────────────
uint8_t LnTcp::_parseHexBytes(const char* param, uint8_t* out, uint8_t outCap) {
    uint8_t count = 0;
    const char* p = param;
    while (*p) {
        while (*p == ' ') p++;
        if (!*p) break;
        char* end = nullptr;
        long v = strtol(p, &end, 16);
        if (end == p) return 0;             // not a valid hex token
        if (v < 0 || v > 0xFF) return 0;     // out of byte range
        if (count >= outCap) return 0;       // exceeds capacity
        out[count++] = (uint8_t)v;
        p = end;
    }
    return count;
}

// ─────────────────────────────────────────────────────────────
//  _injectAndRelay() — see ln_tcp.h's own comment
// ─────────────────────────────────────────────────────────────
void LnTcp::_injectAndRelay(const uint8_t* data, uint8_t len) {
    if (len == 0 || len > sizeof(LnMsg::data)) return;
    LnMsg msg;
    memcpy(msg.data, data, len);
    gLocoNet.bus().broadcast(msg, nullptr);
}

// ─────────────────────────────────────────────────────────────
//  _sendRaw() / _sendAsciiReceive() — onMessage()'s two output formats
// ─────────────────────────────────────────────────────────────
void LnTcp::_sendRaw(LnTcpClient& c, const LnMsg& msg) {
    uint8_t len = msg.length();
    traceLog().logBytes(TraceLevel::DEBUG, TraceSource::LNET, "TCP TX", msg.data, len);
    c.tcp.write(msg.data, len);
}

void LnTcp::_sendAsciiReceive(LnTcpClient& c, const LnMsg& msg) {
    char line[LN_TCP_LINE_BUF];
    int n = snprintf(line, sizeof(line), "RECEIVE");
    uint8_t len = msg.length();
    for (uint8_t i = 0; i < len && (size_t)n + 3 < sizeof(line); i++) {
        n += snprintf(line + n, sizeof(line) - (size_t)n, " %02X", msg.data[i]);
    }
    traceLog().logf(TraceLevel::DEBUG, TraceSource::LNET, "TCP TX: %s", line);
    c.tcp.print(line);
    c.tcp.print("\r\n");
}
