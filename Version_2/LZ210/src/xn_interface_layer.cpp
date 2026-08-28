// ═══════════════════════════════════════════════════════════════
//  xn_interface_layer.cpp
// ═══════════════════════════════════════════════════════════════
#include "xn_interface_layer.h"
#include "eeprom_store.h"
#include "trace_log.h"
#include <cstring>

// ─────────────────────────────────────────────────────────────
//  process() — intercept interface-local commands, forward
//  everything else to the shared protocol handler.
//
//  Looks at the first two bytes of the incoming frame (header byte
//  `h` and id/sub-command byte `id`) and checks them, in order,
//  against five interface-local commands. Each match builds its
//  reply directly into `r`/`l` and returns XNHandleResult::REPLY
//  immediately, without ever calling into the wrapped handler:
//
//    0xF1/0x01 — "Interface Status" query. In practice this doubles
//    as the keep-alive mechanism: clients send this periodically just
//    to keep the connection alive, which is why it used to be handled
//    ad-hoc, separately, before the protocol handler was even reached
//    — it now lives here instead, alongside the other interface-local
//    commands. Reply: F2 01 01 <xor> (fixed "active" status byte).
//
//    0xF0 — "determine interface version". Reply: 02 36 01 <xor>
//    (fixed hardware/software version bytes).
//
//    0xF1/0x02 — "XpressNet Version" command. Reply: F2 02 36 <xor>
//    (reports XpressNet protocol version 3.6).
//
//    0xF1/0x03 — "number of free connections". This differs per
//    interface (see freeConnFn — LAN queries the real W5100/W5500
//    socket status, USB reports a fixed value), so the actual count
//    is obtained by calling the supplied freeConnFn callback (0 if no
//    callback was supplied). Reply: F2 03 <freeConn> <xor>.
//
//    0xF2/0x01 — "determine/change interface device address". Reads
//    the same SV1 value that the shared handler itself uses, so this
//    reports consistently with what RS485 would report for the same
//    field. Reply: F2 01 <addr> <xor>.
//
//  If the frame does not match any of the above (or is shorter than
//  2 bytes), it is not interface-local: the pending-broadcast buffer
//  is cleared first (any leftover broadcast from a previous call must
//  not bleed into this one), and the frame is forwarded unchanged to
//  the wrapped handler's own process() function, whose result is
//  returned as-is. Any broadcast the handler generates synchronously
//  while doing so arrives via bufferBroadcast() (registered as the
//  handler's broadcast callback by the caller) rather than being sent
//  immediately — see the header comment for why this matters.
// ─────────────────────────────────────────────────────────────
XNHandleResult XnInterfaceLayer::process(XpressNetHandler& handler,
                                          const uint8_t* in, uint8_t il,
                                          uint8_t* r, uint8_t& l,
                                          XnFreeConnFn freeConnFn) {
    // Verify the incoming frame's XOR checksum before acting on ANY
    // of it — including the five interface-local shortcut commands
    // below, which previously matched on in[0]/in[1] directly without
    // this check, unlike everything forwarded to the wrapped handler
    // (whose own process() already calls verifyXor() itself). A
    // corrupted frame could otherwise coincidentally match one of the
    // header/id pairs below and receive a spurious reply. Mirrors the
    // same two-case distinction (too-short vs. XOR mismatch) used in
    // XpressNetHandler::process() itself.
    if (il < 2) {
        l = XpressNetHandler::buildError(r, 0x82); return XNHandleResult::ERROR;
    }
    if (!XpressNetHandler::verifyXor(in, il)) {
        l = XpressNetHandler::buildError(r, 0x80); return XNHandleResult::REPLY;
    }

    if (il >= 2) {
        uint8_t h  = in[0];
        uint8_t id = in[1];

        // 0xF1/0x01 — Interface Status (in practice also serves as a
        // keep-alive: clients send this periodically to keep the
        // connection alive — hence why this used to be handled ad-hoc,
        // separately, before the protocol handler; now it lives here
        // alongside the rest of the interface-local commands).
        if (h == 0xF1 && id == 0x01) {
            r[0] = 0xF2; r[1] = 0x01; r[2] = 0x01;
            r[3] = XpressNetHandler::xorCheck(r, 3); l = 4;
            return XNHandleResult::REPLY;
        }

        // 0xF0 — determine interface version
        if (h == 0xF0) {
            r[0] = 0x02; r[1] = 0x36; r[2] = 0x01;
            r[3] = XpressNetHandler::xorCheck(r, 3); l = 4;
            return XNHandleResult::REPLY;
        }

        // 0xF1/0x02 — XpressNet Version command
        if (h == 0xF1 && id == 0x02) {
            r[0] = 0xF2; r[1] = 0x02; r[2] = 0x36;
            r[3] = XpressNetHandler::xorCheck(r, 3); l = 4;
            return XNHandleResult::REPLY;
        }

        // 0xF1/0x03 — number of free connections (differs per
        // interface, see freeConnFn — LAN queries the real W5100/
        // W5500 socket status, USB reports a fixed value)
        if (h == 0xF1 && id == 0x03) {
            uint8_t freeConn = freeConnFn ? freeConnFn() : 0;
            r[0] = 0xF2; r[1] = 0x03; r[2] = freeConn;
            r[3] = XpressNetHandler::xorCheck(r, 3); l = 4;
            return XNHandleResult::REPLY;
        }

        // 0xF2/0x01 — determine/change interface device address
        // (same "xn.li_address" EEPROM key the shared handler uses —
        // see handleDeviceAddr()'s comment in xpressnet_handler.cpp
        // for why this reads live from EEPROM rather than the cached
        // gCentrale.sv[0] — so this stays consistent with what RS485
        // reports, and with any change made via the web interface)
        if (h == 0xF2 && id == 0x01) {
            uint8_t addr = eepromStore().getUint8("xn.li_address", 1);
            if (addr == 0 || addr > 31) addr = 1;
            r[0] = 0xF2; r[1] = 0x01; r[2] = addr;
            r[3] = XpressNetHandler::xorCheck(r, 3); l = 4;
            return XNHandleResult::REPLY;
        }
    }

    // Not interface-local: pass through to the shared protocol logic.
    // Any broadcast generated synchronously while doing so arrives via
    // bufferBroadcast() into _pending — it is not sent out immediately.
    // The caller retrieves it itself, after sending its own direct
    // response (see the header comment).
    _pendingLen = 0;
    // Generic handoff log — the exact point every non-interface-local
    // frame reaches the shared XpressNetHandler, for whichever
    // transport this instance serves (LenzLan or LenzUsb both call
    // through this same process() — handler.srcId() distinguishes
    // them so the log line is labelled correctly either way) (Rob).
    traceLog().logBytes(handler.srcId() == ModuleId::LENZ_USB ? "XN-USB" : "XN-LAN",
                         "-> XN", in, il);
    return handler.process(in, il, r, l);
}

// bufferBroadcast() — stores up to sizeof(_pending) bytes of a
// broadcast frame for later retrieval, instead of sending it right
// away. Silently ignores calls with len==0 or len exceeding the
// internal buffer size. Intended to be wired up as the wrapped
// handler's broadcast callback (see process()).
void XnInterfaceLayer::bufferBroadcast(const uint8_t* data, uint8_t len) {
    if (len == 0 || len > sizeof(_pending)) return;
    memcpy(_pending, data, len);
    _pendingLen = len;
}

// takePendingBroadcast() — if a broadcast is currently buffered,
// copies it into the caller-supplied buffer and clears the internal
// pending state; returns the number of bytes copied (0 if nothing was
// pending, in which case the output buffer is left untouched).
uint8_t XnInterfaceLayer::takePendingBroadcast(uint8_t* buf) {
    uint8_t n = _pendingLen;
    if (n > 0) memcpy(buf, _pending, n);
    _pendingLen = 0;
    return n;
}

// ─────────────────────────────────────────────────────────────
//  Interface-specific error/acknowledgement responses (spec §1
//  Allgemeines). Each of these builds one fixed 3-byte frame into the
//  caller-supplied buffer and returns its length (always 3). None of
//  them inspect any state — they simply encode the literal byte
//  sequences defined by the XpressNet LAN/USB interface specification
//  for these three conditions.
// ─────────────────────────────────────────────────────────────

// buildFrameTimeoutError() — builds the "01/01/00" response, sent by
// the transport layer when a client starts sending a frame but fails
// to complete it within the allowed time window (~250ms).
uint8_t XnInterfaceLayer::buildFrameTimeoutError(uint8_t* r) {
    r[0] = 0x01; r[1] = 0x01; r[2] = 0x00;
    return 3;
}

// buildBufferOverflowError() — builds the "01/06/07" response, sent
// by the transport layer when its receive buffer would overflow
// because too much data arrived without a separating frame marker.
uint8_t XnInterfaceLayer::buildBufferOverflowError(uint8_t* r) {
    r[0] = 0x01; r[1] = 0x06; r[2] = 0x07;
    return 3;
}

// buildCommandSentAck() — builds the "01/04/05" acknowledgement,
// confirming that a command which normally produces no direct
// response has been successfully handed off to the command station.
uint8_t XnInterfaceLayer::buildCommandSentAck(uint8_t* r) {
    r[0] = 0x01; r[1] = 0x04; r[2] = 0x05;
    return 3;
}
