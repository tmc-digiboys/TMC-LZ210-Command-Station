// ═══════════════════════════════════════════════════════════════
//  ln_tcp.h  —  LocoNet-over-TCP server (core1)
//
//  Supports BOTH known "LocoNet-over-TCP" client conventions,
//  detected PER CONNECTION from the first byte a client sends:
//
//  1. RAW-BINARY (confirmed via tcpdump: Rocrail's own LnTCP client) —
//     genuine LocoNet message bytes go directly over the TCP stream,
//     message boundaries determined the same way they are on the
//     physical bus itself, from the opcode byte's own length-encoding
//     (see _expectedMsgLen()'s own comment). No framing, no
//     acknowledgement text at all.
//
//  2. ASCII "LbServer" (loconetovertcp.sourceforge.net; confirmed via
//     a second tcpdump: JMRI's own LnTCP client sends literal ASCII
//     text, e.g. "SEND BB 00 43 07\r\n") — human-readable, line-based:
//     client sends "SEND <space-separated hex bytes>\r\n"; server
//     replies with "RECEIVE <hex>\r\n" for every message on the bus
//     (including the echo of what a client itself just sent).
//
//  This module originally implemented ONLY the ASCII variant, then was
//  rewritten to implement ONLY the raw-binary variant once Rocrail's
//  own traffic was captured and shown to be incompatible with it — but
//  Rob then also wanted to connect JMRI, whose own tcpdump showed it
//  uses the ASCII variant instead. Rather than picking one, each
//  client connection is independently classified on its very first
//  received byte: a genuine LocoNet opcode always has bit 7 set
//  (0x80-0xFF, per the spec), while ASCII text's first byte ('S' of
//  "SEND" = 0x53) does not — this is a reliable, one-byte-cheap way to
//  tell the two apart with no ambiguity, and each connection then uses
//  its own detected protocol consistently for both directions (a
//  client's own SEND format governs how RECEIVE relays are formatted
//  back to it, independent of what any other simultaneously-connected
//  client uses).
//
//  A message received via either protocol is genuinely transmitted
//  onto the physical LocoNet bus (so real, physical devices see it
//  too) and relayed back to every connected client regardless of that
//  client's own protocol (each formatted appropriately for it) —
//  including the echo back to the sender itself, matching how a
//  message on the physical bus would be seen by every device on it.
//
//  ARCHITECTURE:
//  This module is BOTH a HardwareModule (TCP server, core1 — same
//  pattern as LenzLan) AND a LocoNetConsumer (Bus.h's own, documented
//  mechanism for "several sources of LocoNet messages to co-exist" —
//  see loconet_module.h's bus() comment). No cross-core bridge is
//  needed: LocoNetModule itself now also runs on core1, so this
//  module calls straight into gLocoNet.bus() from the same core,
//  exactly as SlotServer's own dispatcher already does.
//
//  A received message (either protocol) is handed to
//  gLocoNet.bus().broadcast(msg, nullptr) — the exact same entry
//  point the physical PHY and SlotServer's own dispatcher use for a
//  message that just arrived, so SlotServer's existing per-opcode
//  dispatch (already driving CommandBus/DccHal) sees it too.
//
//  This module's own onMessage() (called by the bus for every
//  message, from every source) relays it to every connected client,
//  each in its own detected protocol's format.
// ═══════════════════════════════════════════════════════════════
#pragma once
#include "module_arch.h"
#include "loconet_module.h"
#include "eeprom_store.h"
#include <Ethernet.h>

#define LN_TCP_MAX_CLIENTS  8     // Max simultaneous TCP connections
#define LN_TCP_MSG_BUF      16    // Matches LnMsg's own data[16] capacity —
                                    // the longest possible single LocoNet message
#define LN_TCP_LINE_BUF     64    // Max ASCII line length (a LocoNet message is
                                    // at most ~16 bytes; as hex-with-spaces that's
                                    // under 50 chars — generous margin)
#define LN_TCP_PORT_DEFAULT 1234  // No standard port is defined by either protocol;
                                    // kept as the default since neither Rocrail nor
                                    // JMRI appear to mandate a specific one —
                                    // configured per-installation on the client side
                                    // regardless.

// ─────────────────────────────────────────────────────────────
//  LnTcpClient — state of a single TCP client connection
//
//  protocol is UNKNOWN until the first byte arrives, at which point it
//  is classified once (see this header's own comment) and never
//  re-evaluated for the lifetime of the connection. Both a raw-message
//  buffer and an ASCII line buffer exist simultaneously (small, and
//  only one is ever actually used per connection) rather than a union,
//  to keep the accumulation logic in loop() straightforward.
// ─────────────────────────────────────────────────────────────
enum class LnTcpProtocol : uint8_t { UNKNOWN, RAW, ASCII };

struct LnTcpClient {
    EthernetClient tcp;
    LnTcpProtocol  protocol;
    uint8_t        msgBuf[LN_TCP_MSG_BUF];   // RAW protocol accumulation
    uint8_t        msgLen;
    char           lineBuf[LN_TCP_LINE_BUF]; // ASCII protocol accumulation
    uint8_t        lineLen;
    bool           active;

    LnTcpClient() : protocol(LnTcpProtocol::UNKNOWN), msgLen(0), lineLen(0), active(false) {}
};

class LnTcp : public HardwareModule, public LocoNetConsumer {
public:
    LnTcp() : HardwareModule("LnTcp", ModuleId::LN_TCP, ModuleCore::CORE1) {}

    // begin() — does not open the listening socket itself; that only
    // happens once "lntcp.enable" is actually on, handled lazily from
    // loop() via _refreshConfig() — same lazy-enable pattern as
    // TraceLog. Registers this module as a LocoNetConsumer on
    // gLocoNet's shared bus unconditionally (cheap; onMessage() itself
    // checks whether relaying is currently enabled/has any clients
    // before doing any real work).
    void begin() override;

    // loop() — core1 only. Re-checks "lntcp.enable"/"lntcp.port" each
    // call, accepts new clients, classifies each client's protocol on
    // its first byte, and accumulates/dispatches complete messages
    // from each connected client's byte stream in whichever protocol
    // that client was classified as using.
    void loop() override;

    // onMessage() — LocoNetConsumer's required override (Bus.h).
    // Called by gLocoNet.bus() for every message from every source.
    // Relays it to every connected client, formatted per that client's
    // own detected protocol (raw bytes, or an ASCII "RECEIVE <hex>"
    // line) — a client with protocol still UNKNOWN (nothing received
    // from it yet) is skipped, since we cannot yet know which format
    // it expects. Returns LN_STATUS::LN_IDLE unconditionally: this
    // consumer never itself reports a bus-level status back
    // (collision, etc.) — it is a passive relay, not a bus participant
    // that could BE the reason a send needs retrying.
    LN_STATUS onMessage(const LnMsg& msg) override;

private:
    LnTcpClient      _clients[LN_TCP_MAX_CLIENTS];
    EthernetServer*  _server        = nullptr;
    bool             _enabledCached = false;
    uint16_t         _portCached    = 0;

    // _refreshConfig() — see TraceLog::_refreshConfig()'s own comment
    // for the identical pattern.
    void _refreshConfig();

    // _acceptNew() — accepts a new connection if the server has one
    // waiting and a free client slot exists. No greeting/VERSION text
    // is sent on connect — sending anything before the client's own
    // first byte would prevent classifying its protocol from that
    // first byte (and a raw-binary client like Rocrail would treat
    // any such greeting as garbage regardless — see this header's own
    // history comment for that exact failure mode).
    void _acceptNew();

    // _expectedMsgLen() — RAW protocol only. Returns the total message
    // length (including the opcode byte itself and the trailing
    // checksum) implied by a LocoNet opcode byte's own top bits, per
    // the standard convention (confirmed against Rob's own tcpdump:
    // opcode 0xBF and 0xA0 both have bits 6-5 = 01, and both were
    // observed as exactly 4-byte messages on the wire):
    //   bits 6-5 = 00 → 2 bytes total
    //   bits 6-5 = 01 → 4 bytes total
    //   bits 6-5 = 10 → 6 bytes total
    //   bits 6-5 = 11 → variable-length: the SECOND byte (once
    //                   available) gives the total length directly
    // Returns 0 if the length cannot yet be determined (variable-
    // length opcode, second byte not yet received) — the caller keeps
    // accumulating bytes until this returns non-zero.
    static uint8_t _expectedMsgLen(const uint8_t* buf, uint8_t haveLen);

    // _processAsciiLine() — ASCII protocol only. Called once a
    // complete line (terminated by CR and/or LF) has been buffered for
    // a client. Only recognises "SEND <hex bytes>" — the only
    // client→server token the protocol defines; anything else is
    // silently ignored, matching the spec's own permissive stance on
    // unrecognised lines.
    void _processAsciiLine(LnTcpClient& c, const char* line);

    // _parseHexBytes() — ASCII protocol only. Parses a SEND parameter's
    // space-separated hex byte sequence (e.g. "BB 00 43 07") into raw
    // bytes. Returns the byte count (0 on any parse failure — empty
    // parameter, a non-hex token, or more bytes than LN_TCP_MSG_BUF
    // can hold).
    static uint8_t _parseHexBytes(const char* param, uint8_t* out, uint8_t outCap);

    // _injectAndRelay() — common path for a complete message from
    // either protocol: builds a genuine LnMsg from the raw bytes and
    // injects it via gLocoNet.bus().broadcast(msg, nullptr) — see this
    // header's own architecture comment for why sender=nullptr (every
    // client, including the one that sent it, sees it relayed back).
    void _injectAndRelay(const uint8_t* data, uint8_t len);

    // _sendRaw()/_sendAsciiReceive() — onMessage()'s two output
    // formats, one per protocol.
    void _sendRaw(LnTcpClient& c, const LnMsg& msg);
    void _sendAsciiReceive(LnTcpClient& c, const LnMsg& msg);
};

extern LnTcp gLnTcp;
