#include "z21_lan.h"
#include "rs_bus_hal.h"
#include "trace_log.h"

Z21Lan gZ21Lan;

// Z21 header constants
namespace Z21Hdr {
    constexpr uint16_t LAN_GET_SERIAL_NUMBER = 0x0010;
    constexpr uint16_t LAN_GET_VERSION       = 0x001A;
    constexpr uint16_t LAN_GET_STATUS        = 0x0021;
    constexpr uint16_t LAN_SET_BROADCASTFLAGS= 0x0050;
    constexpr uint16_t LAN_GET_BROADCASTFLAGS= 0x0051;
    constexpr uint16_t LAN_GET_LOCOMODE      = 0x0060;
    constexpr uint16_t LAN_SET_LOCOMODE      = 0x0061;
    constexpr uint16_t LAN_GET_TURNOUTMODE   = 0x0070;
    constexpr uint16_t LAN_SET_TURNOUTMODE   = 0x0071;
    constexpr uint16_t LAN_SYSTEMSTATE_DATACHANGED = 0x0084;
    constexpr uint16_t LAN_SYSTEMSTATE_GETDATA     = 0x0085;
    constexpr uint16_t LAN_LOGON             = 0x0030;
    constexpr uint16_t LAN_LOGOFF            = 0x0031;
    constexpr uint16_t LAN_X_BUS             = 0x0040;
    constexpr uint16_t LAN_CV_READ           = 0x0023;
    constexpr uint16_t LAN_CV_WRITE          = 0x0024;
    // FLAG: per the official Z21 LAN Protocol Specification, 0x00A4
    // is genuinely LAN_LOCONET_DETECTOR (independently confirmed
    // earlier this session, decoding detector-poll traffic against the
    // same spec). The standalone "LAN_CV_POM_READ"/"LAN_CV_POM_WRITE"
    // headers that used to be defined here (0x00A4/0x00A8) do not
    // correspond to any genuine, documented top-level Z21 header —
    // real Z21 clients request POM operations through the LAN_X
    // tunnel instead (X-Header 0xE6, DB0=0x30, already correctly
    // handled via _handleXBus() -> the shared XpressNetHandler's
    // handlePOM()). Removed the colliding entries and their dispatch
    // cases/handler functions entirely, rather than just reassigning
    // them, since they were not implementing anything a genuine client
    // would ever actually send this way.
    //
    // LAN_LOCONET_DETECTOR itself is not yet dispatched to any handler
    // — genuine detector-status polls (confirmed arriving from
    // Rocrail, three poll types: 0x80/0x81/0x82) are still silently
    // ignored, same as before, just no longer at risk of being
    // misrouted into POM logic. Reporting real occupancy-detector
    // state back is a separate feature needing its own design (what
    // data we'd actually report), left for a future session.
    constexpr uint16_t LAN_LOCONET_DETECTOR  = 0x00A4;
    constexpr uint16_t LAN_LOCO_SPEED        = 0x00A0;
    constexpr uint16_t LAN_LOCO_FUNCTION     = 0x00F8;
    constexpr uint16_t LAN_RMBUS_DATACHANGED = 0x0080;
    constexpr uint16_t LAN_RMBUS_GETDATA      = 0x0081;
    constexpr uint16_t LAN_RMBUS_PROGRAMMODULE = 0x0082;
    constexpr uint16_t LAN_FAST_CLOCK_DATA     = 0x00CC;
    constexpr uint16_t LAN_FAST_CLOCK_SETTINGS     = 0x00CE;
    constexpr uint16_t LAN_FAST_CLOCK_SETTINGS_SET = 0x00CF;
    constexpr uint16_t LAN_LOCONET_DISPATCH_ADDR = 0x00A3;
}

// ─────────────────────────────────────────────────────────────
//  begin() — initialise the Z21 UDP server
//
//  Opens the UDP listener on the configured port (EEPROM key
//  "z21.port", web interface, default Z21_PORT/21105), clears all
//  client slots to their inactive/empty state, resets the
//  connected-client counter, and registers this module's onEvent()
//  method as the EventBus handler for ModuleId::Z21_LAN, so hardware
//  events published by other modules (power state, loco state,
//  RS-Bus feedback) are translated into Z21 broadcasts.
// ─────────────────────────────────────────────────────────────
void Z21Lan::begin() {
    uint16_t port = eepromStore().getUint16("z21.port", Z21_PORT);
    _udp.begin(port);
    memset(_clients, 0, sizeof(_clients));
    _clientCount = 0;

    EventBus::instance().registerHandler(ModuleId::Z21_LAN,
        [](const Event& ev){ gZ21Lan.onEvent(ev); });
}

// ─────────────────────────────────────────────────────────────
//  loop() — receive and dispatch one UDP packet per call, and check
//  client timeouts
//
//  Polls the UDP socket for a waiting packet. If one is present and
//  fits within the fixed-size receive buffer, reads it in full, looks
//  up (or creates) the client entry for the sender's IP+port, and
//  hands the raw bytes to _processPacket() for parsing and dispatch.
//  Packets larger than Z21_UDP_BUF are silently ignored (parsePacket()
//  still reports their size, but the read is skipped since it would
//  not fit). Always finishes by checking for and removing any clients
//  that have exceeded the inactivity timeout.
// ─────────────────────────────────────────────────────────────
void Z21Lan::loop() {
    int pktSize = _udp.parsePacket();
    if (pktSize > 0 && pktSize <= Z21_UDP_BUF) {
        uint8_t buf[Z21_UDP_BUF];
        _udp.read(buf, pktSize);
        Z21Client* c = _findOrAddClient(_udp.remoteIP(), _udp.remotePort());
        if (c) _processPacket(buf, pktSize, c->ip, c->port);
    }
    _checkTimeouts();
}

// ─────────────────────────────────────────────────────────────
//  Packet dispatch
// ─────────────────────────────────────────────────────────────

// ─────────────────────────────────────────────────────────────
//  _processPacket() — validate and route one incoming Z21 UDP packet
//
//  A Z21 packet starts with a 4-byte header: a little-endian 16-bit
//  total length (including this header) followed by a little-endian
//  16-bit command header value. Bails out if the packet is shorter
//  than this 4-byte header, or if the length field does not match the
//  number of bytes actually received (a mismatch suggests a corrupt
//  or malformed packet, which is safest to simply ignore).
//
//  Looks up (or creates) the client entry for the sender's IP+port
//  and refreshes its last-seen timestamp (this keeps the client alive
//  even for commands that do not go through loop()'s own lookup,
//  and updates the timestamp on every packet, not just the first one
//  from a given client).
//
//  Switches on the 16-bit header value and calls the matching
//  _handle*() method with the payload bytes (everything after the
//  4-byte header) and their length. Two header values (LAN_LOGON and
//  LAN_LOGOFF, and several others) don't need any payload and are
//  called with no extra arguments. Header values with no matching
//  case (including LAN_SET_LOCOMODE, which is accepted but
//  intentionally not acted upon since Motorola mode is not
//  implemented) fall through to the default case and are silently
//  ignored.
// ─────────────────────────────────────────────────────────────
void Z21Lan::_processPacket(const uint8_t* buf, uint16_t len,
                              const IPAddress& ip, uint16_t port) {
    if (len < 4) return;
    uint16_t pktLen = _u16le(buf);
    uint16_t hdr    = _u16le(buf + 2);
    if (pktLen != len) return;

    Z21Client* c = _findOrAddClient(ip, port);
    if (!c) return;
    c->lastSeenMs = millis();

    const uint8_t* data    = buf + 4;
    uint8_t        dataLen = (uint8_t)(len - 4);

    // Generic RX log for every top-level Z21 command (GetSerialNumber,
    // GetStatus, RMBUS_GETDATA, SystemState, FastClock, LocoNet
    // dispatch, ...). LAN_X_BUS commands are ALSO logged again inside
    // _handleXBus() itself, under the same "Z21"/"RX" tag — harmless
    // duplication for that one case, in exchange for every OTHER
    // top-level command (previously entirely uncovered, e.g. RS-Bus's
    // own direct GETDATA request/reply — Rob) now being logged too.
    traceLog().logBytes("Z21", "RX", buf, len);

    switch (hdr) {
        case Z21Hdr::LAN_GET_SERIAL_NUMBER:  _handleLanGetSerialNumber(*c); break;
        case Z21Hdr::LAN_GET_VERSION:        _handleLanGetVersion(*c);      break;
        case Z21Hdr::LAN_GET_STATUS:         _handleLanGetStatus(*c);       break;
        case Z21Hdr::LAN_SET_BROADCASTFLAGS: _handleSetBcFlags(*c, data, dataLen); break;
        case Z21Hdr::LAN_GET_BROADCASTFLAGS: _handleGetBcFlags(*c);         break;
        case Z21Hdr::LAN_SYSTEMSTATE_GETDATA:  _handleSystemStateGet(*c);   break;
        // CC 00 → GET/SET/START/STOP dispatcher
        // CD 00 → LAN_FAST_CLOCK_SETTINGS_GET (data[0]==0x04)
        case Z21Hdr::LAN_FAST_CLOCK_DATA:      _handleFastClockSet(*c, data, dataLen); break;
        case Z21Hdr::LAN_FAST_CLOCK_SETTINGS:     _handleFastClockSettingsGet(*c);               break;
        case Z21Hdr::LAN_FAST_CLOCK_SETTINGS_SET: _handleFastClockSettingsSet(*c, data, dataLen); break;
        case Z21Hdr::LAN_RMBUS_GETDATA:        _handleRmbusGetData(*c, data, dataLen); break;
        case Z21Hdr::LAN_LOCONET_DISPATCH_ADDR: _handleLocoNetDispatch(*c, data, dataLen); break;
        case Z21Hdr::LAN_GET_LOCOMODE:    _handleGetLocoMode(*c, data, dataLen);    break;
        case Z21Hdr::LAN_SET_LOCOMODE:    break;
        case Z21Hdr::LAN_GET_TURNOUTMODE: _handleGetTurnoutMode(*c, data, dataLen); break;
        case Z21Hdr::LAN_SET_TURNOUTMODE: _handleSetTurnoutMode(*c, data, dataLen); break;
        case Z21Hdr::LAN_LOGON:              _handleLogon(*c);              break;
        case Z21Hdr::LAN_LOGOFF:             _handleLogoff(*c);             break;
        case Z21Hdr::LAN_X_BUS:             _handleXBus(*c, data, dataLen); break;
        case Z21Hdr::LAN_LOCO_SPEED:        _handleLocoSpeed(*c, data, dataLen); break;
        case Z21Hdr::LAN_LOCO_FUNCTION:     _handleLocoFunction(*c, data, dataLen); break;
        case Z21Hdr::LAN_CV_READ:           _handleCvRead(*c, data, dataLen);       break;
        case Z21Hdr::LAN_CV_WRITE:          _handleCvWrite(*c, data, dataLen);      break;
        default: break;
    }
}

// ─────────────────────────────────────────────────────────────
//  Z21 information replies
// ─────────────────────────────────────────────────────────────

// ─────────────────────────────────────────────────────────────
// _handleLanGetSerialNumber() — LAN_GET_SERIAL_NUMBER: replies with a
// fixed, made-up serial number (0x00000001). Real Z21 clients mostly
// use this only to have some identifying number to display; the exact
// value is not otherwise significant to protocol behaviour.
// ─────────────────────────────────────────────────────────────
void Z21Lan::_handleLanGetSerialNumber(Z21Client& c) {
    uint8_t buf[8];
    _putU16le(buf, 8);
    _putU16le(buf+2, 0x0010);
    _putU16le(buf+4, 0x0001);
    _putU16le(buf+6, 0x0000);
    _send(c, buf, 8);
}

// ─────────────────────────────────────────────────────────────
// _handleLanGetVersion() — LAN_GET_VERSION: replies with a fixed
// hardware type / firmware version response, formatted the way a
// real Z21 central station would (used by clients to identify what
// kind of station they are talking to).
// ─────────────────────────────────────────────────────────────
void Z21Lan::_handleLanGetVersion(Z21Client& c) {
    uint8_t buf[12];
    _putU16le(buf,   12);
    _putU16le(buf+2, Z21Hdr::LAN_GET_VERSION);
    buf[4] = 0x00; buf[5] = 0x02; buf[6] = 0x00; buf[7] = 0x00;
    buf[8] = 0x30; buf[9] = 0x01; buf[10] = 0x00; buf[11] = 0x00;
    _send(c, buf, 12);
}

// ─────────────────────────────────────────────────────────────
// _handleLanGetStatus() — LAN_GET_STATUS: replies with the current
// system state packet (power/emergency-stop flags), built via
// _buildSystemState().
// ─────────────────────────────────────────────────────────────
void Z21Lan::_handleLanGetStatus(Z21Client& c) {
    uint16_t len = 0;
    _buildSystemState(_txBuf, len);
    _send(c, _txBuf, len);
}

// ─────────────────────────────────────────────────────────────
// _handleSetBcFlags() — LAN_SET_BROADCASTFLAGS: stores the requested
// 32-bit broadcast-flag bitmask (little-endian, 4 bytes) for this
// client. From then on, this client only receives broadcasts whose
// event type matches one of the flags it has set (see _broadcast()
// and onEvent()). Does nothing if fewer than 4 bytes were supplied.
// ─────────────────────────────────────────────────────────────
void Z21Lan::_handleSetBcFlags(Z21Client& c,
                                 const uint8_t* data, uint8_t len) {
    if (len >= 4) {
        c.bcFlags = (uint32_t)data[0] | ((uint32_t)data[1] << 8)
                  | ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
    }
}

// ─────────────────────────────────────────────────────────────
//  _handleLogon() — LAN_LOGON: send the current full state to a
//  newly logging-on client
//
//  Sends the current system state packet first (same payload as
//  LAN_GET_STATUS/the power-change broadcast), so the client
//  immediately knows whether track power is on, off, or in emergency
//  stop.
//
//  Then sends the current RS-Bus feedback state for every group of
//  10 module addresses (13 groups cover the full 1-128 address
//  range), one LAN_RMBUS_DATACHANGED packet per group, packing both
//  nibbles of each of the 10 addresses in that group into 10 bytes.
//  A group is only sent if at least one of its addresses has ever
//  been seen on the RS-Bus (gRsBusHal.isSeen()) — but if so, the
//  whole group is sent even if some of its individual values are
//  zero, since the client still needs to know that a module is
//  present and its inputs are currently all open/off, rather than
//  simply never hearing about that group at all.
// ─────────────────────────────────────────────────────────────
void Z21Lan::_handleLogon(Z21Client& c) {
    // Send system state first
    uint16_t len = 0;
    _buildSystemState(_txBuf, len);
    _send(c, _txBuf, len);

    // Send current RS-Bus feedback state for all active modules.
    // Z21 format: LAN_RMBUS_DATACHANGED, one group (10 modules) per packet.
    // Groups 0-12 cover addresses 1-128. Only send groups with non-zero data.
    for (uint8_t group = 0; group < 13; group++) {
        uint8_t baseAddr = group * 10 + 1;
        // Check if any module in this group has been seen (is present on the RS-Bus).
        // Send the group even if all values are 0 — the client needs to know
        // the module is present and all inputs are open.
        bool hasSeen = false;
        for (uint8_t i = 0; i < 10; i++) {
            uint8_t addr = baseAddr + i;
            if (addr > 128) break;
            if (gRsBusHal.isSeen(addr)) { hasSeen = true; break; }
        }
        if (!hasSeen) continue;  // skip groups with no known modules

        uint8_t buf[15];
        _putU16le(buf,   15);
        _putU16le(buf+2, Z21Hdr::LAN_RMBUS_DATACHANGED);
        buf[4] = group;
        memset(buf+5, 0, 10);
        for (uint8_t i = 0; i < 10; i++) {
            uint8_t addr = baseAddr + i;
            if (addr > 128) break;
            uint8_t n0 = gRsBusHal.getNibble(addr, 0);
            uint8_t n1 = gRsBusHal.getNibble(addr, 1);
            buf[5 + i] = (n1 << 4) | (n0 & 0x0F);
        }
        _send(c, buf, 15);
    }
}

// _handleLogoff() — LAN_LOGOFF: immediately deactivates this client's
// slot (rather than waiting for it to time out) and decrements the
// connected-client counter, freeing the slot for reuse by a future
// connection.
void Z21Lan::_handleLogoff(Z21Client& c) {
    c.active    = false;
    c.bcFlags   = 0;
    _clientCount--;
}

// ─────────────────────────────────────────────────────────────
//  XBus (XpressNet embedded in Z21)
// ─────────────────────────────────────────────────────────────
static XpressNetHandler sZ21XpressNet;
static bool sZ21Init = false;

// ─────────────────────────────────────────────────────────────
//  _handleXBus() — LAN_X_BUS: handle one XpressNet command tunnelled
//  inside a Z21 UDP packet
//
//  Does nothing if fewer than 2 payload bytes are present (not even
//  enough for an XpressNet header+id byte pair).
//
//  On first use, lazily initialises the shared XpressNetHandler
//  instance for this module (sZ21XpressNet) and registers its
//  broadcast callback. Unlike LenzLan/LenzUsb (which buffer broadcasts
//  via XnInterfaceLayer to control ordering relative to the 01/04/05
//  acknowledgement), Z21 does not use that interface-local layer —
//  the callback here builds and sends the broadcast immediately via
//  _broadcast(), choosing the broadcast-flag mask based on the
//  triggering command: a turnout-info reply (0x43) is sent only to
//  clients subscribed to DRIVING_LOCO, while everything else (loco
//  status, etc.) is sent to clients subscribed to either
//  DRIVING_LOCO or SYSTEM_STATE.
//
//  Special case: if the payload is a MultiMaus-style extended loco
//  query (E3 F0), this is answered directly here rather than via the
//  shared handler, because it needs a Z21-specific EF-style response
//  format (extended loco info including F0-F28 and 128-step speed)
//  that differs from what the shared XpressNetHandler builds for
//  plain XpressNet clients. Looks up the requested loco in the
//  LocoRepository (falling back to a zeroed dummy entry if it is not
//  found), re-encodes its address, step mode, speed (converted to
//  the NEM speed range appropriate for its step mode, applying the
//  28-step G-bit shuffle where needed) and function bits into the
//  EF response layout, computes the XOR checksum, wraps it as an
//  XBus reply, and sends it directly to the requesting client.
//
//  For every other XBus payload, applies one further piece of
//  Z21-specific translation before forwarding to the shared handler:
//  LAN_X_SET_TURNOUT (X-header 0x53) uses a different address/data-
//  byte layout in the Z21 dialect than plain XpressNet's own 0x53
//  command (Z21: direct 1-based output address plus a DB2 byte with
//  activate at bit3 and direction at bit0; XpressNet: 0-based decoder
//  address + TT sub-output encoded in the data byte). If the payload
//  is such a Z21-style turnout command, it is remapped here into the
//  equivalent plain-XpressNet decoder+TT representation (mappedData)
//  before being passed on, so the shared handler only ever has to
//  understand the one, plain XpressNet format. Anything else is
//  passed through unchanged.
//
//  Passes the (possibly remapped) payload to the shared handler's
//  process() function. If the result is REPLY or ERROR with a
//  non-empty response, and the result is NOT the "silent" acknowledge
//  case, the reply is wrapped as an XBus reply and sent directly back
//  to the requesting client. If the command was specifically a
//  turnout command (0x52 or 0x53) and produced a silent
//  acknowledgement, a LAN_X_TURNOUT_INFO broadcast is built instead —
//  reconstructing the affected output address from the (already
//  remapped) payload — and sent to every client subscribed to
//  DRIVING_LOCO, reporting the turnout's currently known state.
// ─────────────────────────────────────────────────────────────
void Z21Lan::_handleXBus(Z21Client& c,
                           const uint8_t* data, uint8_t len) {
    if (len < 2) return;
    // Generic RX log — every LAN_X command reaching this read handler,
    // regardless of type, logged once here rather than piecemeal at
    // each individual command's own handling further down (Rob).
    traceLog().logBytes("Z21", "RX", data, len);
    if (!sZ21Init) {
        sZ21XpressNet.begin(ModuleId::Z21_LAN);
        // locoAddr != 0 marks this as a broadcast ABOUT a specific
        // loco (see XnBroadcastFn's comment in xpressnet_handler.h).
        // The shared XpressNetHandler builds `bdata` in the RS485-
        // native format (X-Header 0xE4, no address in the payload —
        // see XpressNetHandler::buildLocoInfo()'s own comment), which
        // genuine Z21 clients cannot parse (confirmed via TraceLog,
        // Rob: it showed up as an unrecognised "Z21 BC" line still
        // starting with 0xE4 instead of the expected 0xEF, right after
        // fixing the direct-reply path for the same 0xE4->0xEF issue
        // earlier). For loco broadcasts, rebuild a proper Z21-native
        // LAN_X_LOCO_INFO (X-Header 0xEF, address included) via
        // _buildLocoInfo() — the same builder already used for direct
        // GET_LOCO_INFO/SET_LOCO_DRIVE replies — using the now-
        // available locoAddr, instead of forwarding bdata unchanged.
        // Non-loco broadcasts (locoAddr==0: track power, e-stop,
        // turnout feedback, fast clock) are already transport-agnostic
        // and pass through as before.
        sZ21XpressNet.setBroadcastCallback([](const uint8_t* bdata, uint8_t dlen, uint16_t locoAddr) {
            uint8_t buf[Z21_UDP_BUF]; uint16_t rlen = 0;
            if (locoAddr != 0) {
                gZ21Lan._buildLocoInfo(buf, rlen, locoAddr);
                traceLog().logBytes("Z21", "BC", buf, rlen);
                gZ21Lan._broadcast(buf, rlen, Z21BcFlag::DRIVING_LOCO | Z21BcFlag::SYSTEM_STATE);
                return;
            }
            XnFrame xf;
            if (dlen > 0 && dlen <= (int)sizeof(xf.data)) {
                memcpy(xf.data, bdata, dlen);
                xf.len = dlen; xf.isBroadcast = true;
                gZ21Lan._buildXBusReply(buf, rlen, xf);
                // 0x43 = LAN_X_TURNOUT_INFO → DRIVING_LOCO flag (spec 2.16)
                // everything else → DRIVING_LOCO | SYSTEM_STATE
                uint32_t flag = (dlen >= 1 && bdata[0] == 0x43)
                                ? Z21BcFlag::DRIVING_LOCO
                                : (Z21BcFlag::DRIVING_LOCO | Z21BcFlag::SYSTEM_STATE);
                traceLog().logBytes("Z21", "BC", buf, rlen);
                gZ21Lan._broadcast(buf, rlen, flag);
            }
        });
        sZ21Init = true;
    }
    if (len >= 4 && data[0] == 0xE3 && data[1] == 0xF0) {
        uint16_t addr = XpressNetHandler::decodeAddr(data[2], data[3]);
        LocoBase loco{}; const LocoBase* found = LocoRepository::instance().find(addr);
        if (found) loco = *found;
        uint8_t ah, al; XpressNetHandler::encodeAddr(addr, ah, al);
        uint8_t steps = (loco.stepMode == 0) ? 0x00 :
                        (loco.stepMode == 2) ? 0x02 : 0x04;
        uint8_t nemSpeed = (loco.stepMode == 0) ? canonicalToNem(loco.speed, 0) :
                            (loco.stepMode == 2) ? xn28Shuffle(canonicalToNem(loco.speed, 2)) :
                            (loco.speed & 0x7F);
        uint8_t spd   = (loco.forward ? 0x80 : 0x00) | (nemSpeed & 0x7F);
        uint32_t fn   = loco.functions;
        uint8_t dirf  = 0;
        if (fn & 0x001) dirf |= 0x10;
        if (fn & 0x002) dirf |= 0x01;
        if (fn & 0x004) dirf |= 0x02;
        if (fn & 0x008) dirf |= 0x04;
        if (fn & 0x010) dirf |= 0x08;
        uint8_t f1 = (fn >>  5) & 0x0F;
        uint8_t f2 = (fn >>  9) & 0x0F;
        uint8_t f3 = (fn >> 13) & 0xFF;
        uint8_t f4 = (fn >> 21) & 0xFF;
        uint8_t xn[17];
        xn[0]=0xEF; xn[1]=ah;  xn[2]=al;  xn[3]=steps; xn[4]=spd;
        xn[5]=dirf; xn[6]=f1;  xn[7]=f2;  xn[8]=f3;    xn[9]=f4;
        xn[10]=0;   xn[11]=0;  xn[12]=0;  xn[13]=0;    xn[14]=0; xn[15]=0;
        xn[16] = XpressNetHandler::xorCheck(xn, 16);
        XnFrame f; memcpy(f.data, xn, 17); f.len=17; f.isBroadcast=false;
        uint16_t rlen=0; _buildXBusReply(_txBuf, rlen, f);
        _send(c, _txBuf, rlen);
        return;
    }
    uint8_t resp[24]; uint8_t respLen = 0;

    // Z21 LAN_X_SET_TURNOUT (X-header 0x53) uses a different address
    // format than plain XpressNet 0x53:
    //   Z21:      AdrMSB/AdrLSB = FAdr, direct 1-based output address
    //             DB2: 1 0 Q 0 A 0 0 P  (A=bit3 activate, P=bit0 direction —
    //             RCN-213: 0=thrown/diverging, 1=straight, Q=?)
    //   XpressNet: AdrMSB/AdrLSB = decoder address (0-based) + TT
    //             Dat: bit3=C(activate), bits2-1=TT, bit0=R
    // The mapping happens here so handleTurnoutZ21 only ever sees plain
    // XpressNet (FAdr is converted internally to decoder+TT for that shared
    // handler — the TT split is purely our own internal representation).
    uint8_t mappedData[6];
    const uint8_t* procData = data;
    uint8_t procLen = len;

    if (len >= 4 && data[0] == 0x53) {
        // Z21 LAN_X_SET_TURNOUT (X-header 0x53) uses a direct, 0-based
        // "FAdr" output address (data[1]/data[2], big-endian). Convert
        // to our own 1-based outAddr, then split into decoder/TT
        // exactly as decodeTurnoutAddr()/handleTurnout() (0x52)
        // expect: a genuine 4-byte packet — [0]=header, [1]=decoder
        // (a single uint8_t, 0-255 — an existing constraint already
        // inherent to the native 0x52 format itself, max output
        // address (255*4)+3+1=1024), [2]=data byte (1000-CTTR:
        // C=activate bit3, TT=bits2-1, R=direction bit0), [3]=XOR —
        // so the shared handler only ever has to understand the one,
        // plain XpressNet format.
        uint16_t FAdr    = ((uint16_t)data[1] << 8) | data[2];  // 0-based, per Z21 spec
        uint16_t outAddr = FAdr + 1;  // 1-based
        uint8_t activate = (data[3] >> 3) & 0x01;  // Z21: bit3=A(activate)
        uint8_t output   =  data[3]       & 0x01;  // Z21: bit0=P(direction)
        uint16_t out0    = outAddr - 1;
        uint16_t decoder = out0 / 4;
        uint8_t  TT      = out0 % 4;
        uint8_t dat = 0x80 | (activate << 3) | (TT << 1) | output;
        mappedData[0] = 0x53;
        mappedData[1] = (uint8_t)(decoder & 0xFF);
        mappedData[2] = dat;
        mappedData[3] = mappedData[0]^mappedData[1]^mappedData[2];
        procData = mappedData;
        procLen  = 4;
    }
    // Generic handoff log — whatever is about to be handed to the
    // shared XpressNetHandler, whether that's the original bytes
    // unchanged (most command types) or the just-built mappedData
    // (turnouts specifically, per the Z21-vs-plain-XpressNet address
    // format difference above). One log point covers every command
    // type reaching XpressNetHandler, rather than needing its own
    // line added per command as new ones are handled (Rob).
    traceLog().logBytes("Z21", "-> XN", procData, procLen);

    XNHandleResult r = sZ21XpressNet.process(procData, procLen, resp, respLen);
    // LAN_X_SET_LOCO_DRIVE (X-Header 0xE4, DB0=0x1S), LAN_X_SET_LOCO_
    // FUNCTION (DB0=0xF8), and LAN_X_SET_LOCO_FUNCTION_GROUP (DB0=a
    // group number) all share X-Header 0xE4. The shared
    // XpressNetHandler::process() call above already updated the loco
    // repository correctly, but the `resp` it produced is built via
    // XpressNetHandler::buildLocoInfo() — the RS485-native reply
    // (X-Header 0xE4), not the Z21-LAN-specific LAN_X_LOCO_INFO format
    // (X-Header 0xEF, different field layout — see Z21Lan::
    // _buildLocoInfo()'s comment for the full explanation). Forwarding
    // `resp` unchanged here produced a reply genuine Z21 clients
    // (Rocrail) could not parse, which is why driving and function
    // control appeared completely broken over Z21 while working fine
    // over RS485 (confirmed via packet capture, Rob). Build and send
    // a proper Z21-native reply instead, using the address from the
    // request (DB1/DB2, i.e. data[2]/data[3] — same offset as the
    // 0xE3/0xF0 GET_LOCO_INFO case above, since DB0 here is the
    // drive/function sub-command rather than part of the address).
    if (data[0] == 0xE4 && len >= 4) {
        uint16_t addr = XpressNetHandler::decodeAddr(data[2], data[3]);
        uint16_t rlen = 0;
        _buildLocoInfo(_txBuf, rlen, addr);
        traceLog().logBytes("Z21", "TX", _txBuf, rlen);
        _send(c, _txBuf, rlen);
        return;
    }
    if ((r == XNHandleResult::REPLY || r == XNHandleResult::ERROR ||
         r == XNHandleResult::OK_SILENT) && respLen > 0) {
        bool isOk = (r == XNHandleResult::OK_SILENT);
        bool isTurnout = (procLen >= 1 && (procData[0] == 0x52 || procData[0] == 0x53));
        if (!isOk) {
            XnFrame f; memcpy(f.data, resp, respLen);
            f.len = respLen; f.isBroadcast = false;
            uint16_t rlen = 0; _buildXBusReply(_txBuf, rlen, f);
            traceLog().logBytes("Z21", "TX", _txBuf, rlen);
            _send(c, _txBuf, rlen);
        } else if (isTurnout && procLen >= 4) {
            // Send LAN_X_TURNOUT_INFO back — address already correct in procData
            uint16_t decoder, TT_fb;
            if (procData[0] == 0x53) {
                decoder = ((uint16_t)procData[1] << 8) | procData[2];
                TT_fb   = (procData[3] >> 1) & 0x03;
            } else {
                decoder = procData[1] & 0x3F;
                TT_fb   = (procData[2] >> 1) & 0x03;
            }
            uint16_t addr = decoder * 4 + TT_fb + 1;
            uint8_t  msb  = addr >> 8;
            uint8_t  lsb  = addr & 0xFF;
            uint8_t  state = (addr < MAX_ACCESSORIES && gAccessories[addr].known)
                             ? (gAccessories[addr].state & 0x03) : 0x00;
            uint8_t xn[5];
            xn[0]=0x43; xn[1]=msb; xn[2]=lsb; xn[3]=state;
            xn[4] = XpressNetHandler::xorCheck(xn, 4);
            XnFrame f; memcpy(f.data, xn, 5); f.len=5; f.isBroadcast=true;
            uint16_t rlen=0; _buildXBusReply(_txBuf, rlen, f);
            traceLog().logBytes("Z21", "BC", _txBuf, rlen);
            _broadcast(_txBuf, rlen, Z21BcFlag::DRIVING_LOCO);
        }
    }
}

// ─────────────────────────────────────────────────────────────
// _handleGetLocoInfo() — LAN_X_GET_LOCO_INFO: decodes the requested
// loco address from the payload and replies with a LAN_X_LOCO_INFO
// packet built via _buildLocoInfo(), reflecting that loco's currently
// known speed, direction, step mode and functions.
// ─────────────────────────────────────────────────────────────
void Z21Lan::_handleGetLocoInfo(Z21Client& c, const uint8_t* data, uint8_t len) {
    if (len < 2) return;
    uint16_t addr = XpressNetHandler::decodeAddr(data[0], data[1]);
    uint16_t rlen = 0;
    _buildLocoInfo(_txBuf, rlen, addr);
    _send(c, _txBuf, rlen);
}

// ─────────────────────────────────────────────────────────────
// _handleLocoSpeed() — Z21 loco drive command: decodes the target
// loco address, speed (bits 6-0) and direction (bit7) from the
// payload, and dispatches a LOCO_SPEED command onto the command bus
// with a fixed stepMode of 4 (128-step, the only step mode this Z21
// command variant is used for). No direct reply is sent here — the
// resulting state change is reported back via the shared handler's
// broadcast mechanism once DccHal/LocoRepository have processed it.
// ─────────────────────────────────────────────────────────────
void Z21Lan::_handleLocoSpeed(Z21Client& c, const uint8_t* data, uint8_t len) {
    if (len < 3) return;
    Command cmd{};
    cmd.type     = CmdType::LOCO_SPEED;
    cmd.sourceId = ModuleId::Z21_LAN;
    cmd.address  = XpressNetHandler::decodeAddr(data[0], data[1]);
    cmd.speed    = data[2] & 0x7F;
    cmd.forward  = (data[2] & 0x80) != 0;
    cmd.stepMode = 4;
    CommandBus::instance().dispatch(cmd);
}

// ─────────────────────────────────────────────────────────────
// _handleLocoFunction() — Z21 loco function command: decodes the
// target loco address, function number and requested on/off state
// from the payload, and dispatches a LOCO_FUNCTION command onto the
// command bus.
// ─────────────────────────────────────────────────────────────
void Z21Lan::_handleLocoFunction(Z21Client& c, const uint8_t* data, uint8_t len) {
    if (len < 4) return;
    Command cmd{};
    cmd.type     = CmdType::LOCO_FUNCTION;
    cmd.sourceId = ModuleId::Z21_LAN;
    cmd.address  = XpressNetHandler::decodeAddr(data[0], data[1]);
    cmd.fn       = data[2];
    cmd.fnState  = (data[3] != 0);
    CommandBus::instance().dispatch(cmd);
}

// ─────────────────────────────────────────────────────────────
// _handleCvRead() — LAN_CV_READ: decodes the requested CV number
// (little-endian, 0-based on the wire, converted to the 1-based CV
// numbering used internally by adding 1) and dispatches a CV_READ
// command (service-mode programming) onto the command bus.
// ─────────────────────────────────────────────────────────────
void Z21Lan::_handleCvRead(Z21Client& c, const uint8_t* data, uint8_t len) {
    if (len < 2) return;
    Command cmd{};
    cmd.type = CmdType::CV_READ; cmd.sourceId = ModuleId::Z21_LAN;
    cmd.cv   = _u16le(data) + 1;
    CommandBus::instance().dispatch(cmd);
}

// ─────────────────────────────────────────────────────────────
// _handleCvWrite() — LAN_CV_WRITE: decodes the requested CV number
// and value to write, and dispatches a CV_WRITE command (service-mode
// programming) onto the command bus.
// ─────────────────────────────────────────────────────────────
void Z21Lan::_handleCvWrite(Z21Client& c, const uint8_t* data, uint8_t len) {
    if (len < 3) return;
    Command cmd{};
    cmd.type    = CmdType::CV_WRITE; cmd.sourceId = ModuleId::Z21_LAN;
    cmd.cv      = _u16le(data) + 1;
    cmd.cvValue = data[2];
    CommandBus::instance().dispatch(cmd);
}

// ─────────────────────────────────────────────────────────────
//  LocoNet dispatch
// ─────────────────────────────────────────────────────────────

// ─────────────────────────────────────────────────────────────
// _handleLocoNetDispatch() — LAN_LOCONET_DISPATCH_ADDR: decodes the
// requested LocoNet address (big-endian 16-bit) and dispatches an
// LN_DISPATCH command onto the command bus, requesting that this
// address be dispatched for use on the LocoNet side.
// ─────────────────────────────────────────────────────────────
void Z21Lan::_handleLocoNetDispatch(Z21Client& c, const uint8_t* data, uint8_t len) {
    if (len < 2) return;
    uint16_t addr = ((uint16_t)data[0] << 8) | data[1];
    Command cmd{};
    cmd.type      = CmdType::LN_DISPATCH;
    cmd.sourceId  = ModuleId::Z21_LAN;
    cmd.address   = addr;
    CommandBus::instance().dispatch(cmd);
}

// ─────────────────────────────────────────────────────────────
// _handleRmbusGetData() — LAN_RMBUS_GETDATA: decodes the requested
// group number (each group covers 10 consecutive RS-Bus module
// addresses) and replies with a LAN_RMBUS_DATACHANGED packet
// containing the current nibble values of every module address in
// that group (both nibbles packed into one byte per address; groups
// beyond the valid 1-128 address range are left as zero).
// ─────────────────────────────────────────────────────────────
void Z21Lan::_handleRmbusGetData(Z21Client& c, const uint8_t* data, uint8_t len) {
    if (len < 1) return;
    uint8_t group = data[0];
    uint8_t buf[15];
    _putU16le(buf,   15);
    _putU16le(buf+2, Z21Hdr::LAN_RMBUS_DATACHANGED);
    buf[4] = group;
    memset(buf+5, 0, 10);
    uint8_t baseAddr = group * 10 + 1;
    for (uint8_t i = 0; i < 10; i++) {
        uint8_t addr = baseAddr + i;
        if (addr <= 128) {
            uint8_t n0 = gRsBusHal.getNibble(addr, 0);
            uint8_t n1 = gRsBusHal.getNibble(addr, 1);
            buf[5 + i] = (n1 << 4) | (n0 & 0x0F);
        }
    }
    traceLog().logBytes("Z21", "TX", buf, 15);
    _send(c, buf, 15);
}

// ─────────────────────────────────────────────────────────────
//  Fast Clock — GET
// ─────────────────────────────────────────────────────────────

// ─────────────────────────────────────────────────────────────
//  _handleFastClockGet() — build and send the current fast-clock
//  time/state (spec chapter 12.2)
//
//  Frame: 0C 00 CC 00 + FastClockTime (8 bytes)
//
//  FastClockTime layout:
//    [0] = 0x66  (fixed magic byte)
//    [1] = 0x25  (fixed magic byte)
//    [2] = DDDh hhhh  (bits7-5 = day, bits4-0 = hour)
//    [3] = 00mm mmmm  (bits5-0 = minute)
//    [4] = SHss ssss  (S=STOP flag, H=HALT flag, bits5-0 = seconds)
//    [5] = 00rr rrrr  (bits5-0 = rate/acceleration factor)
//    [6] = FcSettings (persistent settings flags)
//    [7] = XOR over bytes [0]..[6]
//
//  Reads the current day-of-week/hour/minute/factor from gCentrale,
//  sets the STOP flag if the clock is not running or its factor is
//  zero, sets the HALT flag if an emergency stop is currently active
//  (a temporary pause distinct from STOP), always reports seconds as
//  0 (seconds are not tracked), uses a fixed factory-default settings
//  byte, computes the XOR checksum, and sends the resulting 12-byte
//  packet to the requesting client. Ignores its `data`/`len`
//  parameters entirely — this same function is also called (with
//  data=nullptr, len=0) as the common "send back the current time" tail
//  end of every fast-clock control sub-command in _handleFastClockSet().
// ─────────────────────────────────────────────────────────────
void Z21Lan::_handleFastClockGet(Z21Client& c, const uint8_t* data, uint8_t len) {
    uint8_t dow    = gCentrale.mtDow    & 0x07;
    uint8_t hour   = gCentrale.mtHour   & 0x1F;
    uint8_t minute = gCentrale.mtMinute & 0x3F;
    uint8_t factor = gCentrale.mtFactor & 0x3F;

    // STOP flag: clock is not running (factor=0 or mtRunning=false)
    uint8_t stopFlag = (!gCentrale.mtRunning || factor == 0) ? 0x80 : 0x00;
    // HALT flag: temporarily paused due to emergency stop
    uint8_t haltFlag = gCentrale.emergencyStop ? 0x40 : 0x00;

    uint8_t d0 = 0x66;
    uint8_t d1 = 0x25;
    uint8_t d2 = ((dow & 0x07) << 5) | (hour & 0x1F);
    uint8_t d3 = minute & 0x3F;
    uint8_t d4 = stopFlag | haltFlag;   // seconds = 0
    uint8_t d5 = factor;
    uint8_t d6 = 0x4F;                  // FcSettings factory default
    uint8_t d7 = d0 ^ d1 ^ d2 ^ d3 ^ d4 ^ d5 ^ d6;

    uint8_t buf[12];
    _putU16le(buf,   12);
    _putU16le(buf+2, Z21Hdr::LAN_FAST_CLOCK_DATA);
    buf[4]  = d0;
    buf[5]  = d1;
    buf[6]  = d2;
    buf[7]  = d3;
    buf[8]  = d4;
    buf[9]  = d5;
    buf[10] = d6;
    buf[11] = d7;
    _send(c, buf, 12);
}

// ─────────────────────────────────────────────────────────────
//  Fast Clock — central dispatcher (GET / SET / START / STOP)
// ─────────────────────────────────────────────────────────────

// ─────────────────────────────────────────────────────────────
//  _handleFastClockSet() — LAN_FAST_CLOCK_CONTROL (header CC 00),
//  spec chapter 12.1
//
//  Distinguishes between four sub-commands based on their first two
//  payload bytes:
//    GET:   data[0]=0x21 data[1]=0x2A  → send current clock status
//    SET:   data[0]=0x24 data[1]=0x2B  → set the clock
//    START: data[0]=0x21 data[1]=0x2C  → resume the clock
//    STOP:  data[0]=0x21 data[1]=0x2D  → pause the clock
//
//  If fewer than 2 payload bytes are present, falls back to just
//  reporting the current state (GET behaviour).
//
//  GET simply calls _handleFastClockGet() to report the current state
//  unchanged.
//
//  START sets mtRunning=true, and if the current rate factor is 0,
//  bumps it to 1 first (starting a clock that has a zero rate would
//  otherwise silently do nothing), then reports the resulting state.
//
//  STOP sets mtRunning=false and reports the resulting state.
//
//  SET expects at least 3 more payload bytes: a packed day/hour byte
//  (data[2]), a packed minute byte (data[3]), and a packed rate byte
//  (data[4]). If a 6th byte is present, it is validated as the XOR
//  checksum of the preceding 5 bytes, and the whole command is
//  ignored if that checksum does not match. On success, updates
//  gCentrale's day-of-week/hour/minute/factor fields, sets mtRunning
//  based on whether the new factor is non-zero, and reports the
//  resulting state back to the client.
// ─────────────────────────────────────────────────────────────
void Z21Lan::_handleFastClockSet(Z21Client& c, const uint8_t* data, uint8_t len) {
    if (len < 2) { _handleFastClockGet(c, nullptr, 0); return; }

    if (data[0] == 0x21 && data[1] == 0x2A) {
        // GET
        _handleFastClockGet(c, nullptr, 0);
        return;
    }

    if (data[0] == 0x21 && data[1] == 0x2C) {
        // START
        gCentrale.mtRunning = true;
        if (gCentrale.mtFactor == 0) gCentrale.mtFactor = 1;
        _handleFastClockGet(c, nullptr, 0);
        return;
    }

    if (data[0] == 0x21 && data[1] == 0x2D) {
        // STOP
        gCentrale.mtRunning = false;
        _handleFastClockGet(c, nullptr, 0);
        return;
    }

    if (data[0] == 0x24 && data[1] == 0x2B) {
        // SET
        // data[2] = DDDhhhhh  (bits7-5=day, bits4-0=hour)
        // data[3] = 00mmmmmm  (bits5-0=minute)
        // data[4] = 00rrrrrr  (bits5-0=rate)
        // data[5] = XOR checksum
        if (len < 5) return;

        // Validate the XOR checksum if present
        if (len >= 6) {
            uint8_t xor_calc = data[0] ^ data[1] ^ data[2] ^ data[3] ^ data[4];
            if (xor_calc != data[5]) return;
        }

        gCentrale.mtDow     = (data[2] >> 5) & 0x07;
        gCentrale.mtHour    = data[2] & 0x1F;
        gCentrale.mtMinute  = data[3] & 0x3F;
        gCentrale.mtFactor  = data[4] & 0x3F;
        gCentrale.mtRunning = (gCentrale.mtFactor > 0);

        _handleFastClockGet(c, nullptr, 0);
        return;
    }
}

// ─────────────────────────────────────────────────────────────
//  _handleFastClockSettingsGet() — LAN_FAST_CLOCK_SETTINGS_GET
//  response, spec chapter 12.3
//
//  Frame: 08 00 CE 00 FcSettings Rate StartDDDhhhhh StartMMMMMM
//
//  FcSettings bits:
//    bit0 = fcFastClockLocoNetEn
//    bit1 = fcFastClockXBUSEn
//    bit3 = fcFastClockDCCEn
//    bit4 = fcFastClockMRclockEn
//    bit6 = fcFastClockEmergencyHaltEn
//    bit7 = fcFastClockEnabled
//
//  Starts from a fixed factory-default settings byte, clearing the
//  "Enabled" bit if the clock is not currently running, and reports
//  the current rate factor and start day/hour/minute (which here
//  double as the persistent "settings" values, since this station
//  does not separately track a distinct start time from the live
//  clock state).
// ─────────────────────────────────────────────────────────────
void Z21Lan::_handleFastClockSettingsGet(Z21Client& c) {
    uint8_t fcSettings = 0x4F;  // factory default
    if (!gCentrale.mtRunning) fcSettings &= ~0x80;  // clear Enabled if stopped
    uint8_t rate          = gCentrale.mtFactor & 0x3F;
    uint8_t startDDDhhhhh = ((gCentrale.mtDow  & 0x07) << 5)
                           | (gCentrale.mtHour  & 0x1F);
    uint8_t startMMMMMM   =  gCentrale.mtMinute & 0x3F;
    uint8_t buf[8];
    _putU16le(buf,   8);
    _putU16le(buf+2, Z21Hdr::LAN_FAST_CLOCK_SETTINGS);
    buf[4] = fcSettings;
    buf[5] = rate;
    buf[6] = startDDDhhhhh;
    buf[7] = startMMMMMM;
    _send(c, buf, 8);
}

// ─────────────────────────────────────────────────────────────
//  _handleFastClockSettingsSet() — LAN_FAST_CLOCK_SETTINGS_SET,
//  spec chapter 12.4
//
//  Payload: FcSettings Rate StartDDDhhhhh StartMMMMMM (4 bytes). If
//  fewer than 4 bytes are supplied, simply reports the current
//  settings unchanged (via _handleFastClockSettingsGet()) instead of
//  applying anything. Otherwise applies the new settings: mtRunning
//  follows the "Enabled" bit, mtFactor takes the new rate, and
//  mtDow/mtHour/mtMinute take the new start day/hour/minute — then
//  reports the resulting settings back to the client as confirmation.
// ─────────────────────────────────────────────────────────────
void Z21Lan::_handleFastClockSettingsSet(Z21Client& c, const uint8_t* data, uint8_t len) {
    if (len < 4) { _handleFastClockSettingsGet(c); return; }
    uint8_t fcSettings    = data[0];
    uint8_t rate          = data[1] & 0x3F;
    uint8_t startDDDhhhhh = data[2];
    uint8_t startMMMMMM   = data[3] & 0x3F;
    gCentrale.mtRunning = (fcSettings & 0x80) != 0;
    gCentrale.mtFactor  = rate;
    gCentrale.mtDow     = (startDDDhhhhh >> 5) & 0x07;
    gCentrale.mtHour    = startDDDhhhhh & 0x1F;
    gCentrale.mtMinute  = startMMMMMM;
    _handleFastClockSettingsGet(c);  // send confirmation back
}

// ─────────────────────────────────────────────────────────────
// _handleSystemStateGet() — LAN_SYSTEMSTATE_GETDATA: builds and sends
// a full 20-byte system state packet directly (rather than via
// _buildSystemState(), which only sets the length/header and zeroes
// the payload). Sets the CentralState byte's emergency-stop and
// track-power-off bits according to the current gCentrale flags; all
// other fields in the (mostly unused-by-this-station) system state
// payload are left at zero.
// ─────────────────────────────────────────────────────────────
void Z21Lan::_handleSystemStateGet(Z21Client& c) {
    uint8_t buf[20];
    _putU16le(buf,   20);
    _putU16le(buf+2, Z21Hdr::LAN_SYSTEMSTATE_DATACHANGED);
    memset(buf+4, 0, 16);
    uint8_t state = 0;
    if (gCentrale.emergencyStop) state |= 0x01;
    if (gCentrale.trackPowerOff) state |= 0x02;
    buf[16] = state;
    buf[17] = 0x00;
    buf[18] = 0x00;
    buf[19] = 0x00;
    _send(c, buf, 20);
}
// ─────────────────────────────────────────────────────────────
// _handleGetBcFlags() — LAN_GET_BROADCASTFLAGS: replies with this
// client's currently stored 32-bit broadcast-flag bitmask, encoded
// little-endian across 4 bytes.
// ─────────────────────────────────────────────────────────────
void Z21Lan::_handleGetBcFlags(Z21Client& c) {
    uint8_t buf[8];
    _putU16le(buf,   8);
    _putU16le(buf+2, Z21Hdr::LAN_GET_BROADCASTFLAGS);
    buf[4] = (c.bcFlags      ) & 0xFF;
    buf[5] = (c.bcFlags >>  8) & 0xFF;
    buf[6] = (c.bcFlags >> 16) & 0xFF;
    buf[7] = (c.bcFlags >> 24) & 0xFF;
    _send(c, buf, 8);
}

// ─────────────────────────────────────────────────────────────
// _handleGetLocoMode() — LAN_GET_LOCOMODE: echoes back the requested
// loco address together with a fixed "DCC" mode byte (0x00) — this
// station does not implement Motorola-format loco addressing, so the
// mode is always reported as DCC regardless of the requested address.
// ─────────────────────────────────────────────────────────────
void Z21Lan::_handleGetLocoMode(Z21Client& c, const uint8_t* data, uint8_t len) {
    if (len < 2) return;
    uint8_t buf[7];
    _putU16le(buf, 7);
    _putU16le(buf+2, Z21Hdr::LAN_GET_LOCOMODE);
    buf[4] = data[0];
    buf[5] = data[1];
    buf[6] = 0x00;
    _send(c, buf, 7);
}

// ─────────────────────────────────────────────────────────────
// _handleSetTurnoutMode() — LAN_SET_TURNOUTMODE: decodes the target
// turnout address and stores the requested mode bit (0=DCC, 1=MM) in
// that address's AccessoryState entry, if the address is within
// range. Per spec, no reply is sent for this command.
// ─────────────────────────────────────────────────────────────
void Z21Lan::_handleSetTurnoutMode(Z21Client& c, const uint8_t* data, uint8_t len) {
    // LAN_SET_TURNOUTMODE — no response per spec
    if (len < 3) return;
    uint16_t addr = ((uint16_t)data[0] << 8) | data[1];
    if (addr < MAX_ACCESSORIES) {
        gAccessories[addr].mode = data[2] & 0x01;  // 0=DCC, 1=MM
    }
}

// ─────────────────────────────────────────────────────────────
// _handleGetTurnoutMode() — LAN_GET_TURNOUTMODE: decodes the
// requested turnout address, looks up its stored mode bit (defaulting
// to 0/DCC if the address is out of range), and echoes the address
// back together with that mode byte.
// ─────────────────────────────────────────────────────────────
void Z21Lan::_handleGetTurnoutMode(Z21Client& c, const uint8_t* data, uint8_t len) {
    if (len < 2) return;
    uint16_t addr = ((uint16_t)data[0] << 8) | data[1];
    uint8_t mode = (addr < MAX_ACCESSORIES) ? gAccessories[addr].mode : 0;
    uint8_t buf[7];
    _putU16le(buf, 7);
    _putU16le(buf+2, Z21Hdr::LAN_GET_TURNOUTMODE);
    buf[4] = data[0];
    buf[5] = data[1];
    buf[6] = mode;
    _send(c, buf, 7);
}

// ─────────────────────────────────────────────────────────────
//  onEvent() — translate hardware events into Z21 broadcasts
//
//  POWER_ON / POWER_OFF: builds a full LAN_SYSTEMSTATE_DATACHANGED
//  packet reflecting the current emergency-stop/track-power-off
//  flags, and sends it directly to every active client that has the
//  SYSTEM_STATE broadcast flag set (bypassing _broadcast() here since
//  the payload is built inline rather than via a separate builder
//  call).
//
//  FEEDBACK: translates a single RS-Bus module/nibble update into the
//  corresponding LAN_RMBUS_DATACHANGED group packet — computing which
//  group and byte position the module falls into, and packing the new
//  nibble value into the correct half of that byte (low nibble for
//  nibble 0, high nibble for nibble 1) within an otherwise all-zero
//  10-byte data block. Note that this only reports the single changed
//  nibble's own byte accurately; the other 9 bytes in the group are
//  sent as zero rather than their actual current values. Sent to
//  every currently active client, regardless of broadcast flags.
// ─────────────────────────────────────────────────────────────
void Z21Lan::onEvent(const Event& ev) {
    // Power events — send LAN_SYSTEMSTATE_DATACHANGED to all clients
    // with the SYSTEM_STATE broadcast flag set
    if (ev.type == EvType::POWER_ON || ev.type == EvType::POWER_OFF) {
        uint8_t buf[20];
        _putU16le(buf,   20);
        _putU16le(buf+2, Z21Hdr::LAN_SYSTEMSTATE_DATACHANGED);
        memset(buf+4, 0, 16);
        uint8_t state = 0;
        if (gCentrale.emergencyStop) state |= 0x01;
        if (gCentrale.trackPowerOff) state |= 0x02;
        buf[16] = state;
        for (uint8_t i = 0; i < Z21_MAX_CLIENTS; i++) {
            if (_clients[i].active &&
                (_clients[i].bcFlags & Z21BcFlag::SYSTEM_STATE))
                _send(_clients[i], buf, 20);
        }
        return;
    }
    if (ev.type == EvType::FEEDBACK) {
        uint8_t addr    = ev.fbModule;
        uint8_t group   = (addr - 1) / 10;
        uint8_t pos     = (addr - 1) % 10;
        uint8_t nibData = ev.fbDat & 0x0F;

        uint8_t payload[14] = {};
        _putU16le(payload,   14);
        _putU16le(payload+2, Z21Hdr::LAN_RMBUS_DATACHANGED);
        payload[4] = group;
        uint8_t bytePos = pos / 2;
        if (ev.fbNibble == 0)
            payload[5 + bytePos] = (payload[5 + bytePos] & 0xF0) | nibData;
        else
            payload[5 + bytePos] = (payload[5 + bytePos] & 0x0F) | (nibData << 4);

        // This broadcast arrives via EventBus (core0's RsBusHal
        // publishing a FEEDBACK event), not via the RX/-> XN read-
        // handler path logged in _handleXBus() — so it needed its own
        // log point here, at the moment the resulting Z21 packet is
        // actually built and sent (Rob: RS-Bus broadcasts weren't
        // visible in TraceLog despite the rest of Z21 traffic being
        // covered).
        traceLog().logBytes("Z21", "BC", payload, 14);
        for (uint8_t i = 0; i < Z21_MAX_CLIENTS; i++) {
            if (_clients[i].active)
                _send(_clients[i], payload, 14);
        }
    }
}

// ─────────────────────────────────────────────────────────────
//  Frame builders
// ─────────────────────────────────────────────────────────────

// ─────────────────────────────────────────────────────────────
// _buildSystemState() — fills buf with the header/length of a
// LAN_SYSTEMSTATE_DATACHANGED packet and zeroes its 16-byte payload.
// Note that this does NOT itself set the emergency-stop/track-power
// bits — callers that need those set (e.g. _handleSystemStateGet(),
// onEvent()) currently set buf[16] themselves after calling this, or
// build the packet inline instead of using this helper at all.
// ─────────────────────────────────────────────────────────────
void Z21Lan::_buildSystemState(uint8_t* buf, uint16_t& len) {
    len = 20;
    _putU16le(buf,   len);
    _putU16le(buf+2, Z21Hdr::LAN_SYSTEMSTATE_DATACHANGED);
    memset(buf+4, 0, 16);
}

// ─────────────────────────────────────────────────────────────
// _buildLocoInfo() — looks up the given loco address in the
// LocoRepository (falling back to a dummy entry with forward=true,
// stepMode=4, and the requested address if it is not found), encodes
// its address, step mode, speed+direction and function bits, and
// wraps the result as a Z21 LAN_X reply via _buildXBusReply().
//
// IMPORTANT: the X-Header used here (0xEF) is the Z21-LAN-specific
// LAN_X_LOCO_INFO value — it is NOT the same as native RS485
// XpressNet's own loco-info reply header (0xE4, used by
// XpressNetHandler::buildLocoInfo(), a separate function for the
// RS485 transport). Z21's own protocol spec (section 4.4
// LAN_X_LOCO_INFO) is explicit that this reply uses X-Header 0xEF,
// despite X-Bus tunneling otherwise reusing genuine XpressNet
// command/reply bytes elsewhere. Field order per spec: DB0=Adr_MSB,
// DB1=Adr_LSB, DB2=steps byte (0000BKKK), DB3=speed/direction
// (RVVVVVVV), DB4=F0-F4, DB5=F5-F12 — an earlier version of this
// function had these shifted by one position AND used the wrong
// header, confirmed via packet capture (Rob) to make every reply
// unparseable by genuine Z21 clients (Rocrail, the official Z21 test
// tool), which is why Z21 driving/functions appeared completely
// broken while RS485 XpressNet (a genuinely different, correct
// codepath) worked fine.
// ─────────────────────────────────────────────────────────────
void Z21Lan::_buildLocoInfo(uint8_t* buf, uint16_t& len, uint16_t addr) {
    const LocoBase* loco = locoRepo().find(addr);
    LocoBase dummy{}; dummy.address = addr; dummy.forward = true; dummy.stepMode = 4;
    const LocoBase& l = loco ? *loco : dummy;
    XnFrame xf;
    uint8_t ah, al; XpressNetHandler::encodeAddr(l.address, ah, al);
    uint8_t spdDir = (l.forward ? 0x80 : 0x00) | (l.speed & 0x7F);
    uint32_t fn    = l.functions;
    xf.data[0]=0xEF; xf.data[1]=ah;      xf.data[2]=al;
    xf.data[3]=l.stepMode&0x07;
    xf.data[4]=spdDir;
    xf.data[5]=((fn&1)<<4)|((fn>>1)&0x0F);
    xf.data[6]=(fn>>5)&0xFF;
    uint8_t xv=0; for(int i=0;i<7;i++) xv^=xf.data[i];
    xf.data[7]=xv; xf.len=8; xf.isBroadcast=true;
    _buildXBusReply(buf, len, xf);
}

// ─────────────────────────────────────────────────────────────
// _buildXBusReply() — wraps a raw XpressNet frame (header+data+XOR,
// as produced/consumed by the shared XpressNetHandler) inside the
// standard Z21 4-byte length+header envelope, using the LAN_X_BUS
// header value. This is the common final step used by every code
// path that needs to send an XpressNet-shaped response or broadcast
// over the Z21 UDP transport.
// ─────────────────────────────────────────────────────────────
void Z21Lan::_buildXBusReply(uint8_t* buf, uint16_t& len, const XnFrame& f) {
    len = 4 + f.len;
    _putU16le(buf,   len);
    _putU16le(buf+2, Z21Hdr::LAN_X_BUS);
    memcpy(buf+4, f.data, f.len);
}

// ─────────────────────────────────────────────────────────────
//  Sending
// ─────────────────────────────────────────────────────────────

// ─────────────────────────────────────────────────────────────
// _send() — sends one UDP packet containing exactly the given bytes
// to a single client's recorded IP address and port.
// ─────────────────────────────────────────────────────────────
void Z21Lan::_send(Z21Client& c, const uint8_t* buf, uint16_t len) {
    _udp.beginPacket(c.ip, c.port);
    _udp.write(buf, len);
    _udp.endPacket();
}

// ─────────────────────────────────────────────────────────────
// _broadcast() — sends the given bytes to every currently active
// client whose broadcast-flag bitmask has at least one bit in common
// with flagMask (i.e. the client has subscribed to at least one of
// the relevant event categories).
// ─────────────────────────────────────────────────────────────
void Z21Lan::_broadcast(const uint8_t* buf, uint16_t len, uint32_t flagMask) {
    for (uint8_t i = 0; i < Z21_MAX_CLIENTS; i++) {
        if (_clients[i].active && (_clients[i].bcFlags & flagMask))
            _send(_clients[i], buf, len);
    }
}

// ─────────────────────────────────────────────────────────────
//  Client management
// ─────────────────────────────────────────────────────────────

// ─────────────────────────────────────────────────────────────
// _findOrAddClient() — first scans all client slots for one that is
// already active and matches the given IP+port exactly, returning it
// if found. Otherwise looks for the first inactive (free) slot,
// initialises it for this new IP+port (defaulting its broadcast flags
// to SYSTEM_STATE only, so a client that never explicitly sets its
// own flags still receives power-state broadcasts), marks it active,
// increments the connected-client counter, and returns it. Returns
// nullptr if every slot is already in use (Z21_MAX_CLIENTS reached).
// ─────────────────────────────────────────────────────────────
Z21Client* Z21Lan::_findOrAddClient(const IPAddress& ip, uint16_t port) {
    for (uint8_t i = 0; i < Z21_MAX_CLIENTS; i++) {
        if (_clients[i].active &&
            _clients[i].ip == ip && _clients[i].port == port)
            return &_clients[i];
    }
    for (uint8_t i = 0; i < Z21_MAX_CLIENTS; i++) {
        if (!_clients[i].active) {
            _clients[i].ip         = ip;
            _clients[i].port       = port;
            _clients[i].bcFlags    = Z21BcFlag::SYSTEM_STATE;
            _clients[i].lastSeenMs = millis();
            _clients[i].active     = true;
            _clientCount++;
            return &_clients[i];
        }
    }
    return nullptr;
}

// ─────────────────────────────────────────────────────────────
// _checkTimeouts() — scans every client slot; for each one that is
// active and has not been seen (no packet received) for longer than
// the configured timeout, deactivates the slot, clears its
// broadcast flags, and decrements the connected-client counter,
// freeing the slot for reuse. Reads the timeout live from EEPROM
// ("z21.client_timeout_s", see z21_lan.h) each call, so a change made
// via the web interface takes effect immediately, without needing a
// reboot — matching how other web-configurable settings in this
// project are read (e.g. DccCurrentMonitor::thresholdMv()).
// ─────────────────────────────────────────────────────────────
void Z21Lan::_checkTimeouts() {
    uint32_t timeoutMs = (uint32_t)eepromStore().getUint16(
        "z21.client_timeout_s", Z21_CLIENT_TIMEOUT_DEFAULT_MS / 1000) * 1000;
    uint32_t now = millis();
    for (uint8_t i = 0; i < Z21_MAX_CLIENTS; i++) {
        if (_clients[i].active &&
            now - _clients[i].lastSeenMs > timeoutMs) {
            _clients[i].active = false;
            _clients[i].bcFlags = 0;
            _clientCount--;
        }
    }
}
