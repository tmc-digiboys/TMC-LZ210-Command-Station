// ─────────────────────────────────────────────────────────────
//
// Module: Xpressnet_Handler
// Description:
//      The Xpressnet_Handler implements the protocol handling of xpressnet.
//      It Handles messages received form one of the interface handlers
//      LenzLan, LenzUSB, LenzRS485 and Z21. The interface modules handels all interface related traffic.
//      The xpressnet_handler implements the xpressnet4.0 protocol specification.
//      In the process function it detemines wich command is recieved and calls the appropriate
//      commandhandelr to process the message en transfers the commands throughout the system and sends a reply
//      if necessary.
//  ─────────────────────────────────────────────────────────────
//
#include "xpressnet_handler.h"
#include "eeprom_store.h"
#include "DCCPacket.h"
#include "rs_bus_hal.h"
#include "hardware/watchdog.h"
#include "pico/bootrom.h"

CentraleState   gCentrale;
AccessoryState  gAccessories[MAX_ACCESSORIES];

// ─────────────────────────────────────────────────────────────
//  begin() — initialise this handler instance for one interface
//            (LAN, USB or RS485). Stores the module id used as
//            the "source" on dispatched commands, so the command
//            bus can avoid echoing a command back to the module
//            that originated it.
// ─────────────────────────────────────────────────────────────
void XpressNetHandler::begin(uint8_t srcId) {
    _srcId = srcId;
}

// ─────────────────────────────────────────────────────────────
//  Address helpers
// ─────────────────────────────────────────────────────────────

// ─────────────────────────────────────────────────────────────
// decodeAddr - decodes address as 16 bit integer
// ─────────────────────────────────────────────────────────────
uint16_t XpressNetHandler::decodeAddr(uint8_t ah, uint8_t al) {
    if ((ah & 0xC0) == 0xC0) return ((uint16_t)(ah & 0x3F) << 8) | al;
    return al;
}

// ─────────────────────────────────────────────────────────────
//  decodeTurnoutAddr — central conversion for turnout address
//
//  Converts xpressnet 0x52 decoder+TT to output address (1-based).
//  All three XpressNet protocols (LAN, USB, RS485) use the 0x52 format:
//    b1 = decoder address (bits 5-0, bit7 optional)
//    b2 = 1 ~MSB C TT1 TT0 R
//    outputAddress = decoder * 4 + TT + 1
//
//  Z21 LAN_X_SET_TURNOUT (0x53) uses a different format:
//    in[1..2] = big-endian outputAddress direct (no conversion needed)
//
//  Definition outputAddress: 1-based, 1-2048
//  Definition output: 0=straight, 1=thrown
// ─────────────────────────────────────────────────────────────
uint16_t XpressNetHandler::decodeTurnoutAddr(uint8_t b1, uint8_t b2) {
    // XpressNet 0x52 -> logical outputAddress (1-based, 1..1024)
    // The software (XpressnetLanClient) uses b1 as a full 8-bit decoder address:
    //   b1 = (addr-1) >> 2  (decoder, 0-based)
    //   b2 bits2-1 = TT (output within the decoder)
    // Inverse: outputAddress = b1 * 4 + TT + 1
    // DCC format conversion (TrntFormat, linear/non-linear) happens in getBitstream.
    uint8_t TT = (b2 >> 1) & 0x03;
    return (uint16_t)b1 * 4 + TT + 1;
}

// ─────────────────────────────────────────────────────────────
// encodeAddr - support function to encode Address
// ─────────────────────────────────────────────────────────────
void XpressNetHandler::encodeAddr(uint16_t addr, uint8_t& ah, uint8_t& al) {
    if (addr >= 100) {
        ah = 0xC0 | ((addr >> 8) & 0x3F);
        al = addr & 0xFF;
    } else {
        ah = 0x00; al = addr & 0xFF;
    }
}
// ─────────────────────────────────────────────────────────────
// xorCheck - calculates the xor value for the recievd buffer
// ─────────────────────────────────────────────────────────────
uint8_t XpressNetHandler::xorCheck(const uint8_t* buf, uint8_t len) {
    uint8_t x = 0;
    for (uint8_t i = 0; i < len; i++) x ^= buf[i];
    return x;
}

// ─────────────────────────────────────────────────────────────
// Verify xor - value returns a true or false depending of received
//              xor value is the correct xor value
// ─────────────────────────────────────────────────────────────
bool XpressNetHandler::verifyXor(const uint8_t* buf, uint8_t len) {
    if (len < 2) return false;
    return xorCheck(buf, len - 1) == buf[len - 1];
}

// ─────────────────────────────────────────────────────────────
//  Lok helpers — via LocoRepository
//                Updates the fields in the loc reposirory and
//                send a command to the dispatcher for handling
//                in the system
// ─────────────────────────────────────────────────────────────
void XpressNetHandler::updateLoco(uint16_t addr, uint8_t speed, uint8_t dir,
                                    uint32_t fn, SpeedStepMode mode) {
    if (locoRepo().lockWrite(500)) {
        LocoBase* loco = locoRepo().findOrCreate(addr);
        if (loco) {
            loco->speed     = speed;
            loco->forward   = (dir != 0);
            loco->functions = fn;
            loco->stepMode  = static_cast<uint8_t>(mode);
            locoRepo().markDirtySpeed(addr);
        }
        locoRepo().unlock();
    }
    Command cmd{};
    cmd.type     = CmdType::LOCO_SPEED;
    cmd.sourceId = _srcId;
    cmd.address  = addr;
    cmd.speed    = speed;
    cmd.forward  = (dir != 0);
    cmd.stepMode = static_cast<uint8_t>(mode);
    CommandBus::instance().dispatch(cmd);
}

// ─────────────────────────────────────────────────────────────
// updateLocoFunctions - Updates the loco functions in the loco repository
//                       and indicates that new functions are received
// ─────────────────────────────────────────────────────────────
void XpressNetHandler::updateLocoFunctions(uint16_t addr, uint32_t fn,
                                             const uint8_t* funcExt5) {
    LocoBase* loco = locoRepo().findOrCreate(addr);
    if (loco) {
        loco->functions = fn;
        if (funcExt5)
            memcpy(loco->functionsExt, funcExt5, 5);
        locoRepo().markDirtyFn(addr);  // only functions dirty, not speed
    }
}

// ─────────────────────────────────────────────────────────────
//  queryloco - Queries the locrepository if a loc is  present
//              otherwise it creates a default entry
// ─────────────────────────────────────────────────────────────
bool XpressNetHandler::queryLoco(uint16_t addr, LocoBase& out) {
    const LocoBase* loco = locoRepo().find(addr);
    if (loco) { out = *loco; return true; }
    memset(&out, 0, sizeof(LocoBase));
    out.address   = addr;
    out.forward   = true;
    out.stepMode  = 4;  // DCC128
    return false;
}

// ─────────────────────────────────────────────────────────────
//  StackSearchNext - Given a locomotive address (fromAddr),
//                    it returns the address of the next active
//                    locomotive in the repository — used for
//                    XpressNet's "stack search" feature,
//                    which lets a throttle step through the
//                    list of locomotives currently known to the
//                    command station one at a time.
// ─────────────────────────────────────────────────────────────
uint16_t XpressNetHandler::stackSearchNext(uint16_t fromAddr) const {
    uint8_t count = locoRepo().count();
    bool passFrom = (fromAddr == 0);
    for (uint8_t i = 0; i < count; i++) {
        const LocoBase* l = locoRepo().at(i);
        if (!l || !l->active) continue;
        if (passFrom) return l->address;
        if (l->address == fromAddr) passFrom = true;
    }
    return 0;
}

// ─────────────────────────────────────────────────────────────
//  Response builders
// ─────────────────────────────────────────────────────────────

// ─────────────────────────────────────────────────────────────
// BuildOk - Builds a standard XpressNet acknowledgement response
//           indicating that the received command was accepted
//           and send to the xpressnethandler to be processed
// ─────────────────────────────────────────────────────────────
uint8_t XpressNetHandler::buildOk(uint8_t* buf) {
    buf[0] = 0x01; buf[1] = 0x04;
    buf[2] = xorCheck(buf, 2); return 3;
}

// ─────────────────────────────────────────────────────────────
// buildError - Builds an XpressNet error response containing the
//              supplied error code. Used when a command cannot
//              be processed or contains invalid data.
// ─────────────────────────────────────────────────────────────
uint8_t XpressNetHandler::buildError(uint8_t* buf, uint8_t errCode) {
    buf[0] = 0x61; buf[1] = errCode;
    buf[2] = xorCheck(buf, 2); return 3;
}

// ─────────────────────────────────────────────────────────────
// buildlocoinfo - Builds a locomotive information response containing
//                 the current speed, direction, function states
//                 and operational status of the locomotive.
// ─────────────────────────────────────────────────────────────
uint8_t XpressNetHandler::buildLocoInfo(uint8_t* buf, const LocoBase& l) {
    // XpressNet E4 steps/avail byte (bits 1-0): 00=14-step, 01=28-step, 10=128-step
    // bit7=0 free, bit7=1 occupied by another device
    // Intern stepMode: 0=14-step, 2=28-step, 4=128-step
    uint8_t xnStep;
    uint8_t nemSpeed;
    switch (l.stepMode) {
        case 0:  xnStep = 0; nemSpeed = canonicalToNem(l.speed, 0); break;                  // 14 steps
        case 2:  xnStep = 1; nemSpeed = xn28Shuffle(canonicalToNem(l.speed, 2)); break;     // 28 ststeps
        default: xnStep = 2; nemSpeed = l.speed & 0x7F;               break;                // 128 steps (intern 4)
    }
    uint8_t spdDir = (l.forward ? 0x80 : 0x00) | (nemSpeed & 0x7F);
    uint32_t fn    = l.functions;

    // XpressNet DIRF byte: bit4=F0, bit3=F4, bit2=F3, bit1=F2, bit0=F1
    // Internal functions:    bit0=F0, bit1=F1, bit2=F2, bit3=F3, bit4=F4
    uint8_t f0f4 = (((fn>>0)&1) << 4)   // F0 -> bit4
                 | (((fn>>4)&1) << 3)   // F4 -> bit3
                 | (((fn>>3)&1) << 2)   // F3 -> bit2
                 | (((fn>>2)&1) << 1)   // F2 -> bit1
                 | (((fn>>1)&1) << 0);  // F1 -> bit0

    uint8_t f5f12 = (fn >> 5) & 0xFF;

    // Standard XpressNet v3.6 Locomotive Information Response:
    // E4 [avail] [SpeedDir] [DIRF] [F5F12] [XOR]  — WITHOUT address in the data bytes
    // E6 does not exist in the spec and confuses the LocoMouse2/MultiMaus.
    buf[0] = 0xE4; buf[1] = xnStep;
    buf[2] = spdDir; buf[3] = f0f4; buf[4] = f5f12;
    buf[5] = xorCheck(buf, 5); return 6;
}

// ─────────────────────────────────────────────────────────────
//  Build locoinfoF13F28 - builds the response on the locomotive
//                         function command with the status of
//                         the functions F13 .. F28
// ─────────────────────────────────────────────────────────────
uint8_t XpressNetHandler::buildLocoInfoF13F28(uint8_t* buf, const LocoBase& l) {
    uint32_t fn = l.functions;
    buf[0] = 0xE3; buf[1] = 0x52;
    buf[2] = (fn >> 13) & 0xFF;   // F13-F20
    buf[3] = (fn >> 21) & 0xFF;   // F21-F28
    buf[4] = xorCheck(buf, 4); return 5;
}

// ── §2.19.8 / §2.20 / §2.21 — "Funktionsstatus" (momentary) ──
// Not to be confused with "Funktionszustand" (on/off, see above and
// buildFuncZustandF29F68). Status = whether the function is configured
// as momentary.

// ─────────────────────────────────────────────────────────────
// packF29F68 - this function rearranges two separate function-status representations
//              (a 32-bit base part and an extended byte array).
//               combines them in a 5-byte-array F29-F68 for the xpressnet-protocol
//               that it in the specific messages expects.
// ─────────────────────────────────────────────────────────────
void XpressNetHandler::packF29F68(uint32_t low32, const uint8_t ext5[5],
                                    uint8_t out[5]) {
    auto getBit = [&](int fn) -> uint8_t {
        if (fn <= 31) return (low32 >> fn) & 1;
        int idx = fn - 32;  // bit0 of ext5[0] = F32
        return (ext5[idx / 8] >> (idx % 8)) & 1;
    };
    for (int b = 0; b < 5; b++) {
        uint8_t v = 0;
        for (int bit = 0; bit < 8; bit++) {
            int fn = 29 + b * 8 + bit;   // groep b: F(29+8b) .. F(36+8b)
            if (fn > 68) break;
            v |= (getBit(fn) << bit);
        }
        out[b] = v;
    }
}

// ─────────────────────────────────────────────────────────────
// buildfuncstatusF0F12 -  builds the response on the locomotive
//                         function status command with the status of
//                         the functions F0 .. F12.
//                         message: 0xE3|0x50|S0|S1|Xor
//                          S0: 0|0|0|S0|S4|S3|S2|S1
//                          S1: S12|S11|S10|S9|S8|S7|S6|S5
//
// ─────────────────────────────────────────────────────────────
uint8_t XpressNetHandler::buildFuncStatusF0F12(uint8_t* buf, const LocoBase& l) {
    uint32_t mo = l.momentary;
    // §2.19.8 Daten1: bit4=S0(F0), bit3=S4, bit2=S3, bit1=S2, bit0=S1
    // (identical bit-sequence as the  DIRF-byte in buildLocoInfo)
    uint8_t s0 = (((mo>>0)&1) << 4) | (((mo>>4)&1) << 3) | (((mo>>3)&1) << 2)
               | (((mo>>2)&1) << 1) | (((mo>>1)&1) << 0);
    // Daten2: bit0=S5 .. bit7=S12
    uint8_t s1 = (mo >> 5) & 0xFF;
    buf[0] = 0xE3; buf[1] = 0x50;
    buf[2] = s0;   buf[3] = s1;
    buf[4] = xorCheck(buf, 4); return 5;
}

// ─────────────────────────────────────────────────────────────
// buildFuncStstausF13F28 - build function status respone for
//                          F13 .. F28 message: 0xE4|0x51|S0|S1|R|X-or
//                          S0=S20|S19|S18|S17|S16|S15|S14|S13
//                          S1=S28|S27|S26|S25|S24|S23|S22|S21
//                          R = 0x0F fixed on refresh mode F0..F28
// ─────────────────────────────────────────────────────────────
uint8_t XpressNetHandler::buildFuncStatusF13F28(uint8_t* buf, const LocoBase& l) {
    uint32_t mo = l.momentary;
    buf[0] = 0xE4; buf[1] = 0x51;
    buf[2] = (mo >> 13) & 0xFF;  // S0: F13-F20
    buf[3] = (mo >> 21) & 0xFF;  // S1: F21-F28
    buf[4] = 0x0F;               // R: refresh-modus, vast op "F0..F28"
    buf[5] = xorCheck(buf, 5); return 6;
}

// ─────────────────────────────────────────────────────────────
// buildFuncStatusF29F68 - build function status response for
//                          F29 .. F68
//                          §2.21 message: 0xE6|0x54|S0|S1|S2|S3|S4|X-Or
//                          S0: S36|S35|S34|S33|S32|S31|S30|S29
//                          S1: S44|S43|S42|S41|S40|S39|S38|S37
//                          S3: S52|S51|S50|S49|S48|S47|S46|S45
//                          S4: S60|S59|S58|S57|S56|S55|S54|S53
//                          S5: S68|S67|S66|S65|S64|S63|S62|S61
// ─────────────────────────────────────────────────────────────
uint8_t XpressNetHandler::buildFuncStatusF29F68(uint8_t* buf, const LocoBase& l) {
    uint8_t s[5];
    packF29F68(l.momentary, l.momentaryExt, s);
    buf[0] = 0xE6; buf[1] = 0x54;
    buf[2] = s[0]; buf[3] = s[1]; buf[4] = s[2]; buf[5] = s[3]; buf[6] = s[4];
    buf[7] = xorCheck(buf, 7); return 8;
}

// ─────────────────────────────────────────────────────────────
// buildFuncZustandF29F86 - build response  for F29..F68 zustand
//                          message: 0xE6|0x53|S0|S1|S2|S3|S4|S5|X-or
//                          S0: S36|S35|S34|S33|S32|S31|S30|S29
//                          S1: S44|S43|S42|S41|S40|S39|S38|S37
//                          S3: S52|S51|S50|S49|S48|S47|S46|S45
//                          S4: S60|S59|S58|S57|S56|S55|S54|S53
//                          S5: S68|S67|S66|S65|S64|S63|S62|S61
// ─────────────────────────────────────────────────────────────
uint8_t XpressNetHandler::buildFuncZustandF29F68(uint8_t* buf, const LocoBase& l) {
    uint8_t d[5];
    packF29F68(l.functions, l.functionsExt, d);
    buf[0] = 0xE6; buf[1] = 0x53;
    buf[2] = d[0]; buf[3] = d[1]; buf[4] = d[2]; buf[5] = d[3]; buf[6] = d[4];
    buf[7] = xorCheck(buf, 7); return 8;
}
// ─────────────────────────────────────────────────────────────
// Build CentraleStatus - building reponse messgae for command
//                          station status response
//                          message : 0x62|0x22|DAT|X-or
//  DAT:  Status byte definitie (XpressNet spec):
// bit0 = emergency stop active
// bit1 = track power OFF
// bit2 = service mode active
// bit3 = power up (command station just started, not yet fully operational)
// bit4 = RAM check error (always 0 for Lenz under normal operation)
// bit6 = cold start (first boot after reset)
// ─────────────────────────────────────────────────────────────
uint8_t XpressNetHandler::buildCentraleStatus(uint8_t* buf) {

    uint8_t st = 0;
    if (gCentrale.emergencyStop)  st |= 0x01;
    if (gCentrale.trackPowerOff)  st |= 0x02;
    if (gCentrale.progModeActive) st |= 0x08;
    buf[0] = 0x62; buf[1] = 0x22; buf[2] = st;
    buf[3] = xorCheck(buf, 3); return 4;
}

// ─────────────────────────────────────────────────────────────
// build SoftwareVersion -  Creates a response containing the
//                          firmware or protocol version information
//                          requested by an XpressNet client.
//                          message: 0x63|0x21|DAT1|DAT2|X-or
//
// ─────────────────────────────────────────────────────────────
uint8_t XpressNetHandler::buildSoftwareVersion(uint8_t* buf) {
    // Version and identification, configurable via the web interface
    // (xn.version / xn.kennung)
    // xn.version: 0=v3.2 (0x32), 1=v3.6 (0x36), 2=v4.0 (0x40)
    // xn.kennung: 0=LZ100 (0x00), 1=Roco MultiMaus (0x10)
    static const uint8_t kVersionBytes[] = { 0x32, 0x36, 0x40 };
    static const uint8_t kKennungBytes[] = { 0x00, 0x10 };

    const ParamDef* pv = eepromStore().findParam("xn.version");
    const ParamDef* pk = eepromStore().findParam("xn.kennung");
    uint8_t vi = pv ? pv->value.u8 : 1;  // default v3.6
    uint8_t ki = pk ? pk->value.u8 : 0;  // default LZ100

    if (vi >= sizeof(kVersionBytes)) vi = 1;
    if (ki >= sizeof(kKennungBytes)) ki = 0;

    buf[0] = 0x63; buf[1] = 0x21;
    buf[2] = kVersionBytes[vi];
    buf[3] = kKennungBytes[ki];
    buf[4] = xorCheck(buf, 4); return 5;
}

// ─────────────────────────────────────────────────────────────
// buildSearchResult - build response message for search
//                     message: 0xE3|0x30+kind|AddrH|AddrL|X-or
// ─────────────────────────────────────────────────────────────
uint8_t XpressNetHandler::buildSearchResult(uint8_t* buf,
                                              uint16_t addr, uint8_t kind) {
    uint8_t ah, al; encodeAddr(addr, ah, al);
    buf[0] = 0xE3; buf[1] = 0x30 + kind;
    buf[2] = ah;   buf[3] = al;
    buf[4] = xorCheck(buf, 4); return 5;
}

// ─────────────────────────────────────────────────────────────
// buildSwitchInfo — Creates an accessory status response containing
//                   the current turnout or accessory state as
//                   maintained by the command station.
//                   message: 0x42|adr|state|X-or
// ─────────────────────────────────────────────────────────────
uint8_t XpressNetHandler::buildSwitchInfo(uint8_t* buf, uint8_t adr,
                                            uint8_t nibble, uint8_t state) {
    buf[0] = 0x42; buf[1] = adr; buf[2] = state;
    buf[3] = xorCheck(buf, 3); return 4;
}

// ─────────────────────────────────────────────────────────────
//  Broadcasts
// ─────────────────────────────────────────────────────────────

// ─────────────────────────────────────────────────────────────
// broadcastPowerOn — Sends the current track power status to
//                    all connected XpressNet clients.
//                    Used after power state changes and during
//                    status synchronization.
// ─────────────────────────────────────────────────────────────
void XpressNetHandler::broadcastPowerOn() {
    if (!_broadcastFn) return;
    uint8_t bc[] = { 0x61, 0x01, 0x60 }; _broadcastFn(bc, 3);
}

// ─────────────────────────────────────────────────────────────
// broadcastPowerOff — sends the "track power off" broadcast (0x61 0x00)
// to all connected clients.
// ─────────────────────────────────────────────────────────────
void XpressNetHandler::broadcastPowerOff() {
    if (!_broadcastFn) return;
    uint8_t bc[] = { 0x61, 0x00, 0x61 }; _broadcastFn(bc, 3);
}

// ─────────────────────────────────────────────────────────────
// broadcastEmergencyStop — Broadcasts a global emergency stop event
//                          to all connected clients so that user
//                          interfaces remain synchronized with
//                          the command station.
// ─────────────────────────────────────────────────────────────
void XpressNetHandler::broadcastEmergencyStop() {
    if (!_broadcastFn) return;
    uint8_t bc[] = { 0x81, 0x00, 0x81 }; _broadcastFn(bc, 3);
}

// ─────────────────────────────────────────────────────────────
// broadcastProgMode — sends the "programming mode active" broadcast
// (0x61 0x02) to all connected clients.
// ─────────────────────────────────────────────────────────────
void XpressNetHandler::broadcastProgMode() {
    if (!_broadcastFn) return;
    uint8_t bc[] = { 0x61, 0x02, 0x63 }; _broadcastFn(bc, 3);
}
// ─────────────────────────────────────────────────────────────
// broadcastLocoInfo - builds the loc info message en broadcasts
//                     it to clients:
// ─────────────────────────────────────────────────────────────
void XpressNetHandler::broadcastLocoInfo(uint16_t addr) {
    if (!_broadcastFn) return;
    // query loco repository for the loco
    LocoBase l; queryLoco(addr, l);
    // build the loco-info message
    uint8_t bc[8]; uint8_t len = buildLocoInfo(bc, l);
    _broadcastFn(bc, len);
}
// ─────────────────────────────────────────────────────────────
//  Command handlers
// ─────────────────────────────────────────────────────────────
// Interface info handlers
// ─────────────────────────────────────────────────────────────
// HandleIFVersion -  builds the Interface version message
//                    message: 0x02|VV|CC|X-or
//                    VV: version ind BCD format
//                    CC: code number in BCD format
// ─────────────────────────────────────────────────────────────
XNHandleResult XpressNetHandler::handleIfVersion(uint8_t* r, uint8_t& l) {
    // response : 0x02 0x36 0x01 XOR (interface version 3.6, type 1)
    r[0]=0x02; r[1]=0x36; r[2]=0x01;
    r[3]=xorCheck(r,3); l=4; return XNHandleResult::REPLY;
}

// ─────────────────────────────────────────────────────────────
// HandleIfStatus - builds respone to the interface status command
//                  message: 0xF2|0x02|RRRR RRRA| x-or
//
// ─────────────────────────────────────────────────────────────
XNHandleResult XpressNetHandler::handleIfStatus(uint8_t* r, uint8_t& l) {
    // Life-check response: 0xF2 0x01 status XOR
    // status bits: rrrr rrra
    //   bit0 (a) = 1: interface receives data from the command station (always 1)
    uint8_t st = 0x01;  // bit0 always 1 = interface active
    r[0]=0xF2; r[1]=0x01; r[2]=st;
    r[3]=xorCheck(r,3); l=4; return XNHandleResult::REPLY;
}
// ─────────────────────────────────────────────────────────────
// handleXnVersion - builds the XpressNet protocol version response
//                   message: 0xF2|0x02|0x36|X-or
// ─────────────────────────────────────────────────────────────
XNHandleResult XpressNetHandler::handleXnVersion(uint8_t* r, uint8_t& l) {
    // XpressNet version: 0xF2 0x02 0x36 XOR
    r[0]=0xF2; r[1]=0x02; r[2]=0x36;
    r[3]=xorCheck(r,3); l=4; return XNHandleResult::REPLY;
}

// ─────────────────────────────────────────────────────────────
// HandleDeviceAdd - builds an send response to the command device address
//                   response message: 0xF2|0x01|ADR|X-or
// ─────────────────────────────────────────────────────────────
XNHandleResult XpressNetHandler::handleDeviceAddr(uint8_t* r, uint8_t& l) {
    // F2 01 addr XOR — read or write device address (1-31)
    // r[2] = current address (stored in SV1)
    uint8_t addr = gCentrale.sv[0];  // SV1 = device address
    if (addr == 0 || addr > 31) addr = 1;
    r[0]=0xF2; r[1]=0x01; r[2]=addr;
    r[3]=xorCheck(r,3); l=4; return XNHandleResult::REPLY;
}

// ─────────────────────────────────────────────────────────────
// handleExtraversion - builds the response to the extraversion command
//                      response:
//                      0x67|0x23|ZBldH|ZBldL|RMVer|RMBldH|RMBldL|BIVers|X-or
//
// ─────────────────────────────────────────────────────────────
XNHandleResult XpressNetHandler::handleExtraVersion(uint8_t* r, uint8_t& l) {
    // Extended software version:
    // 0x67 0x23 ZBldH ZBldL RMVer RMBldH RMBldL BIVers XOR
    r[0]=0x67; r[1]=0x23;
    r[2]=(uint8_t)(gCentrale.buildNr >> 8);    // ZBldH
    r[3]=(uint8_t)(gCentrale.buildNr & 0xFF);  // ZBldL
    r[4]=gCentrale.rmVersion;                  // RMVer
    r[5]=(uint8_t)(gCentrale.rmBuildNr >> 8);  // RMBldH
    r[6]=(uint8_t)(gCentrale.rmBuildNr & 0xFF);// RMBldL
    r[7]=gCentrale.bootloaderVer;              // BIVers
    r[8]=xorCheck(r,8); l=9; return XNHandleResult::REPLY;
}

// ─────────────────────────────────────────────────────────────
// handleNumConnn - build and sends response to teh number of
//                  connections command.
//                  message: 0xF2|0x03|AA|X-or
// ─────────────────────────────────────────────────────────────
XNHandleResult XpressNetHandler::handleNumConn(uint8_t* r, uint8_t& l) {
    // Needs refinement to real available connections
    r[0]=0xF2; r[1]=0x03;
    // for now fixed on max 8 connections
    r[2]=0x08;
    r[3]=xorCheck(r,3); l=4; return XNHandleResult::REPLY;
}

// ─────────────────────────────────────────────────────────────
// HandlePowerOn - Processes a track power ON command received via
//                 XpressNet. Enables track voltage, updates the
//                 internal command station state and notifies all
//                 connected clients about the new operating condition.
// ─────────────────────────────────────────────────────────────
XNHandleResult XpressNetHandler::handlePowerOn(uint8_t* r, uint8_t& l) {
    // reset relevant command station flags
    gCentrale.emergencyStop = false;
    gCentrale.trackPowerOff = false;
    gCentrale.shortCircuit  = false;  // explicit power-on clears short-circuit state
    Command cmd{}; cmd.type = CmdType::POWER_ON; cmd.sourceId = _srcId;
    CommandBus::instance().dispatch(cmd);
    
    // no broadcastPowerOn() here, this is done in DccHal
    l = 0; return XNHandleResult::BROADCAST_ONLY;
}

// ─────────────────────────────────────────────────────────────
// handlePowerOff - Processes a track power OFF command. Disables
//                  track voltage, updates the command station
//                  state and broadcasts the power-off condition
//                  to all connected interfaces.
//
// ─────────────────────────────────────────────────────────────
XNHandleResult XpressNetHandler::handlePowerOff(uint8_t* r, uint8_t& l) {
    gCentrale.trackPowerOff = true;
    Command cmd{}; cmd.type = CmdType::POWER_OFF; cmd.sourceId = _srcId;
    CommandBus::instance().dispatch(cmd);
    l = 0; return XNHandleResult::BROADCAST_ONLY;
}

// ─────────────────────────────────────────────────────────────
// HandleEmergSTop - Broadcasts a global emergency stop event
//                   to all connected clients so that user interfaces
//                   remain synchronized with the command station.
//                   response: 0x81|0x00|x-or
// ─────────────────────────────────────────────────────────────
XNHandleResult XpressNetHandler::handleEmergStop(uint8_t* r, uint8_t& l) {
    gCentrale.emergencyStop = true;
    Command cmd{}; cmd.type = CmdType::EMERGENCY_STOP; cmd.sourceId = _srcId;
    CommandBus::instance().dispatch(cmd);
    broadcastEmergencyStop();
    // Send emergency stop status as a direct response
    r[0]=0x81; r[1]=0x00; r[2]=xorCheck(r,2); l=3;
    return XNHandleResult::REPLY;
}

// ─────────────────────────────────────────────────────────────
// handleOneLocoStop - Executes an emergency stop for a single
//                     locomotive while leaving the remainder of
//                     the layout operating normally.
//                     It updates the loc repository to relect the
//                     actual information.
// ─────────────────────────────────────────────────────────────
XNHandleResult XpressNetHandler::handleOneLocoStop(const uint8_t* in, uint8_t il,
                                                     uint8_t* r, uint8_t& l) {
    if (il < 4) { l = buildError(r, 0x82); return XNHandleResult::REPLY; }
    // search the loco repository for the intended loco and update the
    // repository entry
    uint16_t addr = decodeAddr(in[1], in[2]);
    LocoBase loco; queryLoco(addr, loco);
    updateLoco(addr, 1, loco.forward ? 1 : 0, loco.functions,
               static_cast<SpeedStepMode>(loco.stepMode));
    loco.speed = 1;
    // build loco info response
    uint8_t bc[8]; uint8_t bcLen = buildLocoInfo(bc, loco);
    // broadcast loco info if needed
    if (_broadcastFn) _broadcastFn(bc, bcLen);
    l = buildOk(r); return XNHandleResult::OK_SILENT;
}

// ─────────────────────────────────────────────────────────────
// HandleGetVersion - Handles the version command, it builds the version
//                    info en send the reponse.
// ─────────────────────────────────────────────────────────────
XNHandleResult XpressNetHandler::handleGetVersion(uint8_t* r, uint8_t& l) {
    l = buildSoftwareVersion(r); return XNHandleResult::REPLY;
}

// ─────────────────────────────────────────────────────────────
// HandleGetStatus- returns thet status of the commandstation
//                  at request
// ─────────────────────────────────────────────────────────────
XNHandleResult XpressNetHandler::handleGetStatus(uint8_t* r, uint8_t& l) {
    l = buildCentraleStatus(r); return XNHandleResult::REPLY;
}

// ─────────────────────────────────────────────────────────────
// ── MultiMaus loco+function info request (E3 F0 AH AL XOR) ──
// Response: E7 Steps Speed Dirf F1 F2 F3 00 00 XOR
// Conform Z21-PG SetLocoInfoMM implementatie
// ─────────────────────────────────────────────────────────────
XNHandleResult XpressNetHandler::handleLocoQueryMM(const uint8_t* in, uint8_t il,
                                                    uint8_t* r, uint8_t& l) {
    if (il < 5) { l = buildError(r, 0x82); return XNHandleResult::REPLY; }
    uint16_t addr = decodeAddr(in[2], in[3]);
    LocoBase loco; queryLoco(addr, loco);

    // Steps byte: 0x04 = 128 steps, 0x02 = 28 steps, 0x00 = 14 steps
    uint8_t steps = 0x04; // default 128 steps
    switch (loco.stepMode) {
        case 0: steps = 0x00; break; // 14 steps
        case 2: steps = 0x02; break; // 28 steps
        default: steps = 0x04; break; // 128 steps
    }

    // Speed byte: bit7=direction, bits6-0=speed (NEM671, based on stepMode)
    uint8_t nemSpeed;
    switch (loco.stepMode) {
        case 0:  nemSpeed = canonicalToNem(loco.speed, 0); break;
        case 2:  nemSpeed = xn28Shuffle(canonicalToNem(loco.speed, 2)); break;
        default: nemSpeed = loco.speed & 0x7F;               break;
    }
    uint8_t spd = (loco.forward ? 0x80 : 0x00) | (nemSpeed & 0x7F);

    // DIRF: bit5=direction, bit4=F0, bit3=F4, bit2=F3, bit1=F2, bit0=F1
    uint8_t dirf = loco.forward ? 0x20 : 0x00;
    if (loco.functions & 0x001) dirf |= 0x10; // F0
    if (loco.functions & 0x002) dirf |= 0x01; // F1
    if (loco.functions & 0x004) dirf |= 0x02; // F2
    if (loco.functions & 0x008) dirf |= 0x04; // F3
    if (loco.functions & 0x010) dirf |= 0x08; // F4

    // F1: F5-F8, F2: F9-F12, F3: F13-F20
    uint8_t f1 = (loco.functions >> 5)  & 0x0F; // F5-F8
    uint8_t f2 = (loco.functions >> 9)  & 0x0F; // F9-F12
    uint8_t f3 = (loco.functions >> 13) & 0xFF; // F13-F20

    r[0] = 0xE7; r[1] = steps; r[2] = spd; r[3] = dirf;
    r[4] = f1;   r[5] = f2;    r[6] = f3;  r[7] = 0x00; r[8] = 0x00;
    r[9] = xorCheck(r, 9);
    l = 10;
    return XNHandleResult::REPLY;
}

// ─────────────────────────────────────────────────────────────
// HandlelocoQuery - searches the intended loc in the loc repository
//                   and respons with the locoinformation
// ─────────────────────────────────────────────────────────────
XNHandleResult XpressNetHandler::handleLocoQuery(const uint8_t* in, uint8_t il,
                                                   uint8_t* r, uint8_t& l) {
    if (il < 5) { l = buildError(r, 0x82); return XNHandleResult::REPLY; }
    uint16_t addr = decodeAddr(in[2], in[3]);
    LocoBase loco; queryLoco(addr, loco);
    l = buildLocoInfo(r, loco); return XNHandleResult::REPLY;
}

// ─────────────────────────────────────────────────────────────
// ── LAN_X_SET_LOCO_BINARY_STATE (E5 5F) — stub ──────────────
// E5 5F addr_h addr_l stat_low stat_high XOR
// 16-bit: bit15=state, bits14-0=number (1..32767)
// States 1..32 are stored, higher ones are ignored
// ─────────────────────────────────────────────────────────────
XNHandleResult XpressNetHandler::handleLocoBinaryState(const uint8_t* in, uint8_t il,
                                                        uint8_t* r, uint8_t& l) {
    if (il < 7) { l = buildError(r, 0x82); return XNHandleResult::REPLY; }
    uint16_t addr    = decodeAddr(in[2], in[3]);
    uint16_t bs16    = ((uint16_t)in[5] << 8) | in[4];  // low byte first
    uint8_t  state   = (bs16 >> 15) & 1;
    uint16_t bsNum   = bs16 & 0x7FFF;

    if (bsNum >= 1 && bsNum <= 32) {
        LocoBase* loco = locoRepo().findOrCreate(addr);
        if (loco) {
            if (state) loco->binaryStates |=  (1UL << (bsNum - 1));
            else       loco->binaryStates &= ~(1UL << (bsNum - 1));
        }
    }
    l = buildOk(r); return XNHandleResult::OK_SILENT;
}

// ─────────────────────────────────────────────────────────────
// ── LAN_X_SET_LOCO_FUNCTION (E4 F8) — Z21 extension ─────────
// data[4]: bit7=state (0=off, 1=on), bits6-0=function number (0..68)
// No direct response — broadcast only
// ─────────────────────────────────────────────────────────────
XNHandleResult XpressNetHandler::handleLocoFuncF8(const uint8_t* in, uint8_t il,
                                                    uint8_t* r, uint8_t& l) {
    if (il < 6) { l = buildError(r, 0x82); return XNHandleResult::REPLY; }
    uint16_t addr  = decodeAddr(in[2], in[3]);
    uint8_t  fb    = in[4];
    uint8_t  state = (fb >> 7) & 1;
    uint8_t  fnum  = fb & 0x7F;

    LocoBase loco; queryLoco(addr, loco);
    uint32_t fn = loco.functions;

    if (fnum <= 28) {
        if (state) fn |=  (1UL << fnum);
        else       fn &= ~(1UL << fnum);
    }
    // F29-F68 are not yet stored

    updateLocoFunctions(addr, fn);
    loco.functions = fn;
    uint8_t bc[8]; uint8_t bcLen = buildLocoInfo(bc, loco);
    if (_broadcastFn) _broadcastFn(bc, bcLen);
    l = buildOk(r); return XNHandleResult::OK_SILENT;
}

// ─────────────────────────────────────────────────────────────
//  nemToCanonical() / canonicalToNem() — see declaration in
//  xpressnet_handler.h. Non-static so z21_lan.cpp (which builds its
//  own X-Bus-style loco-info response) can reuse them too.
//
//  IMPORTANT: must use EXACTLY the same RCN-212 convention as
//  DccHal::_scaleSpeed() (canonical -> DCC-native), otherwise a
//  systematic distortion occurs on the return conversion:
//  - mode=0 (14-step): single reservation, 0=stop, 1=estop,
//    2-15=14 running steps.
//  - mode=2 (28-step): double reservation, 0/1=stop, 2/3=estop,
//    4-31=28 running steps (the intermediate-step bit "U" doubles
//    the stop/estop values but not the number of running steps).
//
//  nemToCanonical - converts NEM speed values to canonical values
// ─────────────────────────────────────────────────────────────
uint8_t nemToCanonical(uint8_t nem, uint8_t mode) {
    if (mode == 0) {
        // convert for 14-steps
        if (nem <= 1) return nem;
        const uint16_t span = 15 - 2, range = 125;
        uint16_t canon = 2 + ((uint16_t)(nem - 2) * range + span / 2) / span;
        return (uint8_t)(canon > 127 ? 127 : canon);
    } else {
        if (nem <= 1) return 0;             // 0 or 1 -> stop
        if (nem <= 3) return 1;             // 2 or 3 -> emergency stop
        const uint16_t span = 31 - 4, range = 125;
        uint16_t canon = 2 + ((uint16_t)(nem - 4) * range + span / 2) / span;
        return (uint8_t)(canon > 127 ? 127 : canon);
    }
}

// ─────────────────────────────────────────────────────────────
// canonicalToNem - converts canonical speed values to NEM values
// ─────────────────────────────────────────────────────────────
uint8_t canonicalToNem(uint8_t canon, uint8_t mode) {
    if (mode == 0) {
        // 14-steps
        if (canon <= 1) return canon;
        const uint16_t range = 125, span = 15 - 2;
        uint16_t nem = 2 + ((uint16_t)(canon - 2) * span + range / 2) / range;
        return (uint8_t)(nem > 15 ? 15 : nem);
    } else {
        // 28-steps
        if (canon == 0) return 0;
        if (canon == 1) return 2;
        const uint16_t range = 125, span = 31 - 4;
        uint16_t nem = 4 + ((uint16_t)(canon - 2) * span + range / 2) / range;
        return (uint8_t)(nem > 31 ? 31 : nem);
    }
}

// ─────────────────────────────────────────────────────────────
// xn28Unshuffle — undoes the NMRA/RCN G-bit shuffle used for 28-step
// speed values on the wire (in the XpressNet v4.0 RVVVVV speed field),
// returning a plain, non-shuffled 5-bit speed value. Uses the same
// shuffle convention as DCCPacketScheduler::setSpeed28() internally
// (confirmed via LocoNet, and now also against the XpressNet v4.0
// spec). Inverse of xn28Shuffle().
// ─────────────────────────────────────────────────────────────
uint8_t xn28Unshuffle(uint8_t wire) {
    return ((wire >> 4) & 0x01) | ((wire & 0x0F) << 1);
}

// ─────────────────────────────────────────────────────────────
// xn28Shuffle — applies the NMRA/RCN G-bit shuffle to a plain 5-bit
// canonical 28-step speed value, producing the wire-format value used
// on the XpressNet RVVVVV speed field. Inverse of xn28Unshuffle().
// ─────────────────────────────────────────────────────────────
uint8_t xn28Shuffle(uint8_t canon) {
    return ((canon & 0x01) << 4) | ((canon >> 1) & 0x0F);
}


// ─────────────────────────────────────────────────────────────
// handleLocoSpeed - Processes a locomotive speed command received
//                   via XpressNet. Decodes speed step information,
//                   direction flags and requested speed,
//                   updates the locomotive state repository and
//                   forwards the command to the locomotive control
//                   subsystem.
// ─────────────────────────────────────────────────────────────

XNHandleResult XpressNetHandler::handleLocoSpeed(const uint8_t* in, uint8_t il,
                                                   uint8_t* r, uint8_t& l) {
    if (il < 6) { l = buildError(r, 0x82); return XNHandleResult::REPLY; }
    uint8_t  id   = in[1];
    uint16_t addr = decodeAddr(in[2], in[3]);
    if (addr == 0) { l = buildError(r, 0x09); return XNHandleResult::REPLY; }
    uint8_t sb  = in[4];
    uint8_t dir = (sb & 0x80) ? 1 : 0;
    uint8_t spd;
    SpeedStepMode mode = SpeedStepMode::STEPS_128;
    if (id == 0x10) {
        mode = SpeedStepMode::STEPS_14;
        // NEM 671: 0=stop, 1=emergency stop, 2..15=running step 1..14.
        // Command.speed/LocoBase.speed are always canonical (0-127,
        // regardless of step mode — same convention as LocoNet), so
        // scale up to that range. DccHal::setSpeed() scales back down
        // to the library range (0-15) right before the DCC packet.
        spd = nemToCanonical(sb & 0x0F, 0);
    } else if (id == 0x12 || id == 0x11) {
        mode = SpeedStepMode::STEPS_28;
        // XpressNet v4.0 spec: the speed field is RVVVVV, where VVVVV
        // follows the same NMRA/RCN G-bit shuffle as the DCC packet
        // itself (confirmed by Rob against the v4.0 standard) — so
        // FIRST unshuffle with xn28Unshuffle() (the same shuffle as our
        // own DCCPacketScheduler::setSpeed28(), not the differing
        // Gahtow variant), then scale up to canonical.
        spd = nemToCanonical(xn28Unshuffle(sb & 0x1F), 2);
    } else {
        spd  = sb & 0x7F;  // 128-step: already canonical 0..127
    }
    LocoBase loco; queryLoco(addr, loco);
    updateLoco(addr, spd, dir, loco.functions, mode);
    loco.speed = spd; loco.forward = (dir != 0);
    loco.stepMode = static_cast<uint8_t>(mode);  // otherwise buildLocoInfo() would use the stale stepMode from before this command
    uint8_t bc[8]; uint8_t bcLen = buildLocoInfo(bc, loco);
    if (_broadcastFn) _broadcastFn(bc, bcLen);
    l = buildOk(r); return XNHandleResult::OK_SILENT;
}

// ─────────────────────────────────────────────────────────────
// HandleLocoFunc - function to handle the loc functions commands
//                  selects loc from loc repository, builds loc
//                  info message and broadcast if needed
// ─────────────────────────────────────────────────────────────
XNHandleResult XpressNetHandler::handleLocoFunc(const uint8_t* in, uint8_t il,
                                                  uint8_t* r, uint8_t& l) {
    if (il < 6) { l = buildError(r, 0x82); return XNHandleResult::REPLY; }
    uint8_t  id   = in[1];
    uint16_t addr = decodeAddr(in[2], in[3]);
    uint8_t  fb   = in[4];
    LocoBase loco; queryLoco(addr, loco);
    uint32_t fn = loco.functions;
    switch (id) {
        case 0x20: fn=(fn&~0x1FUL)|((fb&0x10)>>4)|((fb&0x0F)<<1); break;
        case 0x21: fn=(fn&~(0x0FUL<<5))|((uint32_t)(fb&0x0F)<<5); break;
        case 0x22: fn=(fn&~(0x0FUL<<9))|((uint32_t)(fb&0x0F)<<9); break;
        case 0x23: fn=(fn&~(0xFFUL<<13))|((uint32_t)fb<<13); break;
        case 0x28: fn=(fn&~(0xFFUL<<21))|((uint32_t)fb<<21); break;
        default: break;
    }
    updateLocoFunctions(addr, fn);
    loco.functions = fn;
    uint8_t bc[8]; uint8_t bcLen = buildLocoInfo(bc, loco);
    if (_broadcastFn) _broadcastFn(bc, bcLen);
    l = buildOk(r); return XNHandleResult::OK_SILENT;
}

// ─────────────────────────────────────────────────────────────
// handleFuncStatus - looks up the requested loco and returns its
//                    function status/state for the requested F-range
//                    (E3 subcode selects which group and which kind).
// ─────────────────────────────────────────────────────────────
XNHandleResult XpressNetHandler::handleFuncStatus(const uint8_t* in, uint8_t il,
                                                    uint8_t* r, uint8_t& l) {
    // E3 subcodes — two families, not to be confused:
    //   "Status"  (momentary):            0x07 (F0-12), 0x08 (F13-28), 0x0A (F29-68)
    //   "Zustand" (on/off, current value): 0x09 (F13-28), 0x0B (F29-68)
    // F0-12 Zustand has no separate command: that's already in the normal E4 loco-info.
    if (il < 5) { l = buildError(r, 0x82); return XNHandleResult::REPLY; }
    uint16_t addr = decodeAddr(in[2], in[3]);
    LocoBase loco; queryLoco(addr, loco);
    switch (in[1]) {
        case 0x07: l = buildFuncStatusF0F12   (r, loco); break;
        case 0x08: l = buildFuncStatusF13F28  (r, loco); break;
        case 0x0A: l = buildFuncStatusF29F68  (r, loco); break;
        case 0x09: l = buildLocoInfoF13F28    (r, loco); break;  // Zustand F13-28 (E3 0x52)
        case 0x0B: l = buildFuncZustandF29F68 (r, loco); break;  // Zustand F29-68 (E6 0x53)
        default:   l = buildError(r, 0x82); break;
    }
    return XNHandleResult::REPLY;
}

// ─────────────────────────────────────────────────────────────
// ── §3.41.6 — "Funktionsstatus setzen" (set function status) group table ──
// Returns the starting function number and bit count for a given
// Kennung (id) byte. count==0 means: unknown Kennung.
// ─────────────────────────────────────────────────────────────
namespace {
struct FuncStatusGroup { uint8_t startFn; uint8_t count; };
FuncStatusGroup funcStatusGroup(uint8_t kennung) {
    switch (kennung) {
        case 0x24: return {0,  5};  // Group 1: F0-F4
        case 0x25: return {5,  4};  // Group 2: F5-F8
        case 0x26: return {9,  4};  // Group 3: F9-F12
        case 0x27: return {13, 8};  // Group 4: F13-F20
        case 0x2C: return {21, 8};  // Group 5: F21-F28
        case 0x2D: return {29, 8};  // Group 6: F29-F36
        case 0x2E: return {37, 8};  // Group 7: F37-F44
        case 0x52: return {45, 8};  // Group 8: F45-F52
        case 0x53: return {53, 8};  // Group 9: F53-F60
        case 0x54: return {61, 8};  // Group 10: F61-F68
        default:   return {0,  0};
    }
}
}  // namespace

// ─────────────────────────────────────────────────────────────
// handleFuncStatusSet - sets the "momentary/status" bits for a group
//                       of functions (F0-F68, depending on group id)
//                       on the addressed loco, using the mask byte.
// ─────────────────────────────────────────────────────────────
XNHandleResult XpressNetHandler::handleFuncStatusSet(const uint8_t* in, uint8_t il,
                                                       uint8_t* r, uint8_t& l) {
    if (il < 6) { l = buildError(r, 0x82); return XNHandleResult::REPLY; }
    FuncStatusGroup grp = funcStatusGroup(in[1]);
    if (grp.count == 0) { l = buildError(r, 0x82); return XNHandleResult::REPLY; }

    uint16_t addr = decodeAddr(in[2], in[3]);
    uint8_t  mask = in[4];
    LocoBase* loco = locoRepo().findOrCreate(addr);
    if (loco) {
        for (uint8_t i = 0; i < grp.count; i++) {
            uint8_t fn  = grp.startFn + i;
            bool    set = (mask >> i) & 1;
            if (fn <= 31) {
                if (set) loco->momentary |=  (1UL << fn);
                else     loco->momentary &= ~(1UL << fn);
            } else {
                uint8_t idx = fn - 32;             // bit0 of momentaryExt[0] = F32
                uint8_t bit = 1U << (idx % 8);
                if (set) loco->momentaryExt[idx / 8] |=  bit;
                else     loco->momentaryExt[idx / 8] &= ~bit;
            }
        }
    }
    l = buildOk(r); return XNHandleResult::OK_SILENT;
}

// ─────────────────────────────────────────────────────────────
// handleTurnout — handles the standard XpressNet 0x52 "set turnout"
//                 command: decodes the target address and direction,
//                 stores the new state in the accessory table,
//                 dispatches an ACCESSORY_SET command onto the command
//                 bus (which DccHal turns into an actual DCC packet),
//                 optionally sends a 0x42 feedback broadcast, and
//                 persists the accessory state to EEPROM on activate.
// ─────────────────────────────────────────────────────────────
XNHandleResult XpressNetHandler::handleTurnout(const uint8_t* in, uint8_t il,
                                                 uint8_t* r, uint8_t& l) {
    if (il < 4) { l = buildError(r, 0x82); return XNHandleResult::REPLY; }

    uint8_t  d1         = (in[2] >> 3) & 0x01;   // D1: activate pulse (1=on, 0=off)
    uint8_t  d2         = (in[2] >> 0) & 0x01;   // D2: direction — RCN-213: 0=thrown/diverging, 1=straight
    uint16_t clientAddr = decodeTurnoutAddr(in[1], in[2]);
    uint8_t  output     = d2;                     // pass through directly — output follows the same convention as D2/R

    // Store the state in the accessory table (RCN-213: 0=thrown, 1=straight)
    if (clientAddr < MAX_ACCESSORIES && d1) {
        if (output == 0)
            gAccessories[clientAddr].state |= 0x01;   // thrown/diverging
        else
            gAccessories[clientAddr].state &= ~0x01;  // straight
        gAccessories[clientAddr].known = true;
    }

    Command cmd{};
    cmd.type     = CmdType::ACCESSORY_SET;
    cmd.sourceId = _srcId;
    cmd.address  = clientAddr;
    cmd.output   = output;
    cmd.active   = (d1 != 0);
    CommandBus::instance().dispatch(cmd);

    // 0x42 feedback broadcast — Lenz format (per logfile analysis).
    // Only on C=1 (activate). No broadcast on deactivate.
    // dataByte is CUMULATIVE: shows the known state of all 4 TTs.
    //   bit = TT*2 + (thrown=1 ? 1 : 0)  → each output is one bit
    //   bit0=TT0 straight, bit1=TT0 thrown, bit2=TT1 straight, ...
    if (_broadcastFn && d1 && eepromStore().getBool("xn.sw_broadcast")) {
        uint8_t  decoder  = in[1] & 0x3F;
        uint8_t  dataByte = 0;
        for (uint8_t tt = 0; tt < 4; tt++) {
            uint16_t a = (uint16_t)decoder * 4 + tt + 1;
            if (a < MAX_ACCESSORIES && gAccessories[a].known) {
                uint8_t out = gAccessories[a].state & 0x01; // 0=straight, 1=thrown
                dataByte |= (uint8_t)(1u << (tt * 2 + out));
            }
        }
        uint8_t bc[4];
        bc[0] = 0x42; bc[1] = in[1]; bc[2] = dataByte;
        bc[3] = bc[0] ^ bc[1] ^ bc[2];
        _broadcastFn(bc, 4);
    }

    // Persist the turnout state to EEPROM on activate (d1=1)
    if (d1) eepromStore().saveAccessories();

    l = buildOk(r); return XNHandleResult::OK_SILENT;
}

// ─────────────────────────────────────────────────────────────
// ── LAN_X_GET_TURNOUT_INFO (0x43) ────────────────────────────
// Request:  43 FAdr_MSB FAdr_LSB XOR
// Response: 43 FAdr_MSB FAdr_LSB state XOR
// state: bit1=position (0=green/straight, 1=red/thrown), bit0=active
// ── LAN_X_SET_TURNOUT Z21 format (0x53) ─────────────────────
// data: 53 AdrMSB AdrLSB Dat XOR
// Dat: bit7=activate, bit0=output (0=green/straight, 1=red/thrown)
//
// handleTurnoutZ21 — handles the XpressNet v4.0 0x53 "set turnout"
// command (used e.g. by Z21 clients). Same overall behaviour as
// handleTurnout() (0x52), but with a different address/data-byte
// layout as used natively in the Z21-style X-Bus format.
// ─────────────────────────────────────────────────────────────
XNHandleResult XpressNetHandler::handleTurnoutZ21(const uint8_t* in, uint8_t il,
                                                   uint8_t* r, uint8_t& l) {
    if (il < 5) { l = buildError(r, 0x82); return XNHandleResult::REPLY; }

    // XpressNet v4.0 0x53 format (spec 3.38):
    // in[1] = AdrMSB  \  decoder address (0-based, big-endian)
    // in[2] = AdrLSB  /
    // in[3] = 1 0 Q 0 A 0 0 P
    //   A = bit3 = activate (coil on/off)
    //   TT (Q + bits 2-1) = output within the decoder (0-3)
    //   P = bit0 = direction — per RCN-213/0x52: 0=thrown/diverging,
    //       1=straight (NOT an inverted convention relative to 0x52,
    //       as previously assumed — passed through directly, no inversion)
    // outputAddress = decoder*4 + TT + 1  (identical to 0x52)
    uint16_t decoder = ((uint16_t)in[1] << 8) | in[2];
    uint8_t  TT      = (in[3] >> 1) & 0x03;
    uint8_t  active  = (in[3] >> 3) & 0x01;   // A bit = bit3
    uint8_t  output  = in[3] & 0x01;           // P bit = bit0, direct = RCN-213 R-bit
    uint16_t clientAddr = decoder * 4 + TT + 1;

    if (clientAddr < MAX_ACCESSORIES && active) {
        if (output == 0)
            gAccessories[clientAddr].state |= 0x01;   // thrown/diverging
        else
            gAccessories[clientAddr].state &= ~0x01;  // straight
        gAccessories[clientAddr].known = true;
    }

    Command cmd{};
    cmd.type     = CmdType::ACCESSORY_SET;
    cmd.sourceId = _srcId;
    cmd.address  = clientAddr;
    cmd.output   = output;
    cmd.active   = (active != 0);
    CommandBus::instance().dispatch(cmd);

    // 0x42 feedback broadcast — Lenz format, cumulative, only on activate
    if (_broadcastFn && active) {
        uint8_t  dataByte = 0;
        for (uint8_t tt = 0; tt < 4; tt++) {
            uint16_t a = decoder * 4 + tt + 1;
            if (a < MAX_ACCESSORIES && gAccessories[a].known) {
                uint8_t out = gAccessories[a].state & 0x01;
                dataByte |= (uint8_t)(1u << (tt * 2 + out));
            }
        }
        uint8_t bc[4];
        bc[0] = 0x42; bc[1] = in[2]; bc[2] = dataByte;
        bc[3] = bc[0] ^ bc[1] ^ bc[2];
        _broadcastFn(bc, 4);
    }

    // Persist the turnout state to EEPROM on activate
    if (active) eepromStore().saveAccessories();

    l = buildOk(r); return XNHandleResult::OK_SILENT;
}

// ─────────────────────────────────────────────────────────────
// handleTurnoutInfo — LAN_X_GET_TURNOUT_INFO (0x43): looks up the
//                      known state of a single turnout output address
//                      and returns it. Unknown addresses return
//                      state=0x00 (no direct DCC pulse is generated
//                      for a query).
// ─────────────────────────────────────────────────────────────
XNHandleResult XpressNetHandler::handleTurnoutInfo(const uint8_t* in, uint8_t il,
                                                    uint8_t* r, uint8_t& l) {
    if (il < 4) { l = buildError(r, 0x82); return XNHandleResult::REPLY; }
    uint8_t  msb  = in[1];
    uint8_t  lsb  = in[2];
    uint16_t addr = ((uint16_t)msb << 8) | lsb;

    uint8_t state = 0x00;
    if (addr < MAX_ACCESSORIES && gAccessories[addr].known) {
        state = gAccessories[addr].state & 0x03;
    }
    r[0] = 0x43; r[1] = msb; r[2] = lsb; r[3] = state;
    r[4] = xorCheck(r, 4); l = 5;
    return XNHandleResult::REPLY;
}

// ─────────────────────────────────────────────────────────────
// handleSwitchQuery — 0x42 command has two variants:
//
// Variant A — Turnout status query (il==3, no nibble selector):
//   Format:   42 [addrByte] [XOR]
//   Response: 42 [addrByte] [nib0] [nib1] [XOR]
//
// Variant B — Feedback module query (il==4, with nibble selector):
//   Format:   42 [addrByte] [nibSel] [XOR]
//   nibSel:   0x80=nibble0, 0x81=nibble1
//   addrByte: protocol address (0-127, §2.15) — NOT the /4 turnout-
//             address formula (that only applies to turnouts). NOTE:
//             this is the protocol address, not the RS-Bus hardware
//             address (see rs_bus_hal.h) — those two differ by +1/-1.
//   Response: 42 [addrByte] [ITNZ] [XOR]
//             ITNZ = I(0) TT(10 if module known, else 00)
//                    N(nibble) Z(value, 0 if module unknown)
// ─────────────────────────────────────────────────────────────
XNHandleResult XpressNetHandler::handleSwitchQuery(const uint8_t* in, uint8_t il,
                                                     uint8_t* r, uint8_t& l) {
    if (il < 3) { l = buildError(r, 0x82); return XNHandleResult::REPLY; }

    // Variant B: nibble selector present
    if (il >= 4 && (in[2] == 0x80 || in[2] == 0x81)) {
        uint8_t protoAddr = in[1];                // §2.15: protocol address 0-127
        uint8_t hwAddr    = protoAddr + 1;        // RS-Bus hardware address 1-128
        uint8_t nibble    = in[2] & 0x01;         // 0x80→nibble0, 0x81→nibble1
        bool    present   = (protoAddr <= 127) && gRsBusHal.isSeen(hwAddr);
        uint8_t nibData   = present ? gRsBusHal.getNibble(hwAddr, nibble) : 0;
        uint8_t tt        = present ? gRsBusHal.getType(hwAddr) : 0;
        uint8_t itnz      = ((tt & 0x03) << 5) | (nibble ? 0x10 : 0x00)
                           | (nibData & 0x0F);
        r[0] = 0x42; r[1] = protoAddr; r[2] = itnz;
        r[3] = xorCheck(r, 3); l = 4;
        return XNHandleResult::REPLY;
    }

    // Variant A: turnout status query
    uint8_t  addrByte = in[1] & 0x3F;
    uint16_t base     = (uint16_t)addrByte * 4 + 1;

    uint8_t nib0 = 0, nib1 = 0;
    bool valid0 = false, valid1 = false;

    for (uint8_t tt = 0; tt < 4; tt++) {
        uint16_t a = base + tt;
        if (a >= MAX_ACCESSORIES || !gAccessories[a].known) continue;
        uint8_t out = gAccessories[a].state & 0x01; // 0=straight, 1=thrown
        uint8_t bitPos = (tt % 2) * 2 + out;        // bit within nibble
        if (tt < 2) { nib0 |= (1u << bitPos); valid0 = true; }
        else        { nib1 |= (1u << bitPos); valid1 = true; }
    }

    if (valid0) nib0 |= 0x80;  // bit7 = data valid
    if (valid1) nib1 |= 0x80;

    r[0] = 0x42; r[1] = addrByte; r[2] = nib0; r[3] = nib1;
    r[4] = xorCheck(r, 4); l = 5;
    return XNHandleResult::REPLY;
}

// ─────────────────────────────────────────────────────────────
// handleProgRead — starts a CV read in service/programming mode.
//                  Marks the station as "in programming mode",
//                  remembers the requested CV so handleProgResult()
//                  can report it later, dispatches a CV_READ command,
//                  and broadcasts the programming-mode state.
// ─────────────────────────────────────────────────────────────
XNHandleResult XpressNetHandler::handleProgRead(const uint8_t* in, uint8_t il,
                                                  uint8_t* r, uint8_t& l) {
    if (il < 4) { l = buildError(r, 0x82); return XNHandleResult::REPLY; }
    gCentrale.progModeActive = true;
    gCentrale.pendingCvAddr  = in[2];
    gCentrale.pendingCvWrite = false;
    Command cmd{};
    cmd.type = CmdType::CV_READ; cmd.sourceId = _srcId; cmd.cv = in[2];
    CommandBus::instance().dispatch(cmd);
    broadcastProgMode();
    l = buildOk(r); return XNHandleResult::OK_SILENT;
}

// ─────────────────────────────────────────────────────────────
// handleProgWrite — starts a CV write in service/programming mode.
//                   Same bookkeeping as handleProgRead(), but also
//                   remembers the value to write and dispatches a
//                   CV_WRITE command instead.
// ─────────────────────────────────────────────────────────────
XNHandleResult XpressNetHandler::handleProgWrite(const uint8_t* in, uint8_t il,
                                                   uint8_t* r, uint8_t& l) {
    if (il < 5) { l = buildError(r, 0x82); return XNHandleResult::REPLY; }
    gCentrale.progModeActive = true;
    gCentrale.pendingCvAddr  = in[2];
    gCentrale.pendingCvData  = in[3];
    gCentrale.pendingCvWrite = true;
    Command cmd{};
    cmd.type = CmdType::CV_WRITE; cmd.sourceId = _srcId;
    cmd.cv   = in[2]; cmd.cvValue = in[3];
    CommandBus::instance().dispatch(cmd);
    broadcastProgMode();
    l = buildOk(r); return XNHandleResult::OK_SILENT;
}

// ─────────────────────────────────────────────────────────────
// handleProgResult — reports the outcome of a previously requested
//                    CV read/write in programming mode. If the DCC
//                    layer hasn't produced a result yet, replies with
//                    a "not ready" error (0x61 0x1F); otherwise returns
//                    the CV value and clears the pending/prog-mode
//                    flags.
// ─────────────────────────────────────────────────────────────
XNHandleResult XpressNetHandler::handleProgResult(uint8_t* r, uint8_t& l) {
    if (!gCentrale.progResultReady) {
        r[0]=0x61; r[1]=0x1F; r[2]=xorCheck(r,2); l=3;
        return XNHandleResult::REPLY;
    }
    uint16_t cv  = gCentrale.pendingCvAddr;
    uint8_t  val = gCentrale.progResultData;
    gCentrale.progResultReady = false;
    gCentrale.progModeActive  = false;
    r[0]=0x63; r[1]=0x14;
    r[2]=(cv==1024)?0:(cv&0xFF); r[3]=val;
    r[4]=xorCheck(r,4); l=5;
    return XNHandleResult::REPLY;
}

// ─────────────────────────────────────────────────────────────
// handlePOM — Programming On Main: writes a CV on a specific loco
//             address while it stays on the main track (no need to
//             enter service mode). Dispatches a POM_WRITE command;
//             no direct response is expected (silent ack only).
// ─────────────────────────────────────────────────────────────
XNHandleResult XpressNetHandler::handlePOM(const uint8_t* in, uint8_t il,
                                             uint8_t* r, uint8_t& l) {
    if (il < 8) { l = buildError(r, 0x82); return XNHandleResult::REPLY; }
    uint16_t locoAddr = decodeAddr(in[2], in[3]);
    uint16_t cv = ((uint16_t)(in[4] & 0x01) << 8) | in[5];
    Command cmd{};
    cmd.type = CmdType::POM_WRITE; cmd.sourceId = _srcId;
    cmd.address = locoAddr; cmd.cv = cv; cmd.cvValue = in[6];
    CommandBus::instance().dispatch(cmd);
    l = buildOk(r); return XNHandleResult::OK_SILENT;
}

// ─────────────────────────────────────────────────────────────
// handleSetStartMode — accepts the "set start mode" command. Not
//                      currently acted upon (no station-side start
//                      mode setting implemented yet); always
//                      acknowledged.
// ─────────────────────────────────────────────────────────────
XNHandleResult XpressNetHandler::handleSetStartMode(const uint8_t* in, uint8_t il,
                                                      uint8_t* r, uint8_t& l) {
    l = buildOk(r); return XNHandleResult::OK_SILENT;
}

// ─────────────────────────────────────────────────────────────
// handleStackSearch — implements the XpressNet "stack search"
//                     feature: given a starting address, finds and
//                     returns the next active locomotive in the
//                     repository (see stackSearchNext()), so a
//                     throttle can step through the list of known
//                     locos one at a time.
// ─────────────────────────────────────────────────────────────
XNHandleResult XpressNetHandler::handleStackSearch(const uint8_t* in, uint8_t il,
                                                     uint8_t* r, uint8_t& l) {
    if (il < 5) { l = buildError(r, 0x82); return XNHandleResult::REPLY; }
    uint16_t fromAddr = decodeAddr(in[2], in[3]);
    uint16_t found    = stackSearchNext(fromAddr);
    l = buildSearchResult(r, found, found ? 0 : 4);
    return XNHandleResult::REPLY;
}

// ─────────────────────────────────────────────────────────────
// handleStackDelete — removes/releases a locomotive entry from the
//                     repository (used when a throttle explicitly
//                     drops a loco from its stack).
// ─────────────────────────────────────────────────────────────
XNHandleResult XpressNetHandler::handleStackDelete(const uint8_t* in, uint8_t il,
                                                     uint8_t* r, uint8_t& l) {
    if (il < 5) { l = buildError(r, 0x82); return XNHandleResult::REPLY; }
    locoRepo().release(decodeAddr(in[2], in[3]));
    l = buildOk(r); return XNHandleResult::OK_SILENT;
}

// ─────────────────────────────────────────────────────────────
// buildModelTimeBuf — Encodes the current fast clock state into
//                     an XpressNet-compatible data // buffer for
//                     transmission to connected clients.
//                     The "fast clock" model-time message
//                     (0x64 0x25): day-of-week, hour, minute and the
//                     current speed-up factor (0 if the clock is
//                     stopped).
// ─────────────────────────────────────────────────────────────
uint8_t XpressNetHandler::buildModelTimeBuf(uint8_t* r) {
    r[0]=0x64; r[1]=0x25;
    r[2]=(gCentrale.mtDow<<5)|gCentrale.mtHour;
    r[3]=gCentrale.mtMinute;
    r[4]=gCentrale.mtRunning ? gCentrale.mtFactor : 0;
    r[5]=xorCheck(r,5); return 6;
}

// ─────────────────────────────────────────────────────────────
// handleModelTimeGet — Processes a request for the current fast
//                      clock time and returns the model railroad
//                      clock information in XpressNet format.
// ─────────────────────────────────────────────────────────────
XNHandleResult XpressNetHandler::handleModelTimeGet(uint8_t* r, uint8_t& l) {
    l = buildModelTimeBuf(r); return XNHandleResult::REPLY;
}

// handleModelTimeBc — broadcasts the current model time to all clients.
XNHandleResult XpressNetHandler::handleModelTimeBc(uint8_t* r, uint8_t& l) {
    // Broadcast the current model time to all clients
    l = buildModelTimeBuf(r);
    if (_broadcastFn) _broadcastFn(r, l);
    l = 0; return XNHandleResult::BROADCAST_ONLY;
}

// ─────────────────────────────────────────────────────────────
// handleModelTimeSet — sets the model (fast clock) day-of-week,
//                      hour, minute and speed-up factor from the
//                      client, then confirms by broadcasting the
//                      new model time to all clients.
// ─────────────────────────────────────────────────────────────
XNHandleResult XpressNetHandler::handleModelTimeSet(const uint8_t* in, uint8_t il,
                                                      uint8_t* r, uint8_t& l) {
    if (il < 6) { l = buildError(r, 0x82); return XNHandleResult::REPLY; }
    gCentrale.mtDow     = (in[2] >> 5) & 0x07;
    gCentrale.mtHour    =  in[2]       & 0x1F;
    gCentrale.mtMinute  =  in[3]       & 0x3F;
    gCentrale.mtFactor  =  in[4]       & 0x1F;
    gCentrale.mtRunning = (in[4] > 0);
    // Confirm with a broadcast of the current model time
    uint8_t bc[8]; uint8_t bcLen = buildModelTimeBuf(bc);
    if (_broadcastFn) _broadcastFn(bc, bcLen);
    l = buildOk(r); return XNHandleResult::OK_SILENT;
}
// ─────────────────────────────────────────────────────────────
// handleFastClockStart — Starts fast clock operation and enables
//                        model time progression (mtRunning=true) and
//                        broadcasts the current status to all clients.
// ─────────────────────────────────────────────────────────────
XNHandleResult XpressNetHandler::handleFastClockStart(uint8_t* r, uint8_t& l) {
    // Start the model clock — send a broadcast with the current status
    gCentrale.mtRunning = true;
    uint8_t bc[8]; uint8_t bcLen = buildModelTimeBuf(bc);
    if (_broadcastFn) _broadcastFn(bc, bcLen);
    l = buildOk(r); return XNHandleResult::OK_SILENT;
}

// ─────────────────────────────────────────────────────────────
// handleFastClockStop — Stops fast clock progression while
//                       preserving the current model time.
//                       (speed-up factor becomes 0 in the broadcast)
//                       and broadcasts the updated status to all clients.
// ─────────────────────────────────────────────────────────────
XNHandleResult XpressNetHandler::handleFastClockStop(uint8_t* r, uint8_t& l) {
    // Stop the model clock — set the factor to 0 and send a broadcast
    gCentrale.mtRunning = false;
    uint8_t bc[8]; uint8_t bcLen = buildModelTimeBuf(bc);
    if (_broadcastFn) _broadcastFn(bc, bcLen);
    l = buildOk(r); return XNHandleResult::OK_SILENT;
}

// ─────────────────────────────────────────────────────────────
// handleReset — handles a client-requested command station reset:
//               clears emergency-stop, forces track power off,
//               leaves programming mode, broadcasts power-off, logs
//               the reset, and sets a flag so core0 stops feeding the
//               watchdog (letting the watchdog trigger an actual
//               hardware reset shortly after).
// ─────────────────────────────────────────────────────────────
XNHandleResult XpressNetHandler::handleReset(uint8_t* r, uint8_t& l) {
    gCentrale.emergencyStop  = false;
    gCentrale.trackPowerOff  = true;
    gCentrale.progModeActive = false;
    broadcastPowerOff();
    Serial2.println("RESET: command station restarting...");
    Serial2.flush();
    // Set the flag — core0 stops watchdog_update() and lets the watchdog trigger
    extern volatile bool gResetRequested;
    gResetRequested = true;
    l = 0; return XNHandleResult::BROADCAST_ONLY;
}

// ─────────────────────────────────────────────────────────────
// handleSVRead — reads one "System Variable" (SV) byte from the
//                command station's own configuration (e.g. SV1 =
//                device address). svAddr==0 is treated as a special
//                index (255) reserved for a specific SV slot.
// ─────────────────────────────────────────────────────────────
XNHandleResult XpressNetHandler::handleSVRead(const uint8_t* in, uint8_t il,
                                                uint8_t* r, uint8_t& l) {
    if (il < 4) { l = buildError(r, 0x82); return XNHandleResult::REPLY; }
    uint8_t svAddr = in[2];
    uint8_t idx    = (svAddr == 0) ? 255 : svAddr - 1;
    r[0]=0x63; r[1]=0x20; r[2]=svAddr; r[3]=gCentrale.sv[idx];
    r[4]=xorCheck(r,4); l=5; return XNHandleResult::REPLY;
}

// ─────────────────────────────────────────────────────────────
// handleSVWrite — writes one System Variable byte. SV1 (device
//                 address) is range-checked (1-31) and reset to 1
//                 if an invalid value was supplied.
// ─────────────────────────────────────────────────────────────
XNHandleResult XpressNetHandler::handleSVWrite(const uint8_t* in, uint8_t il,
                                                 uint8_t* r, uint8_t& l) {
    if (il < 5) { l = buildError(r, 0x82); return XNHandleResult::REPLY; }
    uint8_t svAddr = in[2];
    uint8_t idx    = (svAddr == 0) ? 255 : svAddr - 1;
    gCentrale.sv[idx] = in[3];
    // SV1 = device address — validate range
    if (svAddr == 1 && (in[3] == 0 || in[3] > 31))
        gCentrale.sv[0] = 1;  // reset to 1 if invalid
    r[0]=0x63; r[1]=0x20; r[2]=svAddr; r[3]=in[3];
    r[4]=xorCheck(r,4); l=5; return XNHandleResult::REPLY;
}

// ─────────────────────────────────────────────────────────────
//  process() — main dispatcher
//
//  Entry point for every incoming XpressNet message, called by each
//  interface module (LenzLan, LenzUsb, XpressNetRs485, Z21Lan) with
//  the raw message bytes (excluding any transport-specific framing).
//  Validates length and checksum, then routes the message to the
//  appropriate command handler based on its header/id bytes, and
//  returns the result (a direct reply, an error, a silent
//  acknowledgement, or "broadcast only") together with any reply
//  bytes to send back.
// ─────────────────────────────────────────────────────────────
XNHandleResult XpressNetHandler::process(const uint8_t* in, uint8_t il,
                                          uint8_t* r, uint8_t& l) {
    l = 0;
    if (il < 2)              { l = buildError(r, 0x82); return XNHandleResult::ERROR; }
    if (!verifyXor(in, il))  { l = buildError(r, 0x80); return XNHandleResult::REPLY; }

    uint8_t h  = in[0];
    uint8_t id = in[1];

    // Interface info requests (0xF0, 0xF1, 0xF2)
    if (h==0xF0)              return handleIfVersion    (r, l);
    if (h==0xF1 && id==0x01) return handleIfStatus     (r, l);
    if (h==0xF1 && id==0x02) return handleXnVersion    (r, l);
    //if (h==0xF1 && id==0x03) return handleExtraVersion (r, l);
    if (h==0xF1 && id==0x03) return handleNumConn      (r, l);
    if (h==0xF2 && id==0x01) return handleDeviceAddr   (r, l);

    // Power / emergency stop
    if (h==0x21 && id==0x81) return handlePowerOn      (r, l);
    if (h==0x21 && id==0x80) return handlePowerOff     (r, l);
    if (h==0x80)              return handleEmergStop    (r, l);
    if (h==0x92)              return handleOneLocoStop  (in, il, r, l);

    // Command station info
    if (h==0x21 && id==0x21) return handleGetVersion   (r, l);
    if (h==0x21 && id==0x23) return handleExtraVersion (r, l);
    if (h==0x21 && id==0x24) return handleGetStatus    (r, l);
    if (h==0x21 && id==0x10) return handleProgResult   (r, l);
    if (h==0x21 && id==0x2A) return handleModelTimeGet (r, l);
    if (h==0x21 && id==0x2C) return handleFastClockStart(r, l);
    if (h==0x21 && id==0x2D) return handleFastClockStop (r, l);
    if (h==0x21 && id==0x28) return handleReset        (r, l);
    if (h==0x22 && id==0x22) return handleSetStartMode (in, il, r, l);
    if (h==0x22 && id==0x25) return handleSVRead       (in, il, r, l);
    if (h==0x23 && id==0x26) return handleSVWrite      (in, il, r, l);
    if (h==0x24 && id==0x2B) return handleModelTimeSet (in, il, r, l);

    // Programming (service mode CV read/write)
    if (h==0x22 && (id==0x11||id==0x14||id==0x15||id==0x18||
                    id==0x19||id==0x1A||id==0x1B))
        return handleProgRead (in, il, r, l);
    if (h==0x23 && (id==0x12||id==0x16||id==0x17||id==0x1C||
                    id==0x1D||id==0x1E||id==0x1F))
        return handleProgWrite(in, il, r, l);

    // Loco drive commands
    if (h==0xE4) {
        switch (id) {
            case 0x10: case 0x11: case 0x12: case 0x13:
                return handleLocoSpeed (in, il, r, l);
            case 0x20: case 0x21: case 0x22: case 0x23: case 0x28:
                return handleLocoFunc  (in, il, r, l);
            case 0xF8:
                return handleLocoFuncF8(in, il, r, l);
            case 0x24: case 0x25: case 0x26: case 0x27: case 0x2C:
            case 0x2D: case 0x2E: case 0x52: case 0x53: case 0x54:
                return handleFuncStatusSet(in, il, r, l);
        }
    }
    if (h==0xE5 && id==0x5F) return handleLocoBinaryState(in, il, r, l);

    // Loco queries
    if (h==0xE3) {
        switch (id) {
            case 0x00: return handleLocoQuery    (in, il, r, l);
            case 0x07: case 0x08: case 0x09: case 0x0A: case 0x0B:
                       return handleFuncStatus   (in, il, r, l);
            case 0x05: case 0x06:
                       return handleStackSearch  (in, il, r, l);
            case 0x44: return handleStackDelete  (in, il, r, l);
            case 0xF0: return handleLocoQueryMM  (in, il, r, l); // MultiMaus
        }
    }

    // Turnouts / accessories
    if (h==0x42) return handleSwitchQuery (in, il, r, l);
    if (h==0x43) return handleTurnoutInfo (in, il, r, l);
    if (h==0x52) return handleTurnout     (in, il, r, l);
    if (h==0x53) return handleTurnoutZ21  (in, il, r, l);  // Z21 LAN_X_SET_TURNOUT

    // POM (Programming On Main)
    if (h==0xE6 && id==0x30) return handlePOM (in, il, r, l);

    // Unknown — acknowledge per spec §1.5
    l = buildOk(r); return XNHandleResult::OK_SILENT;
}
