// ═══════════════════════════════════════════════════════════════
//  lenz_lan.cpp  —  Lenz LAN TCP server implementation
//
//  Implements the Lenz LI-USB-Ethernet protocol (LAN v3.6) via TCP
//  on port 5550. Up to 8 PC clients simultaneously.
//
//  Framing structure (PC ↔ station):
//    PC → station:  FF FE [XN bytes] [XOR]   (command)
//    station → PC:  FF FE [XN bytes] [XOR]   (reply to a command)
//    station → PC:  FF FD [XN bytes] [XOR]   (broadcast to all clients)
//
//  Supported messages:
//    F1 01 F0       Keep-alive request from PC
//    21 80          NotAus (emergency stop)
//    21 81          Normal operation
//    21 21          Query version
//    21 24          Query status
//    52 ...         XpressNet turnout command
//    42 ...         XpressNet switch query
//    63 ...         XpressNet version response
//
//  On new connection:
//    1. Welcome: FF FD 61 01 60 (normal-operation / "all on" broadcast)
//    2. RS-Bus dump: all present modules as BC "Rückmeldung" packets
//       (header 0x40+N, §2.4.5 — N = number of data bytes in that packet)
//
//  Dependencies: XpressNetHandler (shared parser), RsBusHal
// ═══════════════════════════════════════════════════════════════

#include "lenz_lan.h"
#include "trace_log.h"
#include <utility/w5100.h>   // For W5100.readSnMR/writeSnMR (No Delay ACK)
#include "rs_bus_hal.h"
#include "led_status.h"

static XpressNetHandler sXpressNet;  // Shared XpressNet parser for this server

LenzLan gLenzLan;  // Global instance, registered with the ModuleRegistry

// ─────────────────────────────────────────────────────────────
//  _freeConnections() — actual number of free W5100/W5500 sockets
//
//  Answers the "Verfügbare Freie Verbindungen" (0xF1/0x03) query by
//  reading the real hardware socket status instead of relying on an
//  assumption/fixed value. Iterates over every hardware socket
//  (0..MAX_SOCK_NUM-1) and asks the Ethernet chip driver for that
//  socket's status register (Sn_SR): a socket with status CLOSED is
//  free; any other status (LISTEN, ESTABLISHED, ...) means it is
//  currently in use by ONE of the modules sharing the Ethernet chip
//  (LenzLan itself, Z21Lan/UDP, the Webserver, mDNS/Bonjour).
//
//  NOTE: this uses the standard Arduino Ethernet library (not
//  Ethernet3) — the underlying chip object is historically still
//  called "W5100" there even though a W5500 sits on the board. Verify
//  this against the exact installed library version (e.g. via
//  utility/w5100.h / utility/socket.h) and adjust if needed.
// ─────────────────────────────────────────────────────────────
uint8_t LenzLan::_freeConnections() {
    uint8_t free = 0;
    for (uint8_t s = 0; s < MAX_SOCK_NUM; s++) {
        if (W5100.readSnSR(s) == SnSR::CLOSED) free++;
    }
    return free;
}

// ─────────────────────────────────────────────────────────────
//  begin() — initialise the TCP server and the XpressNet handler
//
//  Logs the current Ethernet link status and IP address, then starts
//  the TCP server listening on the configured port (EEPROM key
//  "lenz.port", web interface, default 5550) and resets every client
//  slot to its inactive/empty state (rxLen=0, progMode=false,
//  lastActMs=0, active=false), along with the overall connection
//  counter.
//
//  Initialises the shared XpressNetHandler instance for this server
//  (sXpressNet) and registers a broadcast callback on it: rather than
//  transmitting broadcasts immediately, the callback simply hands the
//  bytes to this module's XnInterfaceLayer (_ifLayer.bufferBroadcast),
//  which buffers them until the caller has had a chance to send its
//  own direct response first (see _processFrame() and
//  xn_interface_layer.h for why this ordering matters — the 01/04/05
//  acknowledgement must reach the client before any resulting
//  broadcast, e.g. turnout info after a turnout command). Note that
//  EventBus-originated broadcasts (power state, RS-Bus feedback) take
//  a completely separate path (onEvent() → _sendBroadcast() directly)
//  and are not affected by this buffering.
//
//  Finally registers this module's onEvent() method as the EventBus
//  handler for ModuleId::LENZ_LAN, so that events published by other
//  modules (running on core0, e.g. power changes or RS-Bus feedback)
//  reach this LAN server.
// ─────────────────────────────────────────────────────────────
void LenzLan::begin() {
    traceSerial.print("LenzLan begin: Ethernet link=");
    traceSerial.print(Ethernet.linkStatus());
    traceSerial.print(" IP="); traceSerial.println(Ethernet.localIP());

    uint16_t port = eepromStore().getUint16("lenz.port", 5550);
    _server = new EthernetServer(port);
    _server->begin();
    for (uint8_t i = 0; i < LENZ_LAN_MAX_CLIENTS; i++) {
        _clients[i].rxLen     = 0;
        _clients[i].progMode  = false;
        _clients[i].lastActMs = 0;
        _clients[i].active    = false;
    }
    _connCount = 0;

    traceSerial.printf("LenzLan: server gestart op poort %u\n", port);

    // Broadcast callback: the XpressNet handler calls this
    // synchronously from within command processing (e.g. turnout info
    // after a turnout command). We do NOT send this immediately, but
    // buffer it via the interface layer, so the caller (_processFrame)
    // can send its own direct response first (01/04/05 or REPLY/ERROR)
    // — see xn_interface_layer.h for the reason. EventBus events
    // (power, RS-Bus) take a separate path (onEvent() → _sendBroadcast
    // directly) and are not affected by this.
    sXpressNet.begin(ModuleId::LENZ_LAN);
    sXpressNet.setBroadcastCallback([](const uint8_t* data, uint8_t dlen, uint16_t /*locoAddr*/) {
        gLenzLan._ifLayer.bufferBroadcast(data, dlen);
    });

    // EventBus: receives RS-Bus feedback and turnout state events from core0
    EventBus::instance().registerHandler(ModuleId::LENZ_LAN,
        [](const Event& ev){ gLenzLan.onEvent(ev); });
}

// ─────────────────────────────────────────────────────────────
//  loop() — main poll loop (core1)
//
//  Called repeatedly from the core1 loop. Each call:
//    1. Accepts one new incoming TCP connection, if any (_acceptNew()).
//    2. Processes incoming data from every currently active client
//       (_processClient() for each), with _txBusy set for the
//       duration so that onEvent() cannot interleave an RS-Bus
//       broadcast with a response packet that is only partially
//       written to a client's TCP socket — any feedback events that
//       arrive during this window are queued instead (see
//       _queueFeedback()/_flushPendingFeedback()).
//    3. Sends out any RS-Bus feedback packets that were queued while
//       _txBusy was set (_flushPendingFeedback()).
//    4. Disconnects any client that has been inactive for too long
//       (_checkTimeouts()).
// ─────────────────────────────────────────────────────────────
void LenzLan::loop() {
    _acceptNew();

    // Process clients. Set _txBusy to prevent onEvent() from interleaving
    // RS-Bus broadcasts with response packets mid-transmission.
    _txBusy = true;
    for (uint8_t i = 0; i < LENZ_LAN_MAX_CLIENTS; i++) {
        if (_clients[i].active)
            _processClient(_clients[i]);
    }
    _txBusy = false;

    // Send any RS-Bus packets that were queued during client processing
    _flushPendingFeedback();

    _checkTimeouts();
}

// ─────────────────────────────────────────────────────────────
//  _acceptNew() — accept a new TCP connection
//
//  Checks whether a new client is waiting on the server socket; if
//  not, returns immediately. Otherwise looks for a free (inactive)
//  client slot; if none is free, the new connection is refused
//  (stopped) right away, since LENZ_LAN_MAX_CLIENTS is already
//  reached.
//
//  On success, stores the new EthernetClient in the slot and marks it
//  active, resets its receive state, records the current time as its
//  last-activity timestamp, and increments the connection counter.
//  Also sets the "No Delay ACK" (ND) bit in the W5500's per-socket
//  mode register (Sn_MR), which disables the Nagle algorithm for this
//  socket so outgoing packets are sent immediately instead of being
//  held back waiting for a TCP ACK — this reduces typical response
//  latency from roughly 10ms down to about 1ms.
//
//  Sends the initial "welcome" broadcast reflecting the command
//  station's actual current state (track power off / emergency stop
//  / normal operation), so the new client immediately knows where
//  things stand without having to ask.
//
//  Finally sends an initial RS-Bus "dump": one or more BC
//  "Rückmeldung" packets (per §2.4.5/§2.15) covering every feedback
//  module address that RsBusHal currently knows about (isSeen()==true),
//  including both nibbles for each, batched up to 3 address/data pairs
//  per packet. This lets the new client immediately know the current
//  state of every known feedback module, rather than waiting for the
//  next change to be reported. Note that RsBusHal counts RS-Bus
//  hardware addresses starting at 1 (1-128), while the protocol counts
//  from 0 (0-127) — hence the "-1" applied to addrByte below.
// ─────────────────────────────────────────────────────────────
void LenzLan::_acceptNew() {
    EthernetClient newClient = _server->accept();
    if (!newClient) return;

    traceSerial.print("LenzLan: nieuwe client van ");
    traceSerial.println(newClient.remoteIP());

    int8_t slot = -1;
    for (uint8_t i = 0; i < LENZ_LAN_MAX_CLIENTS; i++) {
        if (!_clients[i].active) { slot = i; break; }
    }

    if (slot < 0) { newClient.stop(); return; }

    _clients[slot].tcp       = newClient;
    _clients[slot].rxLen     = 0;
    _clients[slot].progMode  = false;
    _clients[slot].lastActMs = millis();
    _clients[slot].active    = true;
    _connCount++;
    // Set No Delay ACK (ND) bit in W5500 socket mode register Sn_MR (bit 5).
    // Disables Nagle algorithm — packets are sent immediately without waiting
    // for a TCP ACK, reducing response latency from ~10ms to ~1ms.
    {
        uint8_t sn = _clients[slot].tcp.getSocketNumber();
        W5100.writeSnMR(sn, W5100.readSnMR(sn) | 0x20);  // 0x20 = ND bit
    }


    // Send welcome broadcast reflecting actual track power state
    if (gCentrale.trackPowerOff) {
        uint8_t welcome[] = { 0xFF, 0xFD, 0x61, 0x00, 0x61 };  // Track power off
        _sendRaw(_clients[slot], welcome, sizeof(welcome));
    } else if (gCentrale.emergencyStop) {
        uint8_t welcome[] = { 0xFF, 0xFD, 0x81, 0x00, 0x81 };  // Emergency stop
        _sendRaw(_clients[slot], welcome, sizeof(welcome));
    } else {
        uint8_t welcome[] = { 0xFF, 0xFD, 0x61, 0x01, 0x60 };  // Normal operation
        _sendRaw(_clients[slot], welcome, sizeof(welcome));
    }

    // RS-Bus dump: send only present modules as BC "Rückmeldung" packets
    // Format (§2.4.5 → ADR/DAT format per §2.15):
    //   FF FD [0x40+N] [ADR ITNZ]{1..8} XOR
    //   N    = number of data bytes that follow (2 per pair, max 16 = 8 pairs)
    //   ADR  = protocol address (0-127), no nibble bit packed into it
    //   ITNZ = I(0) TT(10=Rückmeldebaustein) N(nibble) ZZZZ(4-bit value)
    //
    // NOTE: gRsBusHal internally counts in RS-Bus hardware addresses (1-128);
    // the protocol counts 0-127 (see rs_bus_hal.h). Hence the -1 on addrByte.
    uint8_t pkt[10];
    pkt[0] = 0xFF; pkt[1] = 0xFD;
    uint8_t pairCount = 0;

    for (uint8_t hwAddr = 1; hwAddr <= 128; hwAddr++) {
        if (!gRsBusHal.isSeen(hwAddr)) continue;  // module not present — skip
        for (uint8_t nibble = 0; nibble <= 1; nibble++) {
            uint8_t data = gRsBusHal.getNibble(hwAddr, nibble);
            // Send all nibbles for present modules, even if value is 0
            uint8_t addrByte = (hwAddr - 1) & 0x7F;
            uint8_t itnz = ((gRsBusHal.getType(hwAddr) & 0x03) << 5)
                         | (nibble ? 0x10 : 0x00)
                         | (data & 0x0F);
            pkt[3 + pairCount * 2]     = addrByte;
            pkt[3 + pairCount * 2 + 1] = itnz;
            pairCount++;

            if (pairCount == 3) {
                uint8_t kennung = 0x40 + pairCount * 2;  // §2.4.5: header = 0x40+N
                pkt[2] = kennung;
                uint8_t xorVal = kennung;
                for (uint8_t b = 0; b < pairCount * 2; b++) xorVal ^= pkt[3 + b];
                pkt[9] = xorVal;
                _sendRaw(_clients[slot], pkt, 10);
                pairCount = 0;
            }
        }
    }
    // Send remaining pairs as a smaller packet
    if (pairCount > 0) {
        uint8_t kennung = 0x40 + pairCount * 2;
        pkt[2] = kennung;
        uint8_t xorVal = kennung;
        for (uint8_t b = 0; b < pairCount * 2; b++) xorVal ^= pkt[3 + b];
        pkt[3 + pairCount * 2] = xorVal;
        _sendRaw(_clients[slot], pkt, 3 + pairCount * 2 + 1);
    }
}

// ─────────────────────────────────────────────────────────────
//  _processClient() — process incoming data from one client
//
//  If the underlying TCP connection has been closed by the peer (and
//  there is no more buffered data left to read), tears down this
//  client slot: stops the socket, marks it inactive, clears
//  programming mode, and decrements the connection counter.
//
//  Otherwise, reads every currently available byte from the socket
//  into the client's receive buffer (rxBuf), up to its capacity.
//
//  If the buffer is completely full AND the client still has more
//  data waiting to be read, this is a genuine buffer-overflow
//  condition (spec "01/06/07"): logs it, builds and sends the
//  standard buffer-overflow error response, resets the receive state,
//  and discards whatever data is still waiting on the socket for this
//  cycle, so the client and station can resynchronise on the next
//  frame boundary.
//
//  Otherwise, scans the buffer for complete frames: skips forward
//  past any stray bytes that are not the expected LAN header byte,
//  and for each byte that does match the header, asks _frameLen() how
//  long the frame starting there would be. A return value of 0 means
//  "not enough bytes yet — wait for more data", so the scan stops for
//  this call. A return value of 0xFF means the byte was not actually
//  a valid frame start after all, so the scan simply advances by one
//  byte and keeps looking. Otherwise the frame is complete: it is
//  handed to _processFrame() (with the two LAN header bytes already
//  stripped off), the client's last-activity timestamp is refreshed,
//  and the scan continues right after this frame.
//
//  Once scanning stops, any bytes that were consumed are removed from
//  the front of the buffer (the remaining, not-yet-complete bytes are
//  shifted down to offset 0) so the next call starts cleanly.
//
//  Finally, implements the "01/01/00" incomplete-frame timeout from
//  the interface spec: if the buffer is now empty, there is no
//  partial frame in progress, so the timer is cleared. Otherwise, if
//  this is the first byte of a new partial frame, the current time is
//  recorded as its start; if a partial frame has now been sitting
//  incomplete for longer than LENZ_LAN_FRAME_TIMEOUT_MS, the frame is
//  considered abandoned: the timeout error response is sent, and the
//  receive state is reset so the next attempt starts fresh.
// ─────────────────────────────────────────────────────────────
void LenzLan::_processClient(LenzClient& c) {
    if (!c.tcp.connected() && !c.tcp.available()) {
        traceSerial.println("LenzLan: client verbroken");
        c.tcp.stop();
        c.active   = false;
        c.progMode = false;
        _connCount--;
        return;
    }

    while (c.tcp.available() && c.rxLen < LENZ_LAN_RX_BUF)
        c.rxBuf[c.rxLen++] = c.tcp.read();

    // Buffer overflow: buffer full but the client still has more to send.
    if (c.rxLen >= LENZ_LAN_RX_BUF && c.tcp.available()) {
        traceSerial.println("LenzLan: buffer-overflow, herstel...");
        uint8_t err[3];
        uint8_t errLen = XnInterfaceLayer::buildBufferOverflowError(err);
        XnFrame f; memcpy(f.data, err, errLen); f.len = errLen; f.isBroadcast = false;
        _sendFrame(c, f);
        c.rxLen = 0;
        c.frameStartMs = 0;
        while (c.tcp.available()) c.tcp.read();  // discard the rest of this transmission
        return;
    }

    uint8_t offset = 0;
    while (offset < c.rxLen) {
        if (c.rxBuf[offset] != LENZ_LAN_HDR1) { offset++; continue; }

        uint8_t total = _frameLen(c.rxBuf + offset, c.rxLen - offset);
        if (total == 0)    break;         // wait for more bytes
        if (total == 0xFF) { offset++; continue; }  // invalid, skip

        _processFrame(c, c.rxBuf + offset + 2, total - 2);
        c.lastActMs = millis();
        offset += total;
    }

    if (offset > 0 && offset <= c.rxLen) {
        c.rxLen -= offset;
        memmove(c.rxBuf, c.rxBuf + offset, c.rxLen);
    }

    // Timeout on an incomplete frame (spec §1.5, "01/01/00"): a client
    // that starts sending but does not finish within ~250ms.
    if (c.rxLen == 0) {
        c.frameStartMs = 0;
    } else {
        if (c.frameStartMs == 0) c.frameStartMs = millis();
        else if (millis() - c.frameStartMs > LENZ_LAN_FRAME_TIMEOUT_MS) {
            traceSerial.println("LenzLan: onvolledig frame timeout");
            uint8_t err[3];
            uint8_t errLen = XnInterfaceLayer::buildFrameTimeoutError(err);
            XnFrame f; memcpy(f.data, err, errLen); f.len = errLen; f.isBroadcast = false;
            _sendFrame(c, f);
            c.rxLen = 0;
            c.frameStartMs = 0;
        }
    }
    // Flushing now happens per-message in _sendFrame(), not here
}

// ─────────────────────────────────────────────────────────────
//  _processFrame() — handle one complete frame
//
//  Interface-local commands (keep-alive, version, number of
//  connections, ...) are intercepted by XnInterfaceLayer. All other
//  messages go to the shared XpressNetHandler.
//
//  Passes the frame to _ifLayer.process(), which either answers an
//  interface-local command directly, or forwards it to the shared
//  handler (sXpressNet) and returns whatever result that produces.
//
//  Ordering for a "silent" command (OK_SILENT, e.g. a turnout or
//  power command): first send the 01/04/05 acknowledgement (this
//  confirms that the command has been passed on to the command
//  station — something the interface knows immediately, independent
//  of whatever the command station itself goes on to do with it), and
//  only AFTERWARDS send any broadcast that the command-station logic
//  has meanwhile buffered (e.g. the resulting turnout info). This
//  matches the ordering used by the real Lenz interface.
//
//  For REPLY/ERROR results with a non-empty response, the response
//  bytes are sent directly as a non-broadcast frame. For OK_SILENT,
//  the fixed 01/04/05 acknowledgement is built and sent instead (the
//  response the handler itself may have built for this case is not
//  used — the interface layer's own acknowledgement takes its place).
//  In both cases, and also for any other result, any broadcast that
//  ended up buffered in _ifLayer during this call is then retrieved
//  and sent to all clients via _sendBroadcast().
// ─────────────────────────────────────────────────────────────
void LenzLan::_processFrame(LenzClient& c, const uint8_t* buf, uint8_t len) {
    // Generic RX log — every frame reaching this read handler, before
    // XnInterfaceLayer decides whether it's interface-local or should
    // be forwarded to the shared XpressNetHandler. The corresponding
    // "-> XN" handoff log sits at the exact point that decision
    // resolves to "forward" — see XnInterfaceLayer::process() in
    // xn_interface_layer.cpp, since that's shared by both LenzLan and
    // LenzUsb and is a more precise point than logging here a second
    // time (Rob).
    traceLog().logBytes("XN-LAN", "RX", buf, len);
    uint8_t  resp[24];
    uint8_t  respLen = 0;
    XNHandleResult r = _ifLayer.process(sXpressNet, buf, len, resp, respLen,
                                          &LenzLan::_freeConnections);

    if ((r == XNHandleResult::REPLY || r == XNHandleResult::ERROR) && respLen > 0) {
        XnFrame f;
        memcpy(f.data, resp, respLen);
        f.len = respLen; f.isBroadcast = false;
        traceLog().logBytes("XN-LAN", "TX", f.data, f.len);
        _sendFrame(c, f);
    } else if (r == XNHandleResult::OK_SILENT) {
        uint8_t ack[3];
        uint8_t ackLen = XnInterfaceLayer::buildCommandSentAck(ack);
        XnFrame f;
        memcpy(f.data, ack, ackLen);
        f.len = ackLen; f.isBroadcast = false;
        traceLog().logBytes("XN-LAN", "TX", f.data, f.len);
        _sendFrame(c, f);
    }

    // Send any buffered broadcast (e.g. turnout info) only NOW —
    // after the direct response/ack above.
    if (_ifLayer.hasPendingBroadcast()) {
        uint8_t bc[8];
        uint8_t bcLen = _ifLayer.takePendingBroadcast(bc);
        XnFrame f;
        memcpy(f.data, bc, bcLen);
        f.len = bcLen; f.isBroadcast = true;
        traceLog().logBytes("XN-LAN", "BC", f.data, f.len);
        _sendBroadcast(f);
    }
}

// ─────────────────────────────────────────────────────────────
//  _sendRaw() — low-level primitive used by every send path
//
//  Writes the given bytes to the client's TCP socket and flushes
//  immediately afterwards: one call to _sendRaw() results in exactly
//  one TCP segment. This matters because some clients (including a
//  rudimentary VB6 implementation encountered in testing) expect
//  precisely one Lenz message per TCP segment — if multiple messages
//  end up bundled together in a single segment, such a client quickly
//  loses sync entirely (the same class of bug found earlier with the
//  RS-Bus 0x46 broadcasts). Also pulses the Ethernet-traffic status
//  LED. Does nothing if the client slot is inactive or the length is
//  zero.
// ─────────────────────────────────────────────────────────────
void LenzLan::_sendRaw(LenzClient& c, const uint8_t* buf, uint8_t len) {
    if (!c.active || len == 0) return;
    c.tcp.write(buf, len);
    c.tcp.flush();
    gLeds.onEthernetTraffic();
}

// ─────────────────────────────────────────────────────────────
//  _sendFrame() — send one frame to one client
//
//  Prepends the 2-byte LAN header (FF FE or FF FD) to the XpressNet
//  payload bytes and sends the whole thing via _sendRaw():
//    FF FE = direct reply to this command
//    FF FD = broadcast (intended for all clients)
//  Does nothing if the frame is empty.
// ─────────────────────────────────────────────────────────────
void LenzLan::_sendFrame(LenzClient& c, const XnFrame& f) {
    if (f.len == 0) return;
    uint8_t buf[36];
    buf[0] = LENZ_LAN_HDR1;
    buf[1] = f.isBroadcast ? LENZ_LAN_HDR_BC : LENZ_LAN_HDR_CMD;
    memcpy(buf + 2, f.data, f.len);
    _sendRaw(c, buf, 2 + f.len);
}

// ─────────────────────────────────────────────────────────────
//  _sendBroadcast() — send a broadcast to ALL active clients
//
//  Used for power events (61 01/00), loco state updates and 0x42
//  turnout feedback. Every active client gets its own _sendFrame()
//  call, so each client's data is flushed as its own separate TCP
//  segment.
// ─────────────────────────────────────────────────────────────
void LenzLan::_sendBroadcast(const XnFrame& f) {
    for (uint8_t i = 0; i < LENZ_LAN_MAX_CLIENTS; i++)
        if (_clients[i].active) _sendFrame(_clients[i], f);
}

// ─────────────────────────────────────────────────────────────
//  onEvent() — handle events from the EventBus (core0 → core1)
//
//  POWER_ON/POWER_OFF: builds the fixed 3-byte "track power" frame
//    and sends it as a broadcast to every active client immediately.
//  FEEDBACK_BULK: an RS-Bus bulk update (several modules at once).
//    If a client is currently mid-processing (_txBusy), the pairs are
//    queued instead of sent right away, to avoid interleaving this
//    broadcast with a response packet that is only partially written;
//    otherwise they are sent immediately as FF FD 46 packets.
//  FEEDBACK: a single RS-Bus update (from another module). Builds one
//    address/data pair and either queues or sends it immediately,
//    following the same _txBusy logic as FEEDBACK_BULK.
// ─────────────────────────────────────────────────────────────
void LenzLan::onEvent(const Event& ev) {
    // Power events — send a 0x61 broadcast to all LAN clients
    if (ev.type == EvType::POWER_ON) {
        static const uint8_t powerOn[]  = { 0x61, 0x01, 0x60 };
        XnFrame f; memcpy(f.data, powerOn, 3); f.len = 3; f.isBroadcast = true;
        _sendBroadcast(f);
        return;
    }
    if (ev.type == EvType::POWER_OFF) {
        static const uint8_t powerOff[] = { 0x61, 0x00, 0x61 };
        XnFrame f; memcpy(f.data, powerOff, 3); f.len = 3; f.isBroadcast = true;
        _sendBroadcast(f);
        return;
    }
    if (ev.type == EvType::FEEDBACK_BULK) {
        if (_txBusy) {
            // Queue for later — sending during client processing causes interleaving
            _queueFeedback(ev.fbBulkAddr, ev.fbBulkData, ev.fbBulkCount);
        } else {
            _sendFeedbackBulk(ev.fbBulkAddr, ev.fbBulkData, ev.fbBulkCount);
        }
        return;
    }
    if (ev.type == EvType::FEEDBACK) {
        uint8_t addrByte = ev.fbModule & 0x7F;
        uint8_t dataByte = 0x40 | (ev.fbNibble ? 0x10 : 0x00) | (ev.fbDat & 0x0F);
        if (_txBusy) {
            _queueFeedback(&addrByte, &dataByte, 1);
        } else {
            _sendFeedbackBulk(&addrByte, &dataByte, 1);
        }
        return;
    }
}

// ─────────────────────────────────────────────────────────────
//  _queueFeedback() — buffer RS-Bus pairs for deferred sending
//
//  Called from onEvent() when _txBusy is set. Appends addr/data
//  pairs to the queue. Excess pairs are dropped if queue is full.
// ─────────────────────────────────────────────────────────────
void LenzLan::_queueFeedback(const uint8_t* addr, const uint8_t* data,
                               uint8_t count) {
    for (uint8_t i = 0; i < count && _fbQueueCount < LENZ_LAN_FB_QUEUE_SIZE; i++) {
        _fbQueueAddr[_fbQueueCount] = addr[i];
        _fbQueueData[_fbQueueCount] = data[i];
        _fbQueueCount++;
    }
}

// ─────────────────────────────────────────────────────────────
//  _flushPendingFeedback() — send queued RS-Bus packets
//
//  Called from loop() after _txBusy is cleared. Sends all queued
//  feedback pairs and resets the queue.
// ─────────────────────────────────────────────────────────────
void LenzLan::_flushPendingFeedback() {
    if (_fbQueueCount == 0) return;
    _sendFeedbackBulk(_fbQueueAddr, _fbQueueData, _fbQueueCount);
    _fbQueueCount = 0;
}

// ─────────────────────────────────────────────────────────────
//  _sendFeedbackBulk() — send RS-Bus feedback as BC "Rückmeldung"
//                         packets (§2.4.5, header = 0x40+N)
//
//  Splits an array of addr/data pairs into FF FD [0x40+N] packets of
//  at most 3 pairs each (the Lenz packet-size convention), building
//  and appending the XOR checksum for each packet as it is completed.
//
//  addrByte: bits 6-0 = RS-Bus module address (1-127), bit 7 always 0
//  dataByte: ITNZ — I(0) TT(10=Rückmeldebaustein) N(nibble, bit4) ZZZZ(value)
//            (§2.15 "Schaltinformation", as required by §2.4.5)
//
//  The XOR checksum is computed over the header byte plus all data
//  bytes within that one packet. Each finished packet is sent to
//  every active client via _sendRaw() as a broadcast (FF FD header).
//  Does nothing if there are no pairs to send, or no clients are
//  currently connected.
// ─────────────────────────────────────────────────────────────
void LenzLan::_sendFeedbackBulk(const uint8_t* addr, const uint8_t* data,
                                  uint8_t count) {
    if (count == 0 || _connCount == 0) return;

    uint8_t pkt[10];
    pkt[0] = 0xFF; pkt[1] = 0xFD;

    uint8_t i = 0;
    while (i < count) {
        uint8_t pairCount = 0;
        while (pairCount < 3 && i < count) {
            pkt[3 + pairCount * 2]     = addr[i];
            pkt[3 + pairCount * 2 + 1] = data[i];
            pairCount++;
            i++;
        }
        // Header byte = 0x40 + N, N = number of data bytes that follow (§2.4.5)
        uint8_t kennung = 0x40 + pairCount * 2;
        pkt[2] = kennung;
        uint8_t xorVal = kennung;
        for (uint8_t b = 0; b < pairCount * 2; b++) xorVal ^= pkt[3 + b];
        uint8_t pktLen = 3 + pairCount * 2 + 1;
        pkt[pktLen - 1] = xorVal;

        for (uint8_t s = 0; s < LENZ_LAN_MAX_CLIENTS; s++)
            _sendRaw(_clients[s], pkt, pktLen);
    }
}

// ─────────────────────────────────────────────────────────────
//  _checkTimeouts() — disconnect inactive connections
//
//  Normal timeout: configurable via "xn.lan_timeout_s" (web interface,
//  XpressNet-LAN section), read live from EEPROM each call so a
//  change takes effect immediately without a reboot — falls back to
//  LENZ_LAN_TIMEOUT_DEFAULT_MS (60s) if not set. See that constant's
//  own comment in lenz_lan.h for why this became configurable (Rob:
//  Rocrail repeatedly disconnecting while the layout sits idle,
//  because its own idle-state lifecheck interval turned out to be
//  longer than the previous fixed 30s).
//  Programming-mode timeout: 90 seconds (LENZ_LAN_PROG_TIMEOUT).
//  The PC sends keep-alive messages (F1) to prevent this timeout.
//
//  Iterates over every client slot; skips inactive ones. For each
//  active client, picks the applicable timeout (the longer
//  programming-mode timeout if that client is currently in
//  programming mode, otherwise the normal timeout) and, if more time
//  than that has elapsed since its last recorded activity, tears the
//  connection down: stops the socket, marks the slot inactive, clears
//  programming mode, and decrements the connection counter.
// ─────────────────────────────────────────────────────────────
void LenzLan::_checkTimeouts() {
    uint32_t now = millis();
    uint32_t normalTimeoutMs = (uint32_t)eepromStore().getUint16(
        "xn.lan_timeout_s", LENZ_LAN_TIMEOUT_DEFAULT_MS / 1000) * 1000;
    for (uint8_t i = 0; i < LENZ_LAN_MAX_CLIENTS; i++) {
        if (!_clients[i].active) continue;
        uint32_t timeout = _clients[i].progMode
                         ? LENZ_LAN_PROG_TIMEOUT
                         : normalTimeoutMs;
        if (now - _clients[i].lastActMs > timeout) {
            _clients[i].tcp.stop();
            _clients[i].active   = false;
            _clients[i].progMode = false;
            _connCount--;
        }
    }
}

// ─────────────────────────────────────────────────────────────
//  _frameLen() — determine the total length of a LAN frame
//
//  LAN frame structure: FF FE/FD [XN header] [XN data bytes] [XOR]
//    Total = 2 (LAN header) + 1 (XN header) + N (data) + 1 (XOR)
//    where N = XN header bits 3-0
//
//  Returns 0 if fewer than 3 bytes are available yet (not even enough
//  to read the XN header byte), or if the frame is not yet fully
//  received given the length implied by that header (i.e. the caller
//  should wait for more bytes before trying again). Returns 0xFF if
//  buf[0] is not a valid LAN header byte at all (the caller should
//  skip this byte and try the next position). Otherwise returns the
//  total number of bytes this complete frame occupies, including both
//  LAN header bytes and the trailing XOR checksum.
// ─────────────────────────────────────────────────────────────
uint8_t LenzLan::_frameLen(const uint8_t* buf, uint8_t available) {
    if (available < 3) return 0;
    if (buf[0] != LENZ_LAN_HDR1) return 0xFF;
    uint8_t xnHeader  = buf[2];
    uint8_t dataBytes = xnHeader & 0x0F;
    uint8_t total     = 2 + 1 + dataBytes + 1;
    return (available >= total) ? total : 0;
}
