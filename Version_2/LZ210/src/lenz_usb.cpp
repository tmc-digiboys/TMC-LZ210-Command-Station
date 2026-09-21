// ═══════════════════════════════════════════════════════════════
//  lenz_usb.cpp  —  Lenz LI-USB protocol over USB CDC serial
//
//  Implements the Lenz LAN framing protocol over the Arduino USB
//  CDC serial port (Serial). Identical frame format to LenzLAN
//  but without TCP connection management or keep-alive timeouts.
// ═══════════════════════════════════════════════════════════════
#include "lenz_usb.h"
#include "trace_log.h"
#include "rs_bus_hal.h"
#include "eeprom_store.h"
#include "serial_config.h"

// Global instance used by all modules and the EventBus handler
LenzUsb gLenzUsb;

// ─────────────────────────────────────────────────────────────
//  begin() — initialise USB CDC serial and register EventBus handler
//
//  Opens the USB CDC serial port at LENZ_USB_BAUD and resets all
//  receive/state bookkeeping (rxLen, progMode, hasActivity,
//  lastActMs, frameStartMs) to their initial values. Registers this
//  module's onEvent() method as the EventBus handler for
//  ModuleId::LENZ_USB, so hardware events published by other modules
//  are forwarded to the connected PC as Lenz LAN broadcasts.
// ─────────────────────────────────────────────────────────────
void LenzUsb::begin() {
    // "usb.baud"/"usb.databits"/"usb.parity"/"usb.stopbits" (web
    // interface, USB/Serial panel) — configurable per operator need
    // (e.g. Rocrail's default of 57600, which also happens to match
    // LENZ_USB_BAUD's own default). Note: this is native USB CDC
    // (TinyUSB) — most CDC-ACM drivers only report/track these
    // settings rather than actually rate-/frame-limiting transfer by
    // them, so this mainly exists for client software that insists on
    // specific values rather than because the link itself needs it.
    Serial.begin(eepromStore().getUint32("usb.baud", LENZ_USB_BAUD),
                 serialConfigFromEeprom("usb"));
    _rxLen        = 0;
    _progMode     = false;
    _hasActivity  = false;
    _lastActMs    = millis();
    _frameStartMs = 0;

    EventBus::instance().registerHandler(ModuleId::LENZ_USB,
        [](const Event& ev){ gLenzUsb.onEvent(ev); });
}

// ─────────────────────────────────────────────────────────────
//  loop() — read serial data and process complete frames
//
//  Reads every currently available byte from Serial into the receive
//  buffer (up to its capacity), updating the last-activity timestamp
//  and the "has activity" flag as bytes come in.
//
//  If the buffer is completely full AND more data is still waiting on
//  the serial port, this is a genuine buffer-overflow condition (spec
//  "01/06/07"): logs it, builds and sends the standard buffer-
//  overflow error response directly (bypassing _sendFrame(), since
//  the LAN header bytes are prepended manually here), resets the
//  receive state, and discards whatever is still waiting on the
//  serial port for this cycle, so sender and receiver can
//  resynchronise on the next frame boundary.
//
//  Applies the inactivity timeout: if any activity has been seen
//  before, and more time than the applicable timeout (the longer
//  programming-mode timeout if currently in programming mode,
//  otherwise the normal timeout) has passed since the last byte was
//  received, the receive buffer and state are reset — this exists
//  mainly to recover from a stuck/partial frame after a period of
//  silence, since USB itself has no real "connection" to time out in
//  the way a TCP socket would.
//
//  If the buffer is empty, clears the frame-start timer and returns
//  immediately (nothing to process).
//
//  Otherwise scans the buffer for complete frames: skips forward past
//  any stray bytes that are not the expected LAN header byte, and for
//  each byte that does match, asks _frameLen() how long the frame
//  starting there would be. A return value of 0 means "not enough
//  bytes yet", so the scan stops for this call, waiting for more data
//  to arrive. A return value of 0xFF means the byte was not actually
//  a valid frame start, so the scan simply advances by one byte and
//  keeps looking. Otherwise the frame is complete and is handed to
//  _processFrame() (with the two LAN header bytes stripped off), and
//  the scan continues right after this frame.
//
//  Once scanning stops, any bytes that were consumed are removed from
//  the front of the buffer (the remaining, not-yet-complete bytes are
//  shifted down to offset 0) so the next call starts cleanly.
//
//  Finally implements the "01/01/00" incomplete-frame timeout from the
//  interface spec: if the buffer is now empty, there is no partial
//  frame in progress, so the timer is cleared. Otherwise, if this is
//  the first byte of a new partial frame, the current time is
//  recorded as its start; if a partial frame has now been sitting
//  incomplete for longer than LENZ_USB_FRAME_TIMEOUT_MS, the frame is
//  considered abandoned: the timeout error response is sent directly,
//  and the receive state is reset so the next attempt starts fresh.
// ─────────────────────────────────────────────────────────────
void LenzUsb::loop() {
    // Read all available bytes into the receive buffer
    while (Serial.available() && _rxLen < LENZ_USB_RX_BUF) {
        _rxBuf[_rxLen++] = (uint8_t)Serial.read();
        _lastActMs       = millis();
        _hasActivity     = true;
    }

    // Buffer overflow: buffer full but there is still more to read.
    if (_rxLen >= LENZ_USB_RX_BUF && Serial.available()) {
        traceSerial.println("LenzUsb: buffer-overflow, herstel...");
        uint8_t err[3];
        XnInterfaceLayer::buildBufferOverflowError(err);
        uint8_t resp[] = { LENZ_USB_HDR1, LENZ_USB_HDR_CMD, err[0], err[1], err[2] };
        _sendRaw(resp, sizeof(resp));
        _rxLen        = 0;
        _frameStartMs = 0;
        while (Serial.available()) Serial.read();  // discard the rest
        return;
    }

    // Reset on inactivity timeout (longer timeout in programming mode)
    uint32_t timeout = _progMode ? LENZ_USB_PROG_TO : LENZ_USB_TIMEOUT_MS;
    if (_hasActivity && millis() - _lastActMs > timeout) {
        // Explicit "missed lifecheck" logging — same reasoning as
        // LenzLan/Z21Lan's equivalent timeout logging (no persistent
        // "connection" to tear down here, since USB has none, but the
        // reset of programming-mode/frame state previously happened
        // silently).
        traceLog().logf(TraceLevel::WARNING, TraceSource::XN_USB, "TIMEOUT idle=%lums threshold=%lums progMode=%d",
                         (unsigned long)(millis() - _lastActMs),
                         (unsigned long)timeout, _progMode);
        _rxLen       = 0;
        _progMode    = false;
        _hasActivity = false;
    }

    if (_rxLen == 0) { _frameStartMs = 0; return; }

    // Process all complete frames in the receive buffer
    uint8_t offset = 0;
    while (offset < _rxLen) {
        if (_rxBuf[offset] != LENZ_USB_HDR1) { offset++; continue; }

        uint8_t total = _frameLen(_rxBuf + offset, _rxLen - offset);
        if (total == 0)    break;      // incomplete frame — wait for more data
        if (total == 0xFF) { offset++; continue; }  // invalid — skip byte

        _processFrame(_rxBuf + offset + 2, total - 2);
        offset += total;
    }

    // Remove processed bytes from the front of the buffer
    if (offset > 0 && offset <= _rxLen) {
        _rxLen -= offset;
        memmove(_rxBuf, _rxBuf + offset, _rxLen);
    }

    // Timeout on an incomplete frame (spec §1.5, "01/01/00").
    if (_rxLen == 0) {
        _frameStartMs = 0;
    } else {
        if (_frameStartMs == 0) _frameStartMs = millis();
        else if (millis() - _frameStartMs > LENZ_USB_FRAME_TIMEOUT_MS) {
            traceSerial.println("LenzUsb: onvolledig frame timeout");
            uint8_t err[3];
            XnInterfaceLayer::buildFrameTimeoutError(err);
            uint8_t resp[] = { LENZ_USB_HDR1, LENZ_USB_HDR_CMD, err[0], err[1], err[2] };
            _sendRaw(resp, sizeof(resp));
            _rxLen        = 0;
            _frameStartMs = 0;
        }
    }
}

// ─────────────────────────────────────────────────────────────
//  _freeConnections() — USB is a single serial link, with no socket
//  pool like Ethernet has. Simply fixed at 1 (there is always exactly
//  one possible "connection", which already exists as soon as the USB
//  port is open — see also maxConnections()==1 above).
// ─────────────────────────────────────────────────────────────
uint8_t LenzUsb::_freeConnections() {
    return 1;
}

// ─────────────────────────────────────────────────────────────
//  Lazy-initialised XpressNet handler for USB
// ─────────────────────────────────────────────────────────────
static XpressNetHandler sUsbXpressNet;
static bool sUsbInit = false;

// ─────────────────────────────────────────────────────────────
//  _processFrame() — parse and respond to one LAN frame
//
//  Interface-local commands (keep-alive, version, number of
//  connections, ...) are intercepted by XnInterfaceLayer. All other
//  frames go to the shared XpressNet handler. The handler is
//  lazy-initialised on first use and given a broadcast callback that
//  BUFFERS broadcasts (rather than sending them immediately) — see
//  xn_interface_layer.h for the reason (01/04/05 must always be sent
//  before any resulting broadcast).
//
//  On the very first call (sUsbInit still false), performs one-time
//  setup: initialises the shared handler for this module, wires up
//  its broadcast callback to buffer via _ifLayer, and marks
//  initialisation done. Immediately afterwards, sends an initial
//  RS-Bus "dump": one or more BC "Rückmeldung" packets (per
//  §2.4.5/§2.15) covering every feedback module address that
//  RsBusHal currently knows about (isSeen()==true), including both
//  nibbles for each, batched up to 3 address/data pairs per packet —
//  so the PC immediately knows the current state of every known
//  feedback module as soon as it starts talking to the station, the
//  same way LenzLan does for a newly accepted TCP client. Note that
//  RsBusHal counts RS-Bus hardware addresses starting at 1 (1-128),
//  while the protocol counts from 0 (0-127) — hence the "-1" applied
//  to addrByte below.
//
//  After that one-time setup (or immediately, on every subsequent
//  call), passes the frame to _ifLayer.process(), which either
//  answers an interface-local command directly or forwards it to the
//  shared handler (sUsbXpressNet). For REPLY/ERROR results with a
//  non-empty response, the response bytes are sent directly as a
//  non-broadcast frame. For OK_SILENT, the fixed 01/04/05
//  acknowledgement is built and sent instead. In either case (and for
//  any other result), any broadcast that ended up buffered in
//  _ifLayer during this call is retrieved afterwards and sent to the
//  PC via _sendBroadcast() — always AFTER the direct response/ack
//  above, matching the ordering the real Lenz interface uses.
// ─────────────────────────────────────────────────────────────
void LenzUsb::_processFrame(const uint8_t* buf, uint8_t len) {
    // Generic RX log — every frame reaching this read handler. The
    // corresponding "-> XN" handoff log sits in the shared
    // XnInterfaceLayer::process() (xn_interface_layer.cpp), the exact
    // point this frame (if not interface-local) reaches the shared
    // XpressNetHandler — shared with LenzLan's own equivalent RX log
    // for the same reasoning (Rob).
    traceLog().logBytes(TraceLevel::DEBUG, TraceSource::XN_USB, "RX", buf, len);
    // Lazy-initialise the XpressNet handler on first use.
    // Also send initial RS-Bus feedback state so the PC knows the current
    // state of all active feedback modules immediately after connection.
    if (!sUsbInit) {
        sUsbXpressNet.begin(ModuleId::LENZ_USB);
        sUsbXpressNet.setBroadcastCallback([](const uint8_t* data, uint8_t dlen, uint16_t /*locoAddr*/) {
            gLenzUsb._ifLayer.bufferBroadcast(data, dlen);
        });
        sUsbInit = true;

        // Send RS-Bus dump: all present modules as BC "Rückmeldung" packets
        // (§2.4.5 → ADR/ITNZ format per §2.15). gRsBusHal internally counts
        // RS-Bus hw addresses (1-128); the protocol counts 0-127, hence -1.
        uint8_t pkt[10];
        pkt[0] = 0xFF; pkt[1] = 0xFD;
        uint8_t pairCount = 0;
        for (uint8_t hwAddr = 1; hwAddr <= 128; hwAddr++) {
            if (!gRsBusHal.isSeen(hwAddr)) continue;  // module not present — skip
            for (uint8_t nibble = 0; nibble <= 1; nibble++) {
                uint8_t data2 = gRsBusHal.getNibble(hwAddr, nibble);
                // Send all nibbles for present modules, even if value is 0
                uint8_t addrByte = (hwAddr - 1) & 0x7F;
                uint8_t itnz = ((gRsBusHal.getType(hwAddr) & 0x03) << 5)
                             | (nibble ? 0x10 : 0x00)
                             | (data2 & 0x0F);
                pkt[3 + pairCount * 2]     = addrByte;
                pkt[3 + pairCount * 2 + 1] = itnz;
                pairCount++;
                if (pairCount == 3) {
                    uint8_t kennung = 0x40 + pairCount * 2;  // §2.4.5
                    pkt[2] = kennung;
                    uint8_t xorVal = kennung;
                    for (uint8_t b = 0; b < pairCount * 2; b++) xorVal ^= pkt[3 + b];
                    pkt[9] = xorVal;
                    _sendRaw(pkt, 10);
                    pairCount = 0;
                }
            }
        }
        if (pairCount > 0) {
            uint8_t kennung = 0x40 + pairCount * 2;
            pkt[2] = kennung;
            uint8_t xorVal = kennung;
            for (uint8_t b = 0; b < pairCount * 2; b++) xorVal ^= pkt[3 + b];
            pkt[3 + pairCount * 2] = xorVal;
            _sendRaw(pkt, 3 + pairCount * 2 + 1);
        }
    }

    // Interface-local + shared protocol handling
    uint8_t resp[24]; uint8_t respLen = 0;
    XNHandleResult r = gLenzUsb._ifLayer.process(sUsbXpressNet, buf, len, resp, respLen,
                                                   &LenzUsb::_freeConnections);

    if ((r == XNHandleResult::REPLY || r == XNHandleResult::ERROR) && respLen > 0) {
        XnFrame f;
        memcpy(f.data, resp, respLen);
        f.len = respLen; f.isBroadcast = false;
        traceLog().logBytes(TraceLevel::DEBUG, TraceSource::XN_USB, "TX", f.data, f.len);
        _sendFrame(f);
    } else if (r == XNHandleResult::OK_SILENT) {
        uint8_t ack[3];
        uint8_t ackLen = XnInterfaceLayer::buildCommandSentAck(ack);
        XnFrame f;
        memcpy(f.data, ack, ackLen);
        f.len = ackLen; f.isBroadcast = false;
        traceLog().logBytes(TraceLevel::DEBUG, TraceSource::XN_USB, "TX", f.data, f.len);
        _sendFrame(f);
    }

    // Send any buffered broadcast only NOW — after the direct
    // response/ack above (see xn_interface_layer.h).
    if (gLenzUsb._ifLayer.hasPendingBroadcast()) {
        uint8_t bc[8];
        uint8_t bcLen = gLenzUsb._ifLayer.takePendingBroadcast(bc);
        XnFrame f;
        memcpy(f.data, bc, bcLen);
        f.len = bcLen; f.isBroadcast = true;
        traceLog().logBytes(TraceLevel::DEBUG, TraceSource::XN_USB, "BC", f.data, f.len);
        _sendBroadcast(f);
    }
}

// ─────────────────────────────────────────────────────────────
//  _sendRaw() — low-level primitive used by every send path
//
//  Writes the given bytes to the serial port and flushes immediately
//  afterwards. USB serial has no TCP-segment boundary the way a
//  socket does, but keeping the same write+flush discipline in one
//  single place prevents some future send path from forgetting to
//  flush — the same reasoning applied during the LenzLAN refactor.
//  Does nothing if len is zero.
// ─────────────────────────────────────────────────────────────
// _sendRaw() — write raw bytes to the USB CDC port.
//
// Deliberately does NOT call Serial.flush() after every write. This
// used to (Rob, confirmed via TraceLog + reproduction: driving errors
// starting the exact moment the first LocoNet sensor triggers) —
// individual RS-Bus/LocoNet feedback changes are each published as
// their own separate Ev::feedback() event (see SlotServer::
// _handleInputRep()'s comment — not batched like FEEDBACK_BULK), so a
// burst of sensor activity (e.g. right as a train enters the first
// occupied block) meant a burst of individual _sendRaw() calls here,
// each blocking on flush() until the USB stack had actually finished
// transmitting that one small packet — stalling core1 (where LenzUsb,
// LocoNet, and everything else sharing this core runs) for long
// enough to delay/drop the driving commands Rocrail was sending over
// this exact same USB link at the same time. Native USB CDC (TinyUSB)
// transmits queued Serial.write() data in the background regardless —
// flush() only forces this function to wait for that background send
// to finish before returning, which isn't needed for correctness (no
// data is lost by not waiting), only for making one particular send
// synchronous — not worth the cost paid on every single one of these.
void LenzUsb::_sendRaw(const uint8_t* buf, uint8_t len) {
    if (len == 0) return;
    Serial.write(buf, len);
}

// ─────────────────────────────────────────────────────────────
//  _sendFrame() — send one LAN frame over USB serial
//
//  Prepends the 2-byte LAN header (FF FE for direct reply,
//  FF FD for broadcast) before the XpressNet data bytes.
// ─────────────────────────────────────────────────────────────
void LenzUsb::_sendFrame(const XnFrame& f) {
    if (f.len == 0) return;
    uint8_t buf[36];
    buf[0] = LENZ_USB_HDR1;
    buf[1] = f.isBroadcast ? LENZ_USB_HDR_BC : LENZ_USB_HDR_CMD;
    memcpy(buf + 2, f.data, f.len);
    _sendRaw(buf, 2 + f.len);
}

// ─────────────────────────────────────────────────────────────
//  _sendBroadcast() — send a broadcast frame over USB serial
//
//  Broadcasts use the FF FD header. This method is called by the
//  XpressNet handler's broadcast callback and by onEvent().
// ─────────────────────────────────────────────────────────────
void LenzUsb::_sendBroadcast(const XnFrame& f) {
    _sendFrame(f);
}

// ─────────────────────────────────────────────────────────────
//  onEvent() — forward hardware events to the USB port as broadcasts
//
//  POWER_ON/OFF: sends the fixed 3-byte "track power" broadcast
//    directly as an FF FD packet.
//  FEEDBACK_BULK: sends RS-Bus feedback as BC "Rückmeldung" packets
//    (§2.4.5: header = 0x40+N), up to 3 module/nibble pairs per packet.
//    Splits the event's full set of pairs into as many packets as
//    needed, computing and appending the XOR checksum for each one.
//  FEEDBACK: sends a single RS-Bus feedback packet (N=2, one
//    address/data pair, fixed header 0x42).
//  All other event types are handled by the shared XpressNet handler
//  via the broadcast callback registered in _processFrame() (which
//  buffers them through _ifLayer rather than sending them from here).
// ─────────────────────────────────────────────────────────────
void LenzUsb::onEvent(const Event& ev) {
    // Power events — send a 0x61 broadcast over USB
    if (ev.type == EvType::POWER_ON) {
        uint8_t pkt[] = { 0xFF, 0xFD, 0x61, 0x01, 0x60 };
        _sendRaw(pkt, sizeof(pkt));
        return;
    }
    if (ev.type == EvType::POWER_OFF) {
        uint8_t pkt[] = { 0xFF, 0xFD, 0x61, 0x00, 0x61 };
        _sendRaw(pkt, sizeof(pkt));
        return;
    }
    // Forward RS-Bus bulk feedback as BC "Rückmeldung" packets
    if (ev.type == EvType::FEEDBACK_BULK) {
        uint8_t pkt[10];
        pkt[0] = 0xFF; pkt[1] = 0xFD;
        uint8_t i = 0;
        while (i < ev.fbBulkCount) {
            uint8_t pairCount = 0;
            while (pairCount < 3 && i < ev.fbBulkCount) {
                pkt[3 + pairCount * 2]     = ev.fbBulkAddr[i];
                pkt[3 + pairCount * 2 + 1] = ev.fbBulkData[i];
                pairCount++; i++;
            }
            uint8_t kennung = 0x40 + pairCount * 2;  // §2.4.5
            pkt[2] = kennung;
            uint8_t xorVal = kennung;
            for (uint8_t b = 0; b < pairCount * 2; b++) xorVal ^= pkt[3 + b];
            pkt[3 + pairCount * 2] = xorVal;
            _sendRaw(pkt, 3 + pairCount * 2 + 1);
        }
        return;
    }

    // Forward a single RS-Bus/LocoNet feedback change as a BC
    // "Rückmeldung" packet (N=2 → 0x42). LocoNet-derived feedback
    // (this branch) goes through _setFeedbackBitStepped() rather than
    // a direct translation — see that function's own comment for why:
    // it applies the same confirmed +1 bit-shift LenzLan already
    // applies for LocoNet sensors (Rob, confirmed via tcpdump +
    // Rocrail's own log — without it, Rocrail over USB was reporting
    // the wrong sensor for every LocoNet-derived event, one address
    // off, while Ethernet/Z21/LocoNet's own decode were all already
    // correct). RS-Bus-derived feedback is unaffected — RS-Bus always
    // publishes via the separate EvType::FEEDBACK_BULK branch above,
    // which this does not touch.
    if (ev.type == EvType::FEEDBACK) {
        uint16_t rawBase = (uint16_t)(ev.fbModule - 1) * 8 + (ev.fbNibble ? 4 : 0);
        uint8_t  dat     = ev.fbDat & 0x0F;
        for (uint8_t b = 0; b < 4; b++) {
            _setFeedbackBitStepped(rawBase + b + 1, (dat & (1 << b)) != 0);
        }
    }
}

// ─────────────────────────────────────────────────────────────
//  _setFeedbackBitStepped() — sets one bit, at a GLOBAL, continuous
//  bit position (module byte = pos/8, bit-within-byte = pos%8) across
//  the whole _lastSentNibble[] cache, to the given state — and, only
//  if this actually changes that bit, sends the resulting, complete
//  4-bit nibble containing it as a single XpressNet feedback packet
//  over USB.
//
//  Mirrors LenzLan::_setFeedbackBitStepped() (lenz_lan.cpp) exactly —
//  see that function's own comment for the full rationale behind
//  treating _lastSentNibble[128] as one continuous 1024-bit space
//  (letting a +1-shifted bit land in a different module/nibble than
//  the raw LocoNet event's own) and for only sending on an actual
//  change. The one difference from LenzLan's version: no TCP
//  backpressure/_txBusy queueing exists for this USB link, so this
//  sends directly via _sendRaw() rather than going through a
//  queue-or-send-now branch.
// ─────────────────────────────────────────────────────────────
void LenzUsb::_setFeedbackBitStepped(uint16_t globalBitPos, bool state) {
    uint16_t byteIdx = globalBitPos / 8;
    uint8_t  bitIdx  = globalBitPos % 8;
    if (byteIdx >= sizeof(_lastSentNibble)) return;  // out of the 128-module range

    uint8_t mask    = 1 << bitIdx;
    bool    current = (_lastSentNibble[byteIdx] & mask) != 0;
    if (current == state) return;  // already at this value — nothing to send

    if (state) _lastSentNibble[byteIdx] |= mask;
    else       _lastSentNibble[byteIdx] &= (uint8_t)~mask;

    bool    nibbleFlag = bitIdx >= 4;
    uint8_t nibbleVal  = nibbleFlag ? (_lastSentNibble[byteIdx] >> 4)
                                    : (_lastSentNibble[byteIdx] & 0x0F);
    uint8_t addrByte   = (uint8_t)byteIdx;
    uint8_t dataByte   = 0x40 | (nibbleFlag ? 0x10 : 0x00) | nibbleVal;
    uint8_t kennung    = 0x42;  // N=2 (one pair)
    uint8_t xorVal     = kennung ^ addrByte ^ dataByte;
    uint8_t pkt[]      = { 0xFF, 0xFD, kennung, addrByte, dataByte, xorVal };
    _sendRaw(pkt, sizeof(pkt));
}

// ─────────────────────────────────────────────────────────────
//  _frameLen() — determine the length of the next complete LAN frame
//
//  Returns the total frame length in bytes (including the 2-byte
//  header) if a complete frame is present in buf[0..available-1].
//  Returns 0 if more data is needed (incomplete frame).
//  Returns 0xFF if buf[0] is not the expected 0xFF header byte.
// ─────────────────────────────────────────────────────────────
uint8_t LenzUsb::_frameLen(const uint8_t* buf, uint8_t available) {
    if (available < 3) return 0;
    if (buf[0] != LENZ_USB_HDR1) return 0xFF;  // not a valid frame start
    uint8_t dataBytes = buf[2] & 0x0F;           // low nibble of command byte
    uint8_t total = 2 + 1 + dataBytes + 1;       // header + cmd + data + XOR
    return (available >= total) ? total : 0;
}
