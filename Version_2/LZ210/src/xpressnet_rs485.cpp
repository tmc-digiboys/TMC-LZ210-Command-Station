#include "xpressnet_rs485.h"
#include "trace_log.h"

XpressNetRs485 gXpressNetRs485;

// ─────────────────────────────────────────────────────────────
//  PIO programs — identical to the previously working version
//
//  These two small PIO assembly programs implement a 9-bit software
//  UART directly in the RP2040/RP2350's PIO hardware, running at
//  62500 baud (the fixed XpressNet RS-485 bit rate). The 9th bit is
//  used by XpressNet itself to distinguish "address/call bytes"
//  (bit8=1) from "data bytes" (bit8=0) on the wire.
//
//  xn_uart_tx_prog (PIO2 SM0, origin 0):
//    Pulls one 9-bit word from the TX FIFO and shifts it out, LSB
//    first, one bit per iteration of the x-8..0 countdown loop, with
//    side-set used to drive the TX pin's line level. Runs at 8 PIO
//    clock cycles per bit (see the clock divider set in _initPioTx()).
//
//  xn_uart_rx_prog (PIO2 SM1, origin 4):
//    Waits for the start bit (line low), then samples the RX pin
//    once per bit period into the input shift register for 9 bits
//    (using the same x-8..0 countdown), and finally pushes the
//    assembled word to the RX FIFO.
// ─────────────────────────────────────────────────────────────
#define xn_uart_tx_wrap_target 0
#define xn_uart_tx_wrap        3
static const uint16_t xn_uart_tx_instructions[] = {
    0x9fa0, // pull block  side 1 [7]
    0xf728, // set x,8    side 0 [7]
    0x6601, // out pins,1        [6]
    0x0042, // jmp x--,2
};
static const struct pio_program xn_uart_tx_prog = {
    .instructions = xn_uart_tx_instructions,
    .length = 4, .origin = XN_TX_ORIGIN, .pio_version = 0
};

#define xn_uart_rx_wrap_target 0
#define xn_uart_rx_wrap        4
static const uint16_t xn_uart_rx_instructions[] = {
    0x2020, // wait 0 pin 0
    0xea28, // set x,8 [10]
    0x4601, // in pins,1 [6]
    0x0042, // jmp x--,2
    0x8020, // push block
};
static const struct pio_program xn_uart_rx_prog = {
    .instructions = xn_uart_rx_instructions,
    .length = 5, .origin = XN_RX_ORIGIN, .pio_version = 0
};

// ─────────────────────────────────────────────────────────────
//  begin() — one-time module initialisation
//
//  Sets the DE (driver-enable) pin as an output, defaulting to
//  receive mode (LOW). Loads and configures both PIO state machines
//  (see _initPioTx()/_initPioRx()). Clears all per-slave bookkeeping
//  (_slaves[]) and the broadcast queue (_bcQueue[]), and resets the
//  round-robin poll position and programming-mode state to their
//  initial values.
//
//  Registers this module's static _onEvent() callback with the
//  EventBus so that broadcast-worthy events published by other
//  modules (power state changes, emergency stop, feedback updates)
//  get translated into XpressNet broadcasts on this bus.
//
//  Finally, queues an initial "track power off" broadcast (0x61 0x00
//  0x61). This matters because the command station always boots with
//  track power off; broadcasting this immediately at startup lets any
//  already-connected handheld (e.g. a MultiMaus) show the correct
//  "stopped" state on its display right away, instead of waiting for
//  its own next query.
// ─────────────────────────────────────────────────────────────
void XpressNetRs485::begin() {
    pinMode(XN_RS485_DE_PIN, OUTPUT);
    digitalWrite(XN_RS485_DE_PIN, LOW);

    _initPioTx();
    _initPioRx();

    memset(_slaves,  0, sizeof(_slaves));
    memset(_bcQueue, 0, sizeof(_bcQueue));
    _currentSlave   = 1;
    _progModeActive = false;
    _progModeSlave  = 0;

    // EventBus handler for broadcasts
    EventBus::instance().registerHandler(
        ModuleId::XPRESSNET_RS485, XpressNetRs485::_onEvent);

    // Send a power-off broadcast at boot — the station always starts
    // with track power off. This lets a MultiMaus (or any other
    // already-connected handheld) know the initial state and show
    // the stop symbol immediately.
    // 0x61 0x00 0x61 = track power off
    static const uint8_t powerOffMsg[] = { 0x61, 0x00, 0x61 };
    enqueueBroadcast(powerOffMsg, sizeof(powerOffMsg));
}

// ─────────────────────────────────────────────────────────────
//  _initPioTx() — configure PIO2 SM0 as the 9-bit UART transmitter
//
//  Loads xn_uart_tx_prog into PIO2 at the fixed XN_TX_ORIGIN offset,
//  configures the wrap range, enables 2-bit side-set (used for the
//  TX line level), sets the TX pin as a PIO-controlled output pin,
//  configures the output shift register to shift LSB-first out of a
//  32-bit register (only the low 9 bits matter per word), and
//  computes/sets the clock divider so the state machine runs at
//  8 PIO cycles per bit, giving exactly 62500 baud at the current
//  system clock. Finally resets and re-enables the state machine,
//  making sure the TX pin idles high (mark state) before enabling.
// ─────────────────────────────────────────────────────────────
void XpressNetRs485::_initPioTx() {
    _txProgOffset = pio_add_program_at_offset(
        pio2, &xn_uart_tx_prog, XN_TX_ORIGIN);

    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, _txProgOffset + xn_uart_tx_wrap_target,
                           _txProgOffset + xn_uart_tx_wrap);
    sm_config_set_sideset(&c, 2, true, false);

    pio_gpio_init(pio2, XN_RS485_TX_PIN);
    pio_sm_set_consecutive_pindirs(pio2, 0, XN_RS485_TX_PIN, 1, true);
    sm_config_set_sideset_pins(&c, XN_RS485_TX_PIN);
    sm_config_set_out_pins(&c, XN_RS485_TX_PIN, 1);
    sm_config_set_out_shift(&c, true, false, 32);

    float div = (float)clock_get_hz(clk_sys) / (62500.0f * 8.0f);
    sm_config_set_clkdiv(&c, div);

    pio_sm_set_enabled(pio2, 0, false);
    pio_sm_init(pio2, 0, _txProgOffset, &c);
    pio_sm_clear_fifos(pio2, 0);
    pio_sm_set_pins_with_mask(pio2, 0, 1u << XN_RS485_TX_PIN,
                                        1u << XN_RS485_TX_PIN);
    pio_sm_set_enabled(pio2, 0, true);
}

// ─────────────────────────────────────────────────────────────
//  _initPioRx() — configure PIO2 SM1 as the 9-bit UART receiver
//
//  Loads xn_uart_rx_prog into PIO2 at the fixed XN_RX_ORIGIN offset,
//  configures the wrap range, sets the RX pin as a PIO-controlled
//  input pin, configures the input shift register to shift LSB-first
//  into a 32-bit register, enables GPIO input hysteresis on the RX
//  pin (reduces false bit transitions caused by noise/reflections on
//  the shared RS-485 bus), computes/sets the same clock divider as
//  the TX program (so both run at exactly 62500 baud), and resets
//  and re-enables the state machine.
// ─────────────────────────────────────────────────────────────
void XpressNetRs485::_initPioRx() {
    _rxProgOffset = pio_add_program_at_offset(
        pio2, &xn_uart_rx_prog, XN_RX_ORIGIN);

    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, _rxProgOffset + xn_uart_rx_wrap_target,
                           _rxProgOffset + xn_uart_rx_wrap);

    pio_gpio_init(pio2, XN_RS485_RX_PIN);
    pio_sm_set_consecutive_pindirs(pio2, 1, XN_RS485_RX_PIN, 1, false);
    sm_config_set_in_pins(&c, XN_RS485_RX_PIN);
    sm_config_set_in_shift(&c, true, false, 32);

    // Enable hysteresis on the RX pin for better signal integrity
    gpio_set_input_hysteresis_enabled(XN_RS485_RX_PIN, true);

    float div = (float)clock_get_hz(clk_sys) / (62500.0f * 8.0f);
    sm_config_set_clkdiv(&c, div);

    pio_sm_set_enabled(pio2, 1, false);
    pio_sm_init(pio2, 1, _rxProgOffset, &c);
    pio_sm_clear_fifos(pio2, 1);
    pio_sm_set_enabled(pio2, 1, true);
}

// ─────────────────────────────────────────────────────────────
//  loop() — one iteration of the RS-485 poll cycle
//
//  Runs on core1, called repeatedly from the module registry's
//  core1 loop. Each call does exactly three things:
//    1. Flushes any broadcasts that have been queued since the last
//       call (see _flushBroadcasts()).
//    2. Polls exactly one slave address: if programming mode is
//       active and a specific slave has been recorded as "the one in
//       programming mode", that slave is polled every time (so it
//       gets priority bus access while programming); otherwise the
//       normal round-robin address (_currentSlave) is polled.
//    3. Advances _currentSlave to the next address for the following
//       call, via _nextSlave() — but only when NOT in programming
//       mode, so the programming slave keeps getting polled
//       repeatedly without the round-robin moving on.
// ─────────────────────────────────────────────────────────────
void XpressNetRs485::loop() {
    _flushBroadcasts();

    uint8_t addr = (_progModeActive && _progModeSlave)
                 ? _progModeSlave : _currentSlave;

    _pollSlave(addr);

    if (!_progModeActive)
        _currentSlave = _nextSlave(_currentSlave);
}

// Helper function: returns true if any slaves are currently present.
// (Note: this free function currently has no callers in this file —
// hasSlavesPresent() below is called directly instead. Kept here in
// case external code still expects a free-function wrapper.)
static bool _hasPresent(const XpressNetRs485& rs) {
    return rs.hasSlavesPresent();
}

// ─────────────────────────────────────────────────────────────
//  PIO TX/RX primitives
// ─────────────────────────────────────────────────────────────

// _send9() — pushes one raw 9-bit word into the TX state machine's
// FIFO. Bit 8 (0x100) is the XpressNet address/data marker; bits 7-0
// are the payload byte. Blocks (busy-waits) if the FIFO is currently
// full, since the FIFO only holds a few words and the caller must not
// overwrite data that hasn't been shifted out yet.
void XpressNetRs485::_send9(uint16_t word9) {
    while (pio_sm_is_tx_fifo_full(pio2, 0)) {}
    pio_sm_put(pio2, 0, (uint32_t)(word9 & 0x1FF));
}

// _sendAddr() — sends a single "address byte" (bit8=1 set). Used for
// rufbytes (the poll/call byte that starts every bus transaction,
// addressing either a specific slave or the broadcast address).
void XpressNetRs485::_sendAddr(uint8_t byte) {
    _send9((uint16_t)byte | 0x100u);   // D8=1 → address byte
}

// _sendData() — sends a sequence of "data bytes" (bit8=0), i.e. the
// actual XpressNet payload bytes that follow an address byte in a
// frame.
void XpressNetRs485::_sendData(const uint8_t* data, uint8_t len) {
    for (uint8_t i = 0; i < len; i++)
        _send9((uint16_t)data[i]);      // D8=0 → data byte
}

// _read9() — reads one raw word from the RX state machine's FIFO
// without blocking. Returns -1 immediately if the FIFO is currently
// empty (no new bit-serial byte has finished shifting in yet).
// The returned value still needs the address/data marker and payload
// byte extracted from it (see _receiveReply(), which does this via a
// right-shift because the RX program left-justifies the shifted bits
// within the 32-bit word).
int32_t XpressNetRs485::_read9() {
    if (pio_sm_is_rx_fifo_empty(pio2, 1)) return -1;
    return (int32_t)pio_sm_get(pio2, 1);
}

// ─────────────────────────────────────────────────────────────
//  _receiveReply() — wait for and assemble one complete reply frame
//
//  Step 1: waits up to `timeoutUs` microseconds for the very first
//  byte (the frame header) to arrive in the RX FIFO. If nothing
//  arrives within that time, returns 0 immediately (no reply from
//  the polled slave — a normal, expected outcome for addresses with
//  no device attached).
//
//  Step 2: once the header byte has arrived, keeps reading further
//  bytes from the FIFO. Each raw 32-bit FIFO word is unpacked (the RX
//  PIO program shifts bits in from the top, so the 9 bits of
//  interest end up at bits 31-23 of the word) into a 9-bit value:
//  bit8 (d8) is the XpressNet address/data marker, bits 7-0 are the
//  payload byte. Only bytes with d8==0 (data bytes) are stored into
//  _rxBuf; any address-marked byte appearing mid-frame is ignored
//  (it should not normally happen for a well-formed slave reply, but
//  guards against bus glitches).
//
//  As soon as the very first byte has been stored, the expected total
//  frame length is derived from its low nibble (standard XpressNet
//  convention: header's low 4 bits = number of following data bytes),
//  giving expected = 1 (header) + nData + 1 (XOR checksum). Reading
//  stops immediately once that many bytes have been collected — there
//  is no need to wait out any further timeout once a complete,
//  self-consistent frame has arrived.
//
//  If no new byte arrives for more than 300µs while still waiting for
//  more of the frame, the read loop gives up (treating this as an
//  aborted/incomplete transmission) and returns however many bytes
//  were collected so far.
//
//  Returns the number of bytes now stored in _rxBuf (which the caller,
//  _pollSlave(), will only accept as a valid reply after verifying
//  its checksum).
// ─────────────────────────────────────────────────────────────
uint8_t XpressNetRs485::_receiveReply(uint32_t timeoutUs) {
    // Wait for the first byte (header)
    uint32_t t0 = micros();
    while (pio_sm_is_rx_fifo_empty(pio2, 1)) {
        if (micros() - t0 > timeoutUs) return 0;
    }

    // Read bytes — stop as soon as the frame is complete based on the header
    // XpressNet header bits 3-0 = number of data bytes
    // Total frame = 1 (header) + nData + 1 (XOR)
    uint8_t count = 0;
    uint8_t expected = 0;
    uint32_t tByte = micros();

    while (count < XN_RS485_RX_BUF) {
        int32_t raw = _read9();
        if (raw >= 0) {
            uint16_t nine = ((uint32_t)raw >> 23) & 0x1FF;
            uint8_t  d8   = (nine >> 8) & 1;
            uint8_t  byte = nine & 0xFF;
            if (d8 == 0) {
                _rxBuf[count++] = byte;
                if (count == 1)
                    expected = 1 + (byte & 0x0F) + 1;
                if (expected > 0 && count >= expected)
                    break;  // frame complete — stop immediately
            }
            tByte = micros();
        } else {
            if (micros() - tByte > 300) break;  // timeout between bytes
        }
    }
    return count;
}

// ─────────────────────────────────────────────────────────────
//  Poll cycle
// ─────────────────────────────────────────────────────────────

// ─────────────────────────────────────────────────────────────
//  _pollSlave() — poll exactly one slave address for a reply
//
//  1. Bails out immediately for out-of-range addresses (0 or > 31).
//
//  2. Builds a "Normale Anfrage" (normal inquiry) rufbyte for this
//     address, then transmits it: the RX state machine is disabled
//     while transmitting (it would otherwise pick up our own TX
//     signal via the shared bus), the transceiver is switched to
//     transmit mode (_txMode()), and the rufbyte is sent as an
//     address byte.
//
//  3. Waits until the TX FIFO is empty and the shift register has
//     finished (including the stop bit), then switches the DE pin
//     low so the addressed slave is allowed to answer, and waits an
//     additional settling time for the MAX3485E-style transceiver to
//     actually switch direction on the physical bus.
//
//  4. Restarts the RX state machine from a clean state (clears its
//     FIFO, restarts the state machine, and explicitly jumps its
//     program counter back to the wrap target) so it is correctly
//     positioned to wait for a fresh start bit, uncontaminated by
//     whatever was on the line during our own transmission.
//
//  5. Waits for the slave's reply via _receiveReply(), using the
//     configured XN_RS485_REPLY_US timeout.
//
//  6. If no reply arrived (rxLen==0): if this slave was previously
//     marked present, increments its miss counter and — once more
//     than 10 consecutive polls have gone unanswered — marks it
//     absent again (so the discovery/round-robin logic stops wasting
//     time polling it every cycle). Waits the inter-poll settling
//     time and returns.
//
//  7. If a reply DID arrive: marks the slave present, resets its miss
//     counter, and records the current time as lastSeenMs. If the
//     reply's checksum is invalid, the (corrupted) reply is discarded
//     without further processing. Otherwise the reply is handed to
//     _processSlaveReply() for protocol-level handling.
//
//  8. Always waits XN_INTER_POLL_US after finishing, giving the bus a
//     brief settling gap before the next poll begins.
// ─────────────────────────────────────────────────────────────
void XpressNetRs485::_pollSlave(uint8_t addr) {
    if (addr == 0 || addr > XN_RS485_MAX_SLAVES) return;

    // Debug hook: previously used to log every poll for addresses 1..4.
    // Left in place (empty) as a convenient spot to re-enable that
    // logging during future debugging sessions.
    if (addr <= 4) {
    }

    XNSlave& sl = _slaves[addr];
    uint8_t rufbyte = _makeRufbyte(XN_RUF_NORMALE_ANFRAGE, addr);

    // Step 1: send the rufbyte — RX state machine off during TX
    pio_sm_set_enabled(pio2, 1, false);
    _txMode();
    _sendAddr(rufbyte);

    // Wait until the TX FIFO is empty and the shift register has finished (stop bit sent)
    while (!pio_sm_is_tx_fifo_empty(pio2, 0)) tight_loop_contents();
    delayMicroseconds(XN_BYTE_US + 20);

    // DE low — the slave may now answer
    digitalWrite(XN_RS485_DE_PIN, LOW);
    // Settling time for the MAX3485E-style transceiver
    delayMicroseconds(XN_RS485_TX_SETTLE);

    // Restart the RX state machine — reset it back to the start (waiting for a start bit)
    pio_sm_set_enabled(pio2, 1, false);
    pio_sm_clear_fifos(pio2, 1);
    pio_sm_restart(pio2, 1);
    pio_sm_set_enabled(pio2, 1, true);
    // Set the program counter back to the start of the program AFTER the SM is enabled
    pio_sm_exec(pio2, 1, pio_encode_jmp(_rxProgOffset + xn_uart_rx_wrap_target));

    // Wait for a reply — the timeout starts counting from here
    uint8_t rxLen = _receiveReply(XN_RS485_REPLY_US);

    // Debug hook: previously used to log all RX activity, including
    // empty responses, for addresses 1..4. Left in place (empty) as a
    // convenient spot to re-enable that logging during future
    // debugging sessions.
    if (addr <= 4 && rxLen > 0) {
    }

    if (rxLen == 0) {
        if (sl.present && ++sl.missCount > 10) {
            sl.present   = false;
            sl.missCount = 0;
        }
        // Wait for the next poll (DE low → DE high interval)
        delayMicroseconds(XN_INTER_POLL_US);
        return;
    }

    // The slave has answered
    sl.present    = true;
    sl.missCount  = 0;
    sl.lastSeenMs = millis();

    if (!_checkXor(_rxBuf, rxLen)) {
        delayMicroseconds(XN_INTER_POLL_US);
        return;
    }

    _processSlaveReply(addr, _rxBuf, rxLen);
    // Wait for the next poll
    delayMicroseconds(XN_INTER_POLL_US);
}

// ─────────────────────────────────────────────────────────────
//  _processSlaveReply() — handle one validated reply frame
//
//  1. Programming-mode tracking: if the frame's header indicates a
//     CV read/write request (0x22/0x23) and we are not already in
//     programming mode, this slave is recorded as the active
//     programming-mode device (see the loop()/_pollSlave() priority
//     polling described above). If the frame is a "power on" command
//     (0x21 0x81), programming mode is cleared again — leaving
//     programming mode is implied by a normal power-on request.
//
//  2. Lazily creates (on first use) a shared XpressNetHandler instance
//     dedicated to this RS-485 bus, and registers its broadcast
//     callback so that any broadcast the shared protocol logic wants
//     to send (power/estop/clock, 0x42 switching info, 0xE4 loco
//     status, 0x63 CV programming result, etc.) is queued via
//     enqueueBroadcast() rather than sent immediately — RS-485 only
//     allows transmission when it is our turn on the bus.
//
//  3. Passes the slave's previously detected handheld type to the
//     handler (setSlaveType()) so the shared protocol logic can adapt
//     certain responses if needed, then hands the frame to the shared
//     handler's process() function, which does all the actual
//     XpressNet command decoding and returns both a result code and
//     any direct reply bytes.
//
//  4. If the result is REPLY or ERROR (a direct response is expected
//     by the slave that just spoke), the reply is transmitted back to
//     it: the RX state machine is disabled during TX, the transceiver
//     switches to transmit, the reply is addressed using the
//     broadcast/reply rufbyte (0x60 + address) followed by the
//     response payload, and the transceiver switches back to receive
//     mode (clearing/re-enabling the RX state machine) once done.
//
//  5. If the result is BROADCAST_ONLY (the command produced no direct
//     reply, only a broadcast — e.g. a turnout command), the broadcast
//     queue is flushed immediately rather than waiting for the next
//     loop() iteration, so that time-sensitive devices (e.g. a
//     MultiMaus waiting for its own short response timeout) see the
//     resulting state change without unnecessary delay.
// ─────────────────────────────────────────────────────────────
void XpressNetRs485::_processSlaveReply(uint8_t addr,
                                          const uint8_t* buf, uint8_t len) {
    // Generic RX log — same two-point pattern as the other transports
    // (Z21, XpressNet-LAN/USB): every reply reaching this handler from
    // a polled RS485 slave, before any processing (Rob).
    traceLog().logBytes(TraceLevel::DEBUG, TraceSource::XN_RS485, "RX", buf, len);
    uint8_t h = buf[0];

    // Programming-mode tracking
    if ((h == 0x22 || h == 0x23) && !_progModeActive) {
        _progModeActive = true;
        _progModeSlave  = addr;
    }
    if (h == 0x21 && len >= 2 && buf[1] == 0x81) {
        _progModeActive = false;
        _progModeSlave  = 0;
    }

    // Handle via the shared XpressNetHandler
    static XpressNetHandler sRs485Handler;
    static bool sRs485Init = false;
    if (!sRs485Init) {
        sRs485Handler.begin(ModuleId::XPRESSNET_RS485);
        // Broadcast callback for RS-485 — forwards ALL broadcasts from
        // the shared XpressNetHandler (power/estop/clock, 0x42 switching
        // info, 0xE4 loco status, 0x63 CV programming result, etc).
        // enqueueBroadcast() itself enforces the maximum frame length.
        sRs485Handler.setBroadcastCallback([](const uint8_t* data, uint8_t dlen, uint16_t /*locoAddr*/) {
            gXpressNetRs485.enqueueBroadcast(data, dlen);
        });
        sRs485Init = true;
    }
    uint8_t resp[24]; uint8_t respLen = 0;
    // Pass the slave type to the handler for protocol-dependent responses
    sRs485Handler.setSlaveType((uint8_t)_slaves[addr].type);
    // Generic handoff log — RS485 calls the shared XpressNetHandler
    // directly (not via XnInterfaceLayer, unlike LenzLan/LenzUsb), so
    // this sits right before that call rather than inside a shared
    // intermediate layer. buf/len are unchanged from the RX log above
    // (no address remapping needed for this transport, unlike Z21's
    // turnout FAdr conversion) — logged again anyway for consistency
    // with the same two-point pattern used everywhere else (Rob).
    traceLog().logBytes(TraceLevel::DEBUG, TraceSource::XN_RS485, "-> XN", buf, len);
    XNHandleResult r = sRs485Handler.process(buf, len, resp, respLen);

    if ((r == XNHandleResult::REPLY || r == XNHandleResult::ERROR) && respLen > 0) {
        uint8_t replyRuf = _makeRufbyte(0x60, addr);
        pio_sm_set_enabled(pio2, 1, false);
        _txMode();
        _sendAddr(replyRuf);
        _sendData(resp, respLen);
        _rxMode();
        pio_sm_clear_fifos(pio2, 1);
        pio_sm_set_enabled(pio2, 1, true);
    }
    // For BROADCAST_ONLY, flush immediately so the MultiMaus receives the
    // broadcast within its own response timeout
    if (r == XNHandleResult::BROADCAST_ONLY) {
        _flushBroadcasts();
    }
}

// ─────────────────────────────────────────────────────────────
//  Broadcasts
// ─────────────────────────────────────────────────────────────

// ─────────────────────────────────────────────────────────────
//  enqueueBroadcast() — queue a broadcast frame for later transmission
//
//  Rejects zero-length frames and frames longer than XN_BC_MAX_LEN
//  (silently ignored — callers are expected to only ever send
//  correctly-sized XpressNet frames).
//
//  Scans the queue for the first free (not pending) slot and copies
//  the frame into it. If the queue is completely full, the oldest
//  entry (index 0) is dropped by shifting all remaining entries down
//  by one, and the new frame is placed at the end — i.e. the queue
//  behaves as a FIFO that discards the oldest pending broadcast
//  rather than refusing the newest one, on the assumption that
//  it is better to keep the queue moving (recent state changes matter
//  more than ones that are already stale) than to lose the newest
//  update entirely.
// ─────────────────────────────────────────────────────────────
void XpressNetRs485::enqueueBroadcast(const uint8_t* data, uint8_t len) {
    if (!len || len > XN_BC_MAX_LEN) return;
    for (int i = 0; i < XN_BC_QUEUE_SIZE; i++) {
        if (!_bcQueue[i].pending) {
            memcpy(_bcQueue[i].data, data, len);
            _bcQueue[i].len     = len;
            _bcQueue[i].pending = true;
            return;
        }
    }
    // Queue full: drop the oldest entry
    memmove(&_bcQueue[0], &_bcQueue[1],
            sizeof(XNBroadcast) * (XN_BC_QUEUE_SIZE - 1));
    memcpy(_bcQueue[XN_BC_QUEUE_SIZE-1].data, data, len);
    _bcQueue[XN_BC_QUEUE_SIZE-1].len     = len;
    _bcQueue[XN_BC_QUEUE_SIZE-1].pending = true;
}

// sendBroadcastNow() — bypasses the queue entirely and transmits the
// given frame on the bus immediately, repeated XN_RS485_BC_REPEAT
// times (see _sendBroadcast()).
void XpressNetRs485::sendBroadcastNow(const uint8_t* data, uint8_t len) {
    _sendBroadcast(data, len, XN_RS485_BC_REPEAT);
}

// _flushBroadcasts() — transmits every currently pending entry in the
// queue, in order, clearing each entry's pending flag as it is sent.
// Entries that are not pending are skipped. Called once at the start
// of every loop() iteration, and also directly from
// _processSlaveReply() for BROADCAST_ONLY results, so time-sensitive
// broadcasts don't have to wait for the next full loop iteration.
void XpressNetRs485::_flushBroadcasts() {
    for (int i = 0; i < XN_BC_QUEUE_SIZE; i++) {
        if (_bcQueue[i].pending) {
            _sendBroadcast(_bcQueue[i].data, _bcQueue[i].len,
                           XN_RS485_BC_REPEAT);
            _bcQueue[i].pending = false;
        }
    }
}

// ─────────────────────────────────────────────────────────────
//  _sendBroadcast() — transmit one broadcast frame on the bus
//
//  Does nothing if len==0. Disables the RX state machine and switches
//  the transceiver to transmit mode, then sends the frame `repeat`
//  times in a row (repeating the whole address+data sequence provides
//  some resilience against a single corrupted transmission, since
//  XpressNet broadcasts are not individually acknowledged). Each
//  repetition starts with the fixed broadcast rufbyte (0x60,
//  addressing "all devices"), followed by the payload bytes, and
//  waits for the TX FIFO to empty plus one extra byte time before the
//  next repetition (or before switching back to receive mode).
//  Finally switches the transceiver back to receive mode and
//  re-enables the RX state machine with a clean FIFO.
// ─────────────────────────────────────────────────────────────
void XpressNetRs485::_sendBroadcast(const uint8_t* data, uint8_t len,
                                      uint8_t repeat) {
    if (!len) return;
    pio_sm_set_enabled(pio2, 1, false);
    _txMode();
    for (uint8_t r = 0; r < repeat; r++) {
        _sendAddr(0x60);
        _sendData(data, len);
        while (!pio_sm_is_tx_fifo_empty(pio2, 0)) {}
        delayMicroseconds(XN_BYTE_US);
    }
    _rxMode();
    pio_sm_clear_fifos(pio2, 1);
    pio_sm_set_enabled(pio2, 1, true);
}

// ─────────────────────────────────────────────────────────────
//  _onEvent() — EventBus handler: turn system events into broadcasts
//
//  Receives Event structs published by any other module in the
//  system (running on either core), via the shared EventBus. Ignores
//  events whose sourceId is this same module (XPRESSNET_RS485),
//  which prevents a command received ON this bus from being
//  re-broadcast back onto the very same bus a second time (the
//  direct/immediate broadcast already handled inside
//  _processSlaveReply() covers that case).
//
//  For each event type of interest, builds the corresponding fixed
//  XpressNet broadcast frame and queues it via enqueueBroadcast():
//    POWER_ON         → 0x61 0x01 0x60  (track power on / normal operation)
//    POWER_OFF        → 0x61 0x00 0x61  (track power off)
//    EMERGENCY_STOP   → 0x81 0x00 0x81  (all locos stopped)
//    FEEDBACK_BULK    → one or more "Rückmeldung" (§2.4.5/§2.15) frames,
//                       each carrying up to 8 address/data pairs taken
//                       directly from the event's fbBulkAddr/fbBulkData
//                       arrays (already in the correct ADR/ITNZ format
//                       — see rs_bus_hal.cpp). If more than 8 pairs are
//                       reported in one event, multiple broadcast
//                       frames are queued, each with its own header
//                       byte (0x40 + number of data bytes) and XOR
//                       checksum.
//    FEEDBACK         → a single address/data pair packed into one
//                       0x42-headered frame (the "short" form used
//                       when only one module's state changed).
//  Other event types are ignored.
// ─────────────────────────────────────────────────────────────
void XpressNetRs485::_onEvent(const Event& ev) {
    XpressNetRs485& self = gXpressNetRs485;
    // Only broadcast if the event came from a different module
    // (not from ourselves — this prevents duplicate broadcasts)
    if (ev.sourceId == (uint8_t)ModuleId::XPRESSNET_RS485) return;
    if (ev.type == EvType::POWER_ON) {
        static const uint8_t powerOn[]  = { 0x61, 0x01, 0x60 };
        self.enqueueBroadcast(powerOn, sizeof(powerOn));
    } else if (ev.type == EvType::POWER_OFF) {
        static const uint8_t powerOff[] = { 0x61, 0x00, 0x61 };
        self.enqueueBroadcast(powerOff, sizeof(powerOff));
    } else if (ev.type == EvType::EMERGENCY_STOP) {
        static const uint8_t eStop[]    = { 0x81, 0x00, 0x81 };
        self.enqueueBroadcast(eStop, sizeof(eStop));
    } else if (ev.type == EvType::FEEDBACK_BULK) {
        // BC "Rückmeldung" (§2.4.5, format §2.15) — fbBulkAddr/fbBulkData
        // already contain the correct ADR/ITNZ pairs (see rs_bus_hal.cpp).
        // An RS-485 broadcast frame may contain up to 8 pairs (N max 16,
        // §2.4.5); XN_BC_MAX_LEN=24 comfortably allows that in one frame.
        uint8_t i = 0;
        while (i < ev.fbBulkCount) {
            uint8_t pkt[18];
            uint8_t pairCount = 0;
            while (pairCount < 8 && i < ev.fbBulkCount) {
                pkt[1 + pairCount * 2]     = ev.fbBulkAddr[i];
                pkt[1 + pairCount * 2 + 1] = ev.fbBulkData[i];
                pairCount++; i++;
            }
            pkt[0] = 0x40 + pairCount * 2;  // header byte = 0x40+N
            uint8_t xorVal = pkt[0];
            for (uint8_t b = 0; b < pairCount * 2; b++) xorVal ^= pkt[1 + b];
            pkt[1 + pairCount * 2] = xorVal;
            self.enqueueBroadcast(pkt, 1 + pairCount * 2 + 1);
        }
    } else if (ev.type == EvType::FEEDBACK) {
        // Single pair (N=2 → header byte 0x42)
        uint8_t itnz = 0x40 | (ev.fbNibble ? 0x10 : 0x00) | (ev.fbDat & 0x0F);
        uint8_t pkt[] = { 0x42, (uint8_t)ev.fbModule, itnz, (uint8_t)(0x42 ^ ev.fbModule ^ itnz) };
        self.enqueueBroadcast(pkt, sizeof(pkt));
    }
}

// ─────────────────────────────────────────────────────────────
//  Helper functions
// ─────────────────────────────────────────────────────────────

// ─────────────────────────────────────────────────────────────
//  _makeRufbyte() — build one XpressNet "rufbyte" (call byte)
//
//  Rufbyte bit layout: P 1 T1 T0 A4 A3 A2 A1 A0
//    P        = even-parity bit (bit7), computed below
//    1        = fixed marker bit (bit6, part of rufType)
//    T1 T0    = call type (bits 6-5 of rufType: Normale Anfrage /
//               Quittierung / Broadcast)
//    A4..A0   = 5-bit slave address (1-31)
//
//  Combines the call-type bits (masked to bits 6-5) with the address
//  (masked to bits 4-0), then computes even parity over bits 6-0: if
//  the number of set bits in that range is odd, bit7 (P) is set so
//  that the total number of set bits across all 8 bits is even, as
//  required by the XpressNet RS-485 physical-layer parity check.
// ─────────────────────────────────────────────────────────────
uint8_t XpressNetRs485::_makeRufbyte(uint8_t rufType, uint8_t addr) {
    // XpressNet rufbyte format: P 1 0/1 A4 A3 A2 A1 A0
    // P = even parity over bits 6-0
    uint8_t base = (rufType & 0x60) | (addr & 0x1F);
    if ((__builtin_popcount(base & 0x7F)) & 1)
        base |= 0x80;  // odd → set P=1 to make the total even
    return base;
}

// _checkXor() — verifies the trailing XOR checksum of a received
// frame. Returns false if the buffer is shorter than 2 bytes (too
// short to contain both payload and a checksum byte). Otherwise XORs
// all bytes except the last one together and compares the result to
// the last byte (the checksum, per standard XpressNet convention).
bool XpressNetRs485::_checkXor(const uint8_t* buf, uint8_t len) {
    if (len < 2) return false;
    uint8_t x = 0;
    for (uint8_t i = 0; i < len - 1; i++) x ^= buf[i];
    return x == buf[len - 1];
}

// ─────────────────────────────────────────────────────────────
//  _nextSlave() — decide which slave address to poll next
//
//  Strategy: poll present slaves quickly, discover new ones slowly.
//  Every 8th call: try one unknown address (discovery).
//  All other calls: only poll already-present slaves.
//
//  Uses two function-local static variables to remember state between
//  calls: sCycleCount (a simple call counter used to decide when a
//  discovery poll is due) and sDiscoveryAddr (the address to resume
//  discovery scanning from next time).
//
//  On every 8th call (sCycleCount & 0x07 == 0), scans forward from
//  sDiscoveryAddr for the next address that is not currently marked
//  present AND has missed fewer than 3 consecutive polls (limiting
//  how aggressively a genuinely absent/silent address gets re-probed),
//  and returns that address for a one-off discovery poll.
//
//  On all other calls, scans forward from `current` for the next
//  address that IS currently marked present, and returns that —
//  implementing the round-robin "keep polling known devices" cycle.
//
//  If no slave is currently present at all (e.g. right after boot,
//  before anything has answered), falls through to advancing
//  sDiscoveryAddr and returning it, so the bus keeps trying to
//  discover something even outside the normal every-8th-call
//  schedule.
// ─────────────────────────────────────────────────────────────
uint8_t XpressNetRs485::_nextSlave(uint8_t current) const {
    static uint8_t sCycleCount    = 0;
    static uint8_t sDiscoveryAddr = 1;

    sCycleCount++;

    if ((sCycleCount & 0x07) == 0) {
        // Discovery poll: look for the next unknown address
        for (uint8_t i = 0; i < XN_RS485_MAX_SLAVES; i++) {
            sDiscoveryAddr = (sDiscoveryAddr % XN_RS485_MAX_SLAVES) + 1;
            if (!_slaves[sDiscoveryAddr].present &&
                _slaves[sDiscoveryAddr].missCount < 3) {
                return sDiscoveryAddr;
            }
        }
    }

    // Normal case: look for the next present slave
    uint8_t next = current;
    for (uint8_t i = 0; i < XN_RS485_MAX_SLAVES; i++) {
        next = (next % XN_RS485_MAX_SLAVES) + 1;
        if (_slaves[next].present) return next;
    }

    // No present slaves — fall back to discovery
    sDiscoveryAddr = (sDiscoveryAddr % XN_RS485_MAX_SLAVES) + 1;
    return sDiscoveryAddr;
}

// hasSlavesPresent() — scans all 31 possible slave addresses and
// returns true as soon as one is found with present==true. Returns
// false if none of them are currently marked present (e.g. no
// handheld has ever answered, or all have timed out).
bool XpressNetRs485::hasSlavesPresent() const {
    for (uint8_t i = 1; i <= XN_RS485_MAX_SLAVES; i++)
        if (_slaves[i].present) return true;
    return false;
}
