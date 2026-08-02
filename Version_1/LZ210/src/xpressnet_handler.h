#pragma once
// ═══════════════════════════════════════════════════════════════
//  xpressnet_handler.h  —  Shared XpressNet v3.6/v4.0 protocol parser
//
//  Parses and responds to XpressNet messages from all protocol
//  interfaces: LenzLAN (TCP), LenzUSB (serial) and XpressNetRS485.
//  Each interface instantiates its own XpressNetHandler and sets
//  a broadcast callback for sending broadcasts to its clients.
//
//  Usage:
//    XpressNetHandler handler;
//    handler.begin(ModuleId::LENZ_LAN);
//    handler.setBroadcastCallback([](const uint8_t* d, uint8_t l) { ... });
//    uint8_t resp[24]; uint8_t len;
//    XNHandleResult r = handler.process(buf, bufLen, resp, len);
//
//  Shared state:
//    gCentrale       — global station state (power, prog mode, model time)
//    gAccessories[]  — turnout position table (MAX_ACCESSORIES entries)
//    Both are shared across all XpressNetHandler instances.
// ═══════════════════════════════════════════════════════════════
#include "module_arch.h"
#include "command_bus.h"
#include "event_bus.h"
#include "loco_repository.h"

// ─────────────────────────────────────────────────────────────
//  SpeedStepMode — DCC speed step resolution
// ─────────────────────────────────────────────────────────────
enum class SpeedStepMode : uint8_t {
    STEPS_14  = 0,
    STEPS_28  = 2,
    STEPS_128 = 4,
};

// ─────────────────────────────────────────────────────────────
//  XNHandleResult — result of processing one XpressNet message
// ─────────────────────────────────────────────────────────────
enum class XNHandleResult : uint8_t {
    REPLY,          // Send response back to sender
    NO_REPLY,       // No response required
    BROADCAST_ONLY, // Broadcast only, no direct response to sender
    ERROR,          // Error response (resp/respLen contain error frame)
    OK_SILENT,      // Command accepted; no 0x01 0x04 on RS485, but on LAN/USB
};

// ─────────────────────────────────────────────────────────────
//  nemToCanonical() / canonicalToNem() — conversion between the
//  NEM671/DCC-native speed range (RCN-212 compliant: for 14-step
//  0=stop,1=estop,2-15=step; for 28-step 0/1=stop,2/3=estop,4-31=
//  step — NOTE the double reservation for 28-step) and the canonical
//  0-127 convention that Command.speed/LocoBase.speed use everywhere
//  in this codebase (the same convention as LocoNet's SPD byte,
//  regardless of step mode). Symmetric with DccHal::_scaleSpeed()
//  (canonical -> library range). Shared between XpressNetHandler and
//  Z21Lan (which builds its own X-Bus-style loco-info response).
//  mode: 0=14-step, 2=28-step (same meaning as LocoBase.stepMode).
// ─────────────────────────────────────────────────────────────
uint8_t nemToCanonical(uint8_t nem, uint8_t mode);
uint8_t canonicalToNem(uint8_t canon, uint8_t mode);

// ─────────────────────────────────────────────────────────────
//  xn28Unshuffle() / xn28Shuffle() — XpressNet 28-step (and the
//  historical 27-step command 0x11) does NOT use a plain linear
//  0-31 value in the speed byte, but the same bit shuffle as the
//  underlying DCC packet (confirmed against the reference
//  implementation XpressNetMaster/Philipp Gahtow, SetLocoInfo()):
//    wire_bits[4:1] = canonical_bits[3:0], wire_bit0 = canonical_bit4
//  14-step and 128-step do NOT use this shuffle.
// ─────────────────────────────────────────────────────────────
uint8_t xn28Unshuffle(uint8_t wire);
uint8_t xn28Shuffle(uint8_t canon);


// ─────────────────────────────────────────────────────────────
//  XnFrame — internal frame type for transport between modules
// ─────────────────────────────────────────────────────────────
struct XnFrame {
    uint8_t data[24];
    uint8_t len         = 0;
    bool    isBroadcast = false;  // True if this frame should be sent as FF FD
};

// Callback type for broadcasting XpressNet data to clients
using XnBroadcastFn = void(*)(const uint8_t* data, uint8_t len);

// ─────────────────────────────────────────────────────────────
//  CentraleState — global command station state
//
//  Shared across all XpressNetHandler instances. Contains the
//  current power state, programming mode state, model time,
//  CV programming results and SV (system variable) storage.
// ─────────────────────────────────────────────────────────────
struct CentraleState {
    bool     trackPowerOff   = true;   // True if track power is off
    bool     emergencyStop   = false;  // True if emergency stop is active
    bool     shortCircuit    = false;  // True if power-off was caused by a short circuit
                                       // — external GPON messages (e.g. from a Fred via
                                       // LocoNet) must NOT automatically resume the
                                       // station in that case.
    bool     progModeActive  = false;  // True if programming track is active
    bool     progResultReady = false;  // True if CV read result is available
    uint16_t pendingCvAddr   = 0;      // CV address for pending read/write
    uint8_t  pendingCvData   = 0;      // CV data for pending write
    bool     pendingCvWrite  = false;  // True if pending operation is a write
    uint8_t  progResultData  = 0;      // CV value from last successful read
    bool     pomResultReady  = false;  // True if POM read result is available
    uint16_t pomResultLocoAddr = 0;    // Loco address of POM read result
    uint8_t  pomResultData   = 0;      // POM read result value
    uint8_t  sv[256]         = {};     // System variables (SV read/write via 0x63)
    uint8_t  mtDow = 0, mtHour = 0, mtMinute = 0, mtFactor = 0;  // Model time
    bool     mtRunning = false;        // True if model clock is running
    uint16_t buildNr = 0;             // Station firmware build number
    uint8_t  rmVersion = 0, bootloaderVer = 0;  // Firmware version bytes
    uint16_t rmBuildNr = 0;           // Feedback module build number
};

// ─────────────────────────────────────────────────────────────
//  AccessoryState — turnout position table entry
//
//  gAccessories[] is indexed by logical output address (1-based).
//  state bits: bit0=output0 position, bit1=output1 position
//  known: set to true the first time this address is switched
// ─────────────────────────────────────────────────────────────
#define MAX_ACCESSORIES 1024
struct AccessoryState {
    uint8_t state : 2;   // bit0=output0 (thrown), bit1=output1 (straight)
    bool    known : 1;   // Has been switched at least once
    uint8_t mode  : 1;   // 0=DCC, 1=MM (Motorola)
};

extern CentraleState  gCentrale;
extern AccessoryState gAccessories[MAX_ACCESSORIES];

// ─────────────────────────────────────────────────────────────
//  XpressNetHandler — XpressNet message parser and dispatcher
// ─────────────────────────────────────────────────────────────
class XpressNetHandler {
public:
    // Initialises the handler with a module ID for CommandBus dispatch.
    // Must be called before process().
    void begin(uint8_t srcModuleId);

    // Sets the broadcast callback. The callback is called whenever a
    // command generates a broadcast (e.g. power on, loco speed change).
    void setBroadcastCallback(XnBroadcastFn fn) { _broadcastFn = fn; }

    // Sets the slave type hint for RS485 protocol variations.
    // 0=unknown, 1=standard handheld, 2=MultiMaus
    void setSlaveType(uint8_t t) { _currentSlaveType = t; }

    // Calculates XOR checksum over len bytes of buf
    static uint8_t xorCheck(const uint8_t* buf, uint8_t len);

    // Verifies XOR checksum of a received message
    static bool verifyXor(const uint8_t* buf, uint8_t len);

    // Decodes a two-byte XpressNet loco address (ah, al) into a uint16_t DCC address.
    // Short addresses: ah=0x00, al=0x00..0x7F (1-127)
    // Long addresses:  ah=0xC0..0xFF, al=0x00..0xFF (1-9999)
    static uint16_t decodeAddr(uint8_t ah, uint8_t al);

    // Encodes a DCC address into the two-byte XpressNet format (ah, al).
    // Addresses <= 99 are encoded as short; others as long (bit6-7 of ah = 0xC0).
    static void encodeAddr(uint16_t addr, uint8_t& ah, uint8_t& al);

    // Decodes a two-byte XpressNet turnout address (b1, b2) into a
    // 1-based logical output address (1-2048).
    // b1 = decoder address byte, b2 = output/activation byte
    static uint16_t decodeTurnoutAddr(uint8_t b1, uint8_t b2);

    // Parses one XpressNet message and generates a response.
    // input/inputLen: received XpressNet bytes (without LAN header)
    // resp/respLen:   response bytes to send back (without LAN header)
    // Returns XNHandleResult indicating what action to take.
    XNHandleResult process(const uint8_t* input, uint8_t inputLen,
                           uint8_t* resp, uint8_t& respLen);

    // ── Broadcast helpers — call from EventBus handlers ──

    // Broadcasts normal operation (track power on) to all clients
    void broadcastPowerOn();

    // Broadcasts track power off to all clients
    void broadcastPowerOff();

    // Broadcasts emergency stop to all clients
    void broadcastEmergencyStop();

    // Broadcasts programming mode state change to all clients
    void broadcastProgMode();

    // Broadcasts current loco info for the given address
    void broadcastLocoInfo(uint16_t addr);

private:
    XnBroadcastFn _broadcastFn      = nullptr;
    uint8_t       _currentSlaveType = 0;
    uint8_t       _srcId = 0;

    // ── Loco repository helpers ───────────────────────────

    // Updates loco speed, direction and step mode in repository,
    // dispatches LOCO_SPEED command to DCC HAL
    void updateLoco(uint16_t addr, uint8_t speed, uint8_t dir,
                    uint32_t fn, SpeedStepMode mode);

    // Updates loco function state in repository,
    // dispatches LOCO_FUNCTION command to DCC HAL
    void updateLocoFunctions(uint16_t addr, uint32_t fn,
                             const uint8_t* funcExt5 = nullptr);

    // Reads loco data from repository into out. Returns false if not found.
    bool queryLoco(uint16_t addr, LocoBase& out);

    // Returns next occupied loco address after fromAddr (for stack search)
    uint16_t stackSearchNext(uint16_t fromAddr) const;

    // ── Response builders ──────────────────────────────────

    // Builds a 0x01 0x04 (OK) response
    uint8_t buildOk(uint8_t* buf);

    // Builds an error response with the given error code
    uint8_t buildError(uint8_t* buf, uint8_t errCode);

    // Builds an E4 loco info response for the given loco
    uint8_t buildLocoInfo(uint8_t* buf, const LocoBase& loco);

    // Builds an E3 0x52 "Funktionszustand" (on/off) response for F13-F28
    uint8_t buildLocoInfoF13F28(uint8_t* buf, const LocoBase& loco);

    // Builds an E3 0x50 "Funktionsstatus" (momentary) response for F0-F12
    uint8_t buildFuncStatusF0F12(uint8_t* buf, const LocoBase& loco);

    // Builds an E4 0x51 "Funktionsstatus" (momentary) response for F13-F28
    // (includes fixed refresh-mode byte R = 0xF, meaning "refresh F0..F28")
    uint8_t buildFuncStatusF13F28(uint8_t* buf, const LocoBase& loco);

    // Builds an E6 0x54 "Funktionsstatus" (momentary) response for F29-F68
    uint8_t buildFuncStatusF29F68(uint8_t* buf, const LocoBase& loco);

    // Builds an E6 0x53 "Funktionszustand" (on/off) response for F29-F68
    uint8_t buildFuncZustandF29F68(uint8_t* buf, const LocoBase& loco);

    // Packs functions 29-68 (low32 covers F0-31, ext5 covers F32-71) into 5
    // bytes: out[0]=F29-36, out[1]=F37-44, out[2]=F45-52, out[3]=F53-60,
    // out[4]=F61-68. bit0 = lowest function number in each byte's range.
    static void packF29F68(uint32_t low32, const uint8_t ext5[5], uint8_t out[5]);

    // Builds a 0x62 station status response
    uint8_t buildCentraleStatus(uint8_t* buf);

    // Builds a 0x63 0x21 software version response
    uint8_t buildSoftwareVersion(uint8_t* buf);

    // Builds a stack search result response
    uint8_t buildSearchResult(uint8_t* buf, uint16_t addr, uint8_t kind);

    // Builds a switch info (0x42) response
    uint8_t buildSwitchInfo(uint8_t* buf, uint8_t adr,
                            uint8_t nibble, uint8_t state);

    // ── Command handlers ───────────────────────────────────
    // Each handler corresponds to one XpressNet command opcode.
    // Handlers set resp/respLen if a direct reply is needed.

    XNHandleResult handleIfVersion      (uint8_t* r, uint8_t& l);
    XNHandleResult handleExtraVersion   (uint8_t* r, uint8_t& l);
    XNHandleResult handleNumConn        (uint8_t* r, uint8_t& l);
    XNHandleResult handleIfStatus       (uint8_t* r, uint8_t& l);
    XNHandleResult handleXnVersion      (uint8_t* r, uint8_t& l);
    XNHandleResult handleDeviceAddr     (uint8_t* r, uint8_t& l);
    XNHandleResult handlePowerOn        (uint8_t* r, uint8_t& l);
    XNHandleResult handlePowerOff       (uint8_t* r, uint8_t& l);
    XNHandleResult handleEmergStop      (uint8_t* r, uint8_t& l);
    XNHandleResult handleOneLocoStop    (const uint8_t* in, uint8_t il, uint8_t* r, uint8_t& l);
    XNHandleResult handleGetVersion     (uint8_t* r, uint8_t& l);
    XNHandleResult handleGetStatus      (uint8_t* r, uint8_t& l);
    XNHandleResult handleLocoQuery      (const uint8_t* in, uint8_t il, uint8_t* r, uint8_t& l);
    XNHandleResult handleLocoQueryMM    (const uint8_t* in, uint8_t il, uint8_t* r, uint8_t& l);
    XNHandleResult handleLocoSpeed      (const uint8_t* in, uint8_t il, uint8_t* r, uint8_t& l);
    XNHandleResult handleLocoFuncF8     (const uint8_t* in, uint8_t il, uint8_t* r, uint8_t& l);
    XNHandleResult handleLocoBinaryState(const uint8_t* in, uint8_t il, uint8_t* r, uint8_t& l);
    XNHandleResult handleLocoFunc       (const uint8_t* in, uint8_t il, uint8_t* r, uint8_t& l);
    XNHandleResult handleTurnout        (const uint8_t* in, uint8_t il, uint8_t* r, uint8_t& l);
    XNHandleResult handleTurnoutZ21     (const uint8_t* in, uint8_t il, uint8_t* r, uint8_t& l);
    XNHandleResult handleSwitchQuery    (const uint8_t* in, uint8_t il, uint8_t* r, uint8_t& l);
    XNHandleResult handleTurnoutInfo    (const uint8_t* in, uint8_t il, uint8_t* r, uint8_t& l);
    XNHandleResult handleProgRead       (const uint8_t* in, uint8_t il, uint8_t* r, uint8_t& l);
    XNHandleResult handleProgWrite      (const uint8_t* in, uint8_t il, uint8_t* r, uint8_t& l);
    XNHandleResult handleProgResult     (uint8_t* r, uint8_t& l);
    XNHandleResult handlePOM            (const uint8_t* in, uint8_t il, uint8_t* r, uint8_t& l);
    XNHandleResult handleSetStartMode   (const uint8_t* in, uint8_t il, uint8_t* r, uint8_t& l);
    XNHandleResult handleStackSearch    (const uint8_t* in, uint8_t il, uint8_t* r, uint8_t& l);
    XNHandleResult handleStackDelete    (const uint8_t* in, uint8_t il, uint8_t* r, uint8_t& l);
    XNHandleResult handleFuncStatus     (const uint8_t* in, uint8_t il, uint8_t* r, uint8_t& l);
    XNHandleResult handleFuncStatusSet  (const uint8_t* in, uint8_t il, uint8_t* r, uint8_t& l);
    XNHandleResult handleModelTimeGet   (uint8_t* r, uint8_t& l);
    XNHandleResult handleModelTimeSet   (const uint8_t* in, uint8_t il, uint8_t* r, uint8_t& l);
    XNHandleResult handleModelTimeBc    (uint8_t* r, uint8_t& l);
    uint8_t        buildModelTimeBuf    (uint8_t* r);  // fills model time buffer
    XNHandleResult handleFastClockStart (uint8_t* r, uint8_t& l);
    XNHandleResult handleFastClockStop  (uint8_t* r, uint8_t& l);
    XNHandleResult handleReset          (uint8_t* r, uint8_t& l);
    XNHandleResult handleSVRead         (const uint8_t* in, uint8_t il, uint8_t* r, uint8_t& l);
    XNHandleResult handleSVWrite        (const uint8_t* in, uint8_t il, uint8_t* r, uint8_t& l);
};
