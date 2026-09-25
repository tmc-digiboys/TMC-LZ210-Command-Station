#pragma once
#include "module_arch.h"

// ═══════════════════════════════════════════════════════════════
//  CommandBus — protocol-agnostic command routing (core1 → core0)
//
//  Protocol modules (LenzLAN, Z21, XpressNet RS485, LenzUSB) post
//  Commands to the bus. Hardware module handlers on core0 receive
//  and execute them.
//
//  Implementation:
//    Lock-free ring buffer (producer=core1, consumer=core0).
//    core1: dispatch() → _push() → non-blocking, returns false if full
//    core0: poll()     → _pop()  → calls registered handlers
//
//  Loop prevention:
//    Command.sourceId is checked — a module never receives its
//    own commands back, preventing feedback loops.
//
//  Thread safety:
//    dispatch() may be called from both core0 and core1.
//    On core0, handlers are called directly (no ring overhead).
//    On core1, the command is pushed into the ring buffer.
// ═══════════════════════════════════════════════════════════════

#define CMD_BUS_RING_SIZE    32   // Maximum commands in the ring buffer
#define CMD_BUS_MAX_HANDLERS  8   // Maximum registered handlers
#define CMD_BUS_DROP_LOG_SIZE 16  // Recent dropped-command entries kept for the web interface

// ─────────────────────────────────────────────────────────────
//  DispatchDropEntry — one record of a command that _push() had to
//  drop because the ring buffer was full (see CommandBus::dispatch()/
//  _push()/_recordDrop()). Kept for the web interface's diagnostic
//  log (see webserver.cpp's _panelSysteem()) so a dropped command —
//  which the sending client is never otherwise informed of, beyond
//  the handful of XpressNet handlers that check dispatch()'s return
//  value and reply with an error instead — is at least visible
//  somewhere for troubleshooting.
// ─────────────────────────────────────────────────────────────
struct DispatchDropEntry {
    uint32_t ms;         // millis() at the time of the drop
    uint8_t  sourceId;   // ModuleId that attempted the dispatch
    CmdType  type;       // Command type that was dropped
};

typedef void (*CmdHandler)(const Command& cmd);

class CommandBus {
public:
    // instance() — returns the single shared CommandBus instance
    // (backed by the file-scope global _gCommandBus defined in
    // command_bus.cpp). Every caller on either core uses this same
    // instance, so there is exactly one ring buffer and one set of
    // registered handlers for the whole system.
    static CommandBus& instance();

    // begin() — resets the handler count and the ring buffer's
    // read/write indices to zero. Must be called once, before any
    // registerHandler() or dispatch() calls are made, to put the bus
    // into a known, empty starting state.
    void begin();

    // Registers a handler function for a given module ID.
    // Call this in the module's begin() method.
    // Multiple handlers may be registered; all are called on dispatch.
    // Returns false if the handler table is full.
    bool registerHandler(uint8_t moduleId, CmdHandler handler,
                         ModuleCore core = ModuleCore::CORE0);

    // Dispatches a command to all registered handlers.
    // Thread-safe: may be called from both core0 and core1.
    // On core0: handlers are called directly.
    // On core1: command is pushed into the lock-free ring buffer.
    // Returns false if the ring buffer is full (command dropped).
    bool dispatch(const Command& cmd);

    // Processes all pending commands from the ring buffer.
    // Must be called repeatedly on core0 (via ModuleRegistry::loopCore0).
    void poll();

    // ── Diagnostic drop log (web interface) ──────────────────
    //
    // Every command _push() ever had to drop because the ring buffer
    // was full is counted here (dropTotal(), a lifetime counter since
    // boot) and, for the CMD_BUS_DROP_LOG_SIZE most recent drops,
    // recorded with a timestamp/source/type (dropLogCount()/
    // dropLogEntry()) — see _recordDrop(). Both the recording
    // (_push(), called from dispatch() only on core1 — see dispatch()'s
    // comment) and the reading (Webserver's _panelSysteem(), also
    // core1) happen on the same core, so no cross-core synchronisation
    // is needed for this diagnostic log specifically, unlike the main
    // ring buffer itself.

    // Total number of commands dropped since boot (not reset when the
    // detailed log below wraps around).
    uint32_t dropTotal() const { return _dropTotal; }

    // Number of entries currently held in the detailed log (0 to
    // CMD_BUS_DROP_LOG_SIZE).
    uint8_t dropLogCount() const { return _dropLogCount; }

    // Returns the i-th most recent drop (0 = most recent). i must be
    // less than dropLogCount().
    const DispatchDropEntry& dropLogEntry(uint8_t i) const;

public:
    CommandBus() = default;
private:

    struct HandlerEntry {
        uint8_t    moduleId;   // Module that owns this handler
        CmdHandler handler;    // Handler function pointer
        ModuleCore core;       // Core the handler runs on
    };

    HandlerEntry     _handlers[CMD_BUS_MAX_HANDLERS];
    uint8_t          _handlerCount = 0;

    // Lock-free ring buffer (core1 writes, core0 reads)
    Command          _ring[CMD_BUS_RING_SIZE];
    volatile uint8_t _head = 0;   // Write index (core1)
    volatile uint8_t _tail = 0;   // Read index (core0)

    // Diagnostic drop log — see the public accessors above. A small
    // fixed-size ring, newest entry overwriting the oldest once full;
    // _dropLogHead is the index the NEXT entry will be written to.
    DispatchDropEntry _dropLog[CMD_BUS_DROP_LOG_SIZE];
    uint8_t           _dropLogHead  = 0;
    uint8_t           _dropLogCount = 0;
    uint32_t          _dropTotal    = 0;

    // Records one dropped command into the diagnostic log above.
    // Called from _push() exactly when it is about to return false.
    void _recordDrop(const Command& cmd);

    // _push() — writes one command into the next free ring-buffer
    // slot (indexed by _head) and only then advances _head, so a
    // concurrent reader on the other core never observes a partially
    // written slot. Returns false without writing anything if the
    // buffer is currently full (i.e. advancing _head would make it
    // equal to _tail).
    bool _push(const Command& cmd);

    // _pop() — reads the command at the current _tail position (if
    // any) into the caller-supplied reference and advances _tail.
    // Returns false, leaving `cmd` untouched, if the buffer is
    // currently empty (i.e. _head == _tail).
    bool _pop(Command& cmd);

    // _deliverToHandlers() — calls every registered handler with the
    // given command, except the handler whose registered moduleId
    // matches the command's own sourceId, which is skipped to prevent
    // a module from receiving back the very command it just
    // dispatched itself.
    void _deliverToHandlers(const Command& cmd);
};

// ─────────────────────────────────────────────────────────────
//  Cmd namespace — helper functions to build and dispatch Commands
//
//  Usage example:
//    CommandBus::instance().dispatch(
//        Cmd::locoSpeed(ModuleId::LENZ_LAN, 3, 64, true, 128));
// ─────────────────────────────────────────────────────────────
namespace Cmd {
    // locoSpeed() — builds a loco speed/direction command: sets the
    // canonical speed (0-127), direction, and DCC step mode for the
    // given loco address.
    inline Command locoSpeed(uint8_t src, uint16_t addr,
                              uint8_t speed, bool fwd, uint8_t mode) {
        Command c{};
        c.type = CmdType::LOCO_SPEED; c.sourceId = src;
        c.address = addr; c.speed = speed;
        c.forward = fwd;  c.stepMode = mode;
        return c;
    }
    // locoFunction() — builds a loco function command: sets a single
    // function (0-68) of the given loco address on or off.
    inline Command locoFunction(uint8_t src, uint16_t addr,
                                 uint8_t fn, bool state) {
        Command c{};
        c.type = CmdType::LOCO_FUNCTION; c.sourceId = src;
        c.address = addr; c.fn = fn; c.fnState = state;
        return c;
    }
    // accessory() — builds an accessory (turnout) command: sets the
    // requested output address (1-2048) to the given output/direction
    // value, with `active` indicating whether the coil should be
    // energised (true) or released (false).
    inline Command accessory(uint8_t src, uint16_t addr,
                              uint8_t output, bool active) {
        Command c{};
        c.type = CmdType::ACCESSORY_SET; c.sourceId = src;
        c.address = addr; c.output = output; c.active = active;
        return c;
    }
    // power() — builds a track power command (on=true for power on,
    // false for power off), selecting CmdType::POWER_ON or
    // CmdType::POWER_OFF accordingly.
    inline Command power(uint8_t src, bool on) {
        Command c{};
        c.type = on ? CmdType::POWER_ON : CmdType::POWER_OFF;
        c.sourceId = src;
        return c;
    }
    // estop() — builds an emergency-stop command. With the default
    // addr=0, this builds a global EMERGENCY_STOP command (stopping
    // all locos); if a specific loco address is given instead, it
    // builds a LOCO_ESTOP command targeting only that one loco.
    inline Command estop(uint8_t src, uint16_t addr = 0) {
        Command c{};
        c.type = addr ? CmdType::LOCO_ESTOP : CmdType::EMERGENCY_STOP;
        c.sourceId = src; c.address = addr;
        return c;
    }
    // cvRead() — builds a CV read command for service-mode
    // programming, requesting the value of the given CV number.
    inline Command cvRead(uint8_t src, uint16_t cv) {
        Command c{};
        c.type = CmdType::CV_READ; c.sourceId = src; c.cv = cv;
        return c;
    }
    // cvWrite() — builds a CV write command for service-mode
    // programming, requesting the given CV number be set to val.
    inline Command cvWrite(uint8_t src, uint16_t cv, uint8_t val) {
        Command c{};
        c.type = CmdType::CV_WRITE; c.sourceId = src;
        c.cv = cv; c.cvValue = val;
        return c;
    }
    // pomWrite() — builds a POM (Programming On Main) write command,
    // requesting that CV `cv` be set to `val` on the loco at `addr`
    // while it remains on the main track.
    inline Command pomWrite(uint8_t src, uint16_t addr,
                             uint16_t cv, uint8_t val) {
        Command c{};
        c.type = CmdType::POM_WRITE; c.sourceId = src;
        c.address = addr; c.cv = cv; c.cvValue = val;
        return c;
    }
}
