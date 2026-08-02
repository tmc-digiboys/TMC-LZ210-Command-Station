#include "command_bus.h"

// ─────────────────────────────────────────────────────────────
//  Singleton instance
//
//  _gCommandBus is the one and only CommandBus object that exists;
//  instance() simply returns a reference to it, giving every caller
//  (on either core) access to the same ring buffer and handler table.
// ─────────────────────────────────────────────────────────────
CommandBus _gCommandBus;

CommandBus& CommandBus::instance() {
    return _gCommandBus;
}

#include "hardware/sync.h"

// ─────────────────────────────────────────────────────────────
//  begin() — initialise ring buffer and handler table
//
//  Resets the handler count to 0 (forgetting any previously
//  registered handlers) and resets both the read (_tail) and write
//  (_head) ring-buffer indices to 0, giving an empty buffer.
// ─────────────────────────────────────────────────────────────
void CommandBus::begin() {
    _handlerCount = 0;
    _head = 0;
    _tail = 0;
}

// ─────────────────────────────────────────────────────────────
//  registerHandler() — register a command handler for a module
//
//  Appends a new {moduleId, handler, core} entry to the handler table
//  and increments the handler count. Should be called in the module's
//  begin() method. Returns false without registering anything if the
//  handler table is already full (CMD_BUS_MAX_HANDLERS reached).
// ─────────────────────────────────────────────────────────────
bool CommandBus::registerHandler(uint8_t moduleId, CmdHandler handler,
                                  ModuleCore core) {
    if (_handlerCount >= CMD_BUS_MAX_HANDLERS) return false;
    _handlers[_handlerCount++] = { moduleId, handler, core };
    return true;
}

// ─────────────────────────────────────────────────────────────
//  _push() — write a command into the ring buffer (core1)
//
//  Computes what the next write index would be (wrapping around at
//  CMD_BUS_RING_SIZE) and, if that would collide with the current
//  read index (_tail), the buffer is full: returns false without
//  writing anything, meaning the command is dropped by the caller.
//  Otherwise copies the command into the current _head slot, issues a
//  data memory barrier (__dmb) to guarantee that this write is visible
//  to core0 before _head is advanced, and only then updates _head to
//  the next index. Non-blocking.
// ─────────────────────────────────────────────────────────────
bool CommandBus::_push(const Command& cmd) {
    uint8_t next = (_head + 1) % CMD_BUS_RING_SIZE;
    if (next == _tail) return false;  // ring full
    _ring[_head] = cmd;
    __dmb();
    _head = next;
    return true;
}

// ─────────────────────────────────────────────────────────────
//  _pop() — read a command from the ring buffer (core0)
//
//  If _head equals _tail, the buffer is currently empty: returns
//  false without touching `cmd`. Otherwise copies the command at the
//  current _tail slot into the caller-supplied reference, issues a
//  data memory barrier (__dmb) before advancing the read index (so
//  the copy is guaranteed complete before the slot could be
//  overwritten by a subsequent _push() on the other core), then
//  advances _tail (wrapping around at CMD_BUS_RING_SIZE). Non-blocking.
// ─────────────────────────────────────────────────────────────
bool CommandBus::_pop(Command& cmd) {
    if (_head == _tail) return false;
    cmd = _ring[_tail];
    __dmb();
    _tail = (_tail + 1) % CMD_BUS_RING_SIZE;
    return true;
}

// ─────────────────────────────────────────────────────────────
//  dispatch() — send a command to all registered handlers
//
//  Thread-safe: may be called from both cores. Checks which core it
//  is currently running on (get_core_num()):
//    On core0: handlers are invoked immediately and directly, via
//    _deliverToHandlers() — there is no need to go through the ring
//    buffer at all, since we are already on the consuming core.
//    Always returns true in this case.
//    On core1: the command cannot be safely delivered directly (the
//    handlers expect to run on core0), so it is instead pushed into
//    the ring buffer via _push(), to be picked up and delivered later
//    by a poll() call on core0. Returns whatever _push() returns
//    (false if the ring buffer was full and the command had to be
//    dropped).
// ─────────────────────────────────────────────────────────────
bool CommandBus::dispatch(const Command& cmd) {
    if (get_core_num() == 0) {
        _deliverToHandlers(cmd);
        return true;
    }
    // Core1: push into ring buffer for core0 to process
    return _push(cmd);
}

// ─────────────────────────────────────────────────────────────
//  poll() — process pending commands from the ring buffer
//
//  Must be called repeatedly on core0, typically from
//  ModuleRegistry::loopCore0(). Repeatedly pops commands from the
//  ring buffer (each one having been pushed there by a dispatch()
//  call made from core1) until the buffer is empty, delivering each
//  one to the registered handlers via _deliverToHandlers() as it is
//  popped. Processes every currently queued command in one call.
// ─────────────────────────────────────────────────────────────
void CommandBus::poll() {
    Command cmd;
    while (_pop(cmd)) {
        _deliverToHandlers(cmd);
    }
}

// ─────────────────────────────────────────────────────────────
//  _deliverToHandlers() — call all handlers except the sender
//
//  Iterates over every registered handler and calls it with the given
//  command, except the handler whose registered moduleId matches
//  cmd.sourceId — that one is skipped, preventing the module that
//  originally dispatched this command from receiving it back as if it
//  came from somewhere else (which would otherwise create a feedback
//  loop, e.g. a protocol module re-processing its own just-issued
//  command).
// ─────────────────────────────────────────────────────────────
void CommandBus::_deliverToHandlers(const Command& cmd) {
    for (uint8_t i = 0; i < _handlerCount; i++) {
        if (_handlers[i].moduleId == cmd.sourceId) continue;
        _handlers[i].handler(cmd);
    }
}
