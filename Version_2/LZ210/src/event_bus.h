#pragma once
#include "module_arch.h"

// ═══════════════════════════════════════════════════════════════
//  EventBus — hardware event distribution (core0 → core1)
//
//  Hardware modules on core0 publish Events (power changes, loco
//  state updates, RS-Bus feedback, short circuits). Protocol modules
//  on core1 receive them via poll() and forward them to clients.
//
//  Implementation:
//    Lock-free ring buffer (producer=core0, consumer=core1).
//    core0: publish() → _push() → non-blocking
//    core1: poll()    → _pop()  → calls all registered handlers
//
//  Like CommandBus, EventBus skips the handler whose registered
//  moduleId matches the event's own sourceId, so a module never
//  receives its own events back (see poll()). Every OTHER registered
//  handler does receive the event, since events are typically meant
//  to be broadcast to all interested protocol modules at once.
// ═══════════════════════════════════════════════════════════════

#define EVT_BUS_RING_SIZE    32   // Maximum events in the ring buffer
#define EVT_BUS_MAX_HANDLERS 12   // Maximum registered handlers

typedef void (*EvHandler)(const Event& ev);

class EventBus {
public:
    // instance() — returns the single shared EventBus instance
    // (backed by the file-scope global _gEventBus defined in
    // event_bus.cpp). All publishers and subscribers use this same
    // instance, so there is exactly one ring buffer and one set of
    // registered handlers for the whole system.
    static EventBus& instance();

    // begin() — resets the handler count and the ring buffer's
    // read/write indices to zero. Must be called once before any
    // publish() or registerHandler() calls are made, to put the bus
    // into a known, empty starting state.
    void begin();

    // registerHandler() — subscribes a module to receive events.
    // Call this in the module's begin() method on core1.
    // Returns false if the handler table is full.
    bool registerHandler(uint8_t moduleId, EvHandler handler);

    // publish() — posts an event from core0 — non-blocking, lock-free.
    // Returns false if the ring buffer is full (event dropped).
    bool publish(const Event& ev);

    // poll() — processes all pending events from the ring buffer.
    // Must be called repeatedly on core1, typically from loop1().
    void poll();

private:
    public:
    EventBus() = default;

    struct HandlerEntry {
        uint8_t   moduleId;   // Module that owns this handler
        EvHandler handler;    // Handler function pointer
    };

    HandlerEntry     _handlers[EVT_BUS_MAX_HANDLERS];
    uint8_t          _handlerCount = 0;

    // Lock-free ring buffer (core0 writes, core1 reads)
    Event            _ring[EVT_BUS_RING_SIZE];
    volatile uint8_t _head = 0;   // Write index (core0)
    volatile uint8_t _tail = 0;   // Read index (core1)

    // _push() — writes one event into the next free ring-buffer slot
    // (indexed by _head) and only then advances _head, so a
    // concurrent reader on the other core never observes a partially
    // written slot. Returns false without writing anything if the
    // buffer is currently full (i.e. advancing _head would make it
    // equal to _tail).
    bool _push(const Event& ev);

    // _pop() — reads the event at the current _tail position (if any)
    // into the caller-supplied reference and advances _tail. Returns
    // false, leaving `ev` untouched, if the buffer is currently empty
    // (i.e. _head == _tail).
    bool _pop(Event& ev);
};

// ─────────────────────────────────────────────────────────────
//  Ev namespace — helper functions to build and publish Events
//
//  Usage example:
//    EventBus::instance().publish(Ev::power(ModuleId::DCC_HAL, true));
// ─────────────────────────────────────────────────────────────
namespace Ev {
    // power() — builds a track power event (on=true for power on,
    // false for power off). Sets sourceId to the given module and
    // selects EvType::POWER_ON or EvType::POWER_OFF accordingly.
    inline Event power(uint8_t src, bool on) {
        Event e{}; e.sourceId = src;
        e.type = on ? EvType::POWER_ON : EvType::POWER_OFF;
        return e;
    }
    // estop() — builds an emergency-stop event (affecting all locos).
    inline Event estop(uint8_t src) {
        Event e{}; e.sourceId = src;
        e.type = EvType::EMERGENCY_STOP;
        return e;
    }
    // shortCircuit() — builds a short-circuit event, signalling that
    // the DCC output has detected an overcurrent condition.
    inline Event shortCircuit(uint8_t src) {
        Event e{}; e.sourceId = src;
        e.type = EvType::SHORT_CIRCUIT;
        return e;
    }
    // locoState() — builds a loco state-change event, carrying the
    // loco's address, its canonical speed (0-127), direction, and its
    // current function bitmap, so subscribing protocol modules can
    // broadcast the update to their own clients.
    inline Event locoState(uint8_t src, uint16_t addr,
                            uint8_t speed, bool fwd, uint32_t fn) {
        Event e{}; e.sourceId = src;
        e.type = EvType::LOCO_STATE;
        e.address = addr; e.speed = speed;
        e.forward = fwd;  e.functions = fn;
        return e;
    }
    // accessoryState() — builds an accessory (turnout) state-change
    // event, carrying the output address, the switched output number,
    // and its new state.
    inline Event accessoryState(uint8_t src, uint16_t addr,
                                 uint8_t output, bool state) {
        Event e{}; e.sourceId = src;
        e.type = EvType::ACCESSORY_STATE;
        e.address = addr; e.output = output; e.state = state;
        return e;
    }
    // feedback() — builds a single RS-Bus feedback event for one
    // module/nibble pair, carrying the module address, which nibble
    // changed, and its new 4-bit value.
    inline Event feedback(uint8_t src, uint16_t module,
                           uint8_t nibble, uint8_t dat, bool ext) {
        Event e{}; e.sourceId = src;
        e.type = ext ? EvType::FEEDBACK : EvType::FEEDBACK;
        e.fbExt = ext;
        e.fbModule = module; e.fbNibble = nibble; e.fbDat = dat;
        return e;
    }
    // feedbackBulk() — builds a bulk RS-Bus feedback event covering up
    // to 28 address/data pairs in a single event, copying the given
    // addr/data arrays into the event's fixed-size fbBulkAddr/
    // fbBulkData buffers (silently truncating to 28 pairs if more are
    // supplied).
    inline Event feedbackBulk(uint8_t src,
                               const uint8_t* addr, const uint8_t* data,
                               uint8_t count) {
        Event e{}; e.sourceId = src;
        e.type        = EvType::FEEDBACK_BULK;
        e.fbBulkCount = count > 28 ? 28 : count;
        memcpy(e.fbBulkAddr, addr, e.fbBulkCount);
        memcpy(e.fbBulkData, data, e.fbBulkCount);
        return e;
    }
    // cvOk() — builds a successful CV-read event, reporting the CV
    // number and the value that was read back during service-mode
    // programming.
    inline Event cvOk(uint8_t src, uint16_t cv, uint8_t val) {
        Event e{}; e.sourceId = src;
        e.type = EvType::CV_OK; e.cv = cv; e.cvValue = val;
        return e;
    }
    // cvNofound() — builds a CV-not-found event, reporting that no
    // acknowledgement was received from the decoder during a
    // service-mode CV read/verify attempt.
    inline Event cvNofound(uint8_t src) {
        Event e{}; e.sourceId = src; e.type = EvType::CV_NOFOUND;
        return e;
    }
    // pomOk() — builds a successful POM (Programming On Main) write
    // event, reporting the loco address, CV number, and value written.
    inline Event pomOk(uint8_t src, uint16_t addr,
                        uint16_t cv, uint8_t val) {
        Event e{}; e.sourceId = src;
        e.type = EvType::POM_OK;
        e.address = addr; e.cv = cv; e.cvValue = val;
        return e;
    }
    // progModeOn() — builds an event announcing that programming
    // (service) mode has just been activated.
    inline Event progModeOn(uint8_t src) {
        Event e{}; e.sourceId = src; e.type = EvType::PROG_MODE_ON;
        return e;
    }
    // progModeOff() — builds an event announcing that programming
    // (service) mode has just been deactivated.
    inline Event progModeOff(uint8_t src) {
        Event e{}; e.sourceId = src; e.type = EvType::PROG_MODE_OFF;
        return e;
    }
}
