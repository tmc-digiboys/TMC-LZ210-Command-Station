// ═══════════════════════════════════════════════════════════════
//  event_bus.cpp  —  EventBus implementation
//
//  Singleton lock-free ring buffer for distributing hardware events
//  from core0 to protocol modules on core1.
// ═══════════════════════════════════════════════════════════════
#include "event_bus.h"

// ─────────────────────────────────────────────────────────────
//  Singleton instance
//
//  _gEventBus is the one and only EventBus object that exists;
//  instance() simply returns a reference to it, giving every caller
//  (on either core) access to the same ring buffer and handler table.
// ─────────────────────────────────────────────────────────────
EventBus _gEventBus;

EventBus& EventBus::instance() {
    return _gEventBus;
}

#include "hardware/sync.h"

// ─────────────────────────────────────────────────────────────
//  begin() — initialise the ring buffer and clear all handlers
//
//  Resets the handler count to 0 (forgetting any previously
//  registered handlers) and resets both the read (_tail) and write
//  (_head) ring-buffer indices to 0, giving an empty buffer. Must be
//  called before any publish() or registerHandler() calls.
// ─────────────────────────────────────────────────────────────
void EventBus::begin() {
    _handlerCount = 0;
    _head = 0;
    _tail = 0;
}

// ─────────────────────────────────────────────────────────────
//  registerHandler() — subscribe a module to receive events
//
//  Appends a new {moduleId, handler} entry to the handler table and
//  increments the handler count. Call this in the module's begin()
//  method on core1. Returns false without registering anything if the
//  handler table is already full (EVT_BUS_MAX_HANDLERS reached).
// ─────────────────────────────────────────────────────────────
bool EventBus::registerHandler(uint8_t moduleId, EvHandler handler) {
    if (_handlerCount >= EVT_BUS_MAX_HANDLERS) return false;
    _handlers[_handlerCount++] = { moduleId, handler };
    return true;
}

// ─────────────────────────────────────────────────────────────
//  _push() — write an event into the ring buffer (core0)
//
//  Computes what the next write index would be (wrapping around at
//  EVT_BUS_RING_SIZE) and, if that would collide with the current
//  read index (_tail), the buffer is full: returns false without
//  writing anything, meaning the event is dropped by the caller.
//  Otherwise copies the event data into the current _head slot,
//  issues a data memory barrier (__dmb) to guarantee that this write
//  is visible to core1 before _head is advanced, and only then
//  updates _head to the next index. Non-blocking.
// ─────────────────────────────────────────────────────────────
bool EventBus::_push(const Event& ev) {
    uint8_t next = (_head + 1) % EVT_BUS_RING_SIZE;
    if (next == _tail) return false;  // ring buffer full
    _ring[_head] = ev;
    __dmb();
    _head = next;
    return true;
}

// ─────────────────────────────────────────────────────────────
//  _pop() — read an event from the ring buffer (core1)
//
//  If _head equals _tail, the buffer is currently empty: returns
//  false without touching `ev`. Otherwise copies the event at the
//  current _tail slot into the caller-supplied reference, issues a
//  data memory barrier (__dmb) before advancing the read index (so
//  the copy is guaranteed complete before the slot could be
//  overwritten by a subsequent _push() on the other core), then
//  advances _tail (wrapping around at EVT_BUS_RING_SIZE). Non-blocking.
// ─────────────────────────────────────────────────────────────
bool EventBus::_pop(Event& ev) {
    if (_head == _tail) return false;
    ev = _ring[_tail];
    __dmb();
    _tail = (_tail + 1) % EVT_BUS_RING_SIZE;
    return true;
}

// ─────────────────────────────────────────────────────────────
//  publish() — post an event from core0 to the ring buffer
//
//  Thin wrapper around _push(). Non-blocking, lock-free. Returns
//  false if the ring is full, in which case the event is silently
//  dropped. Intended to be called from core0 hardware modules whenever
//  something worth reporting to core1 happens (power state changes,
//  loco updates, feedback, short circuits, etc.).
// ─────────────────────────────────────────────────────────────
bool EventBus::publish(const Event& ev) {
    return _push(ev);
}

// ─────────────────────────────────────────────────────────────
//  poll() — deliver all pending events to registered handlers
//
//  Must be called repeatedly on core1 (from loopCore1). Repeatedly
//  pops events from the ring buffer until it is empty; for each
//  popped event, iterates over every registered handler and calls it
//  with that event — except the handler whose registered moduleId
//  matches the event's own sourceId, which is skipped. This prevents
//  a module from receiving back the very event it just published
//  itself, while every other subscribed module still receives it
//  normally.
// ─────────────────────────────────────────────────────────────
void EventBus::poll() {
    Event ev;
    while (_pop(ev)) {
        for (uint8_t i = 0; i < _handlerCount; i++) {
            if (_handlers[i].moduleId == ev.sourceId) continue;
            _handlers[i].handler(ev);
        }
    }
}
