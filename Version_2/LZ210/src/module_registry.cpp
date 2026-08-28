// ═══════════════════════════════════════════════════════════════
//  module_registry.cpp  —  Module lifecycle and resource manager
//
//  Manages registration, startup and loop execution of all modules.
//  Detects GPIO pin and PIO state machine conflicts at registration.
//  Routes core0/core1 lifecycle calls to the correct modules.
// ═══════════════════════════════════════════════════════════════
#include <Ethernet.h>
#include "trace_log.h"
#include "event_bus.h"
#include "command_bus.h"
#include "module_registry.h"
extern bool gEthernetUsesDhcp;

// ─────────────────────────────────────────────────────────────
//  Singleton instance
// ─────────────────────────────────────────────────────────────
ModuleRegistry _gRegistry;

ModuleRegistry& ModuleRegistry::instance() {
    return _gRegistry;
}

// ─────────────────────────────────────────────────────────────
//  add() — register a module with the registry
//
//  Checks for duplicate module IDs, GPIO pin conflicts and PIO
//  state machine conflicts before accepting the module.
//  Claims the module's resources on success.
//  Returns false if the table is full, module is null, the ID
//  is already registered, or a resource conflict is detected.
//
//  Order of checks: table capacity (MODULE_REGISTRY_MAX) → null
//  pointer → duplicate module ID (via find()) → pin conflict →
//  PIO state-machine conflict. Only once every check has passed are
//  the module's resources actually claimed (_claimResources()) and
//  the module appended to _modules[], with _count incremented. Any
//  failed check leaves the registry completely unchanged.
// ─────────────────────────────────────────────────────────────
bool ModuleRegistry::add(ModuleBase* module) {
    if (_count >= MODULE_REGISTRY_MAX) return false;
    if (!module)                        return false;
    if (find(module->id()))             return false;  // duplicate ID

    if (!_checkPinConflict(module)) return false;
    if (!_checkPioConflict(module)) return false;

    _claimResources(module);
    _modules[_count++] = module;
    return true;
}

// ─────────────────────────────────────────────────────────────
//  _checkPinConflict() — verify no GPIO pin overlap
//
//  Retrieves the GPIO pin list from the module and checks each
//  pin against the bitmask of already-claimed pins.
//  Non-hardware modules always pass (return true).
//
//  Casts the module to HardwareModule (safe here since
//  isHardwareModule() was already checked) and calls its usedPins()
//  to obtain the pin array and count. For each pin below 64 (the
//  bitmask width), checks whether the corresponding bit is already
//  set in _usedPins; if so, a conflict exists and the function
//  returns false immediately without checking any remaining pins.
//  Pins numbered 64 or above are silently skipped (cannot be
//  represented in the 64-bit _usedPins mask) rather than causing a
//  spurious conflict or out-of-range access.
// ─────────────────────────────────────────────────────────────
bool ModuleRegistry::_checkPinConflict(ModuleBase* m) {
    if (!m->isHardwareModule()) return true;
    HardwareModule* hw = static_cast<HardwareModule*>(m);
    if (!hw) return true;

    const uint8_t* pins; uint8_t count;
    hw->usedPins(pins, count);
    for (uint8_t i = 0; i < count; i++) {
        if (pins[i] < 64 && (_usedPins >> pins[i]) & 1) return false;
    }
    return true;
}

// ─────────────────────────────────────────────────────────────
//  _checkPioConflict() — verify no PIO state machine overlap
//
//  Retrieves the PIO SM bitmask from the module and checks for
//  overlap with already-claimed state machines.
//  Non-hardware modules always pass (return true).
//
//  Casts the module to HardwareModule and calls usedPioSmMask() to
//  get its requested state-machine bitmask in one call (rather than
//  an array/count pair, since there are only a small, fixed number of
//  PIO state machines in total). A conflict exists if the bitwise AND
//  of this requested mask and the already-claimed _usedPioSMs mask is
//  non-zero — i.e. at least one requested state machine is already in
//  use by a previously registered module.
// ─────────────────────────────────────────────────────────────
bool ModuleRegistry::_checkPioConflict(ModuleBase* m) {
    if (!m->isHardwareModule()) return true;
    HardwareModule* hw = static_cast<HardwareModule*>(m);
    if (!hw) return true;
    uint16_t mask = hw->usedPioSmMask();
    return (_usedPioSMs & mask) == 0;
}

// ─────────────────────────────────────────────────────────────
//  _claimResources() — mark a module's GPIO pins and PIO SMs as used
//
//  Called after conflict checks pass. Updates the internal bitmasks
//  so subsequent registrations detect any overlap.
//
//  Does nothing for non-HardwareModules (they have no GPIO/PIO
//  resources to claim). For a HardwareModule, sets the bit
//  corresponding to each of its reported GPIO pins (below 64) in
//  _usedPins, and ORs its usedPioSmMask() into _usedPioSMs — the same
//  two sources of information that _checkPinConflict()/
//  _checkPioConflict() consult when checking the NEXT module to be
//  registered.
// ─────────────────────────────────────────────────────────────
void ModuleRegistry::_claimResources(ModuleBase* m) {
    if (!m->isHardwareModule()) return;
    HardwareModule* hw = static_cast<HardwareModule*>(m);

    const uint8_t* pins; uint8_t count;
    hw->usedPins(pins, count);
    for (uint8_t i = 0; i < count; i++)
        if (pins[i] < 64) _usedPins |= (1ULL << pins[i]);

    _usedPioSMs |= hw->usedPioSmMask();
}

// ─────────────────────────────────────────────────────────────
//  beginCore0() — start all core0 modules
//
//  Calls begin() on every registered module that runs on core0
//  or on both cores. Call once from setup() on core0.
//
//  Iterates every registered module in registration order, skipping
//  any that are currently disabled (enabled()==false). For each
//  remaining module, checks its declared core affinity: modules
//  declared as CORE0 or as BOTH (meaning they need explicit
//  initialisation on each core separately) have their begin() called
//  here; CORE1-only modules are left untouched (they are started
//  later, from beginCore1()).
// ─────────────────────────────────────────────────────────────
void ModuleRegistry::beginCore0() {
    for (uint8_t i = 0; i < _count; i++) {
        if (!_modules[i]->enabled()) continue;
        ModuleCore mc = _modules[i]->core();
        if (mc == ModuleCore::CORE0 || mc == ModuleCore::BOTH) {
            _modules[i]->begin();
        }
    }
}

// ─────────────────────────────────────────────────────────────
//  beginCore1() — start all core1 modules
//
//  Calls begin() on every registered module that runs on core1.
//  Logs the module name to Serial2 before and after each begin()
//  for startup diagnostics. Call once from setup1() on core1.
//
//  Iterates every registered module in registration order, skipping
//  disabled ones. For each module whose declared core affinity is
//  exactly CORE1 (note: unlike beginCore0(), modules declared as
//  BOTH are NOT started again here — their begin() already ran once
//  from beginCore0(), so BOTH-affinity modules must handle both
//  cores' initialisation within that single begin() call), prints the
//  module's name before and after calling its begin(), so that a
//  device that hangs or crashes during a specific module's
//  initialisation can be pinpointed from the Serial2 log (the last
//  "start X" line without a matching "klaar X" line identifies the
//  culprit).
// ─────────────────────────────────────────────────────────────
void ModuleRegistry::beginCore1() {
    for (uint8_t i = 0; i < _count; i++) {
        if (!_modules[i]->enabled()) continue;
        if (_modules[i]->core() == ModuleCore::CORE1) {
            traceSerial.print("beginCore1: start ");
            traceSerial.println(_modules[i]->name());
            _modules[i]->begin();
            traceSerial.print("beginCore1: klaar ");
            traceSerial.println(_modules[i]->name());
        }
    }
}

// ─────────────────────────────────────────────────────────────
//  loopCore0() — run one iteration of all core0 modules
//
//  First processes the CommandBus ring buffer (core1 → core0
//  commands), then calls loop() on each core0 and BOTH module.
//  Call repeatedly from loop() on core0.
//
//  CommandBus::instance().poll() is called first and unconditionally,
//  draining every command that protocol modules on core1 have
//  dispatched since the last call and delivering them to their
//  registered handlers (typically DccHal) — this happens before any
//  module's own loop() runs, so that a module's loop() logic always
//  sees the effects of commands issued earlier in this same cycle.
//  Afterwards, iterates every registered, enabled module and calls
//  loop() on those whose core affinity is CORE0 or BOTH.
// ─────────────────────────────────────────────────────────────
void ModuleRegistry::loopCore0() {
    // Process commands from core1 → core0 via CommandBus ring buffer
    CommandBus::instance().poll();

    for (uint8_t i = 0; i < _count; i++) {
        if (!_modules[i]->enabled()) continue;
        ModuleCore c = _modules[i]->core();
        if (c == ModuleCore::CORE0 || c == ModuleCore::BOTH)
            _modules[i]->loop();
    }
}

// ─────────────────────────────────────────────────────────────
//  loopCore1() — run one iteration of all core1 modules
//
//  Calls loop() on each core1 module, then drains the EventBus
//  ring buffer (core0 → core1 events). Also calls Ethernet.maintain()
//  if DHCP is active, throttled to once per 60 seconds to avoid
//  the 8-second blocking DHCP renewal call on every iteration.
//  Call repeatedly from loop1() on core1.
//
//  Note the ordering here differs from loopCore0(): every CORE1
//  module's own loop() runs FIRST, and only afterwards is
//  EventBus::instance().poll() called to deliver any core0-originated
//  events to this cycle's set of handlers — the opposite order from
//  loopCore0(), which drains its incoming bus before running module
//  loops. (Modules declared as BOTH are handled exclusively by
//  loopCore0() above and are intentionally not looped again here.)
//
//  Finally, if gEthernetUsesDhcp is set, checks whether at least
//  60000ms have passed since the last Ethernet.maintain() call
//  (tracked in a function-local static, sLastMaintain) and, if so,
//  calls it and updates the timestamp. Ethernet.maintain() is
//  entirely skipped when a static IP is configured, since it is
//  unnecessary in that case and would otherwise block for up to 8
//  seconds performing a DHCP lease renewal check.
// ─────────────────────────────────────────────────────────────
void ModuleRegistry::loopCore1() {
    for (uint8_t i = 0; i < _count; i++) {
        if (!_modules[i]->enabled()) continue;
        if (_modules[i]->core() == ModuleCore::CORE1) {
            _modules[i]->loop();
        }
    }
    EventBus::instance().poll();

    // Ethernet.maintain() only when DHCP is active.
    // With a static IP it is not needed and would block for 8 seconds.
    if (gEthernetUsesDhcp) {
        static uint32_t sLastMaintain = 0;
        uint32_t now = millis();
        if (now - sLastMaintain >= 60000) {
            sLastMaintain = now;
            Ethernet.maintain();
        }
    }
}

// ─────────────────────────────────────────────────────────────
//  find() — look up a registered module by ID
//
//  Returns a pointer to the module, or nullptr if not found.
//  Performs a simple linear scan over every registered module
//  (regardless of enabled/disabled state or core affinity), comparing
//  each one's id() against the requested id. Used both externally
//  (e.g. by other code looking up a specific module) and internally
//  by add() to detect duplicate-ID registration attempts.
// ─────────────────────────────────────────────────────────────
ModuleBase* ModuleRegistry::find(uint8_t id) const {
    for (uint8_t i = 0; i < _count; i++)
        if (_modules[i]->id() == id) return _modules[i];
    return nullptr;
}
