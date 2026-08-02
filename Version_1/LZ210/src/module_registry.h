#pragma once
#include "module_arch.h"

// ═══════════════════════════════════════════════════════════════
//  ModuleRegistry — central module registration and lifecycle manager
//
//  All modules register themselves here before startup.
//  The registry calls begin() and loop() on the correct core,
//  and detects GPIO pin and PIO state machine conflicts at
//  registration time.
//
//  Usage in main sketch:
//    void setup()  { registry().add(&gDccHal); ... registry().beginCore0(); }
//    void setup1() { registry().beginCore1(); }
//    void loop()   { registry().loopCore0(); }
//    void loop1()  { registry().loopCore1(); }
// ═══════════════════════════════════════════════════════════════

#define MODULE_REGISTRY_MAX  16   // Maximum number of registered modules

class ModuleRegistry {
public:
    // instance() — returns the single shared ModuleRegistry instance
    // (backed by the file-scope global _gRegistry defined in
    // module_registry.cpp). The whole system (both cores) uses this
    // one instance to register modules and drive their lifecycle.
    static ModuleRegistry& instance();

    // Registers a module. Call before beginCore0()/beginCore1().
    // Returns false if a GPIO pin or PIO state machine conflict is detected.
    bool add(ModuleBase* module);

    // ── Core lifecycle ────────────────────────────────────

    // Calls begin() on all core0 modules (call from setup() on core0)
    void beginCore0();

    // Calls begin() on all core1 modules (call from setup1() on core1)
    void beginCore1();

    // Calls loop() on all core0 modules and processes the CommandBus.
    // Also calls Ethernet.maintain() if DHCP is active (throttled to 60s).
    void loopCore0();

    // Calls loop() on all core1 modules and polls the EventBus.
    void loopCore1();

    // ── Diagnostics ───────────────────────────────────────

    // Returns the number of registered modules
    uint8_t     count()          const { return _count; }

    // Returns module at index i, or nullptr if out of range
    ModuleBase* at(uint8_t i)    const { return i < _count ? _modules[i] : nullptr; }

    // Returns module with given ID, or nullptr if not registered
    ModuleBase* find(uint8_t id) const;

    // Returns true if a module with the given ID is registered
    bool        isRegistered(uint8_t id) const { return find(id) != nullptr; }

private:
    public:
    ModuleRegistry() = default;

    ModuleBase* _modules[MODULE_REGISTRY_MAX];
    uint8_t     _count = 0;

    // Resource conflict tracking
    uint64_t  _usedPins    = 0;   // Bitmask of claimed GPIO pins (GP0-GP63)
    uint16_t  _usedPioSMs  = 0;   // Bitmask of claimed PIO state machines

    // _checkPinConflict() — queries the module for the list of GPIO
    // pins it needs (via HardwareModule::usedPins()) and checks each
    // one against the running _usedPins bitmask of pins already
    // claimed by previously-added modules. Returns true (no conflict)
    // immediately for modules that are not HardwareModules at all
    // (plain ModuleBase-derived modules have no GPIO pins to check).
    // Returns false as soon as any single requested pin is found to
    // already be claimed.
    bool _checkPinConflict(ModuleBase* m);

    // _checkPioConflict() — queries the module for its PIO state-
    // machine usage bitmask (via HardwareModule::usedPioSmMask()) and
    // checks it for any overlapping bits with the running
    // _usedPioSMs bitmask. Returns true (no conflict) immediately for
    // modules that are not HardwareModules. Returns false if the
    // bitwise AND of the module's requested mask and the already-
    // claimed mask is non-zero.
    bool _checkPioConflict(ModuleBase* m);

    // _claimResources() — called only after both conflict checks have
    // already passed for this module. Sets the corresponding bits in
    // _usedPins for every GPIO pin the module reported via
    // usedPins(), and ORs the module's usedPioSmMask() into
    // _usedPioSMs, so that any subsequently added module's conflict
    // checks correctly see these pins/state-machines as taken. Does
    // nothing for non-HardwareModules.
    void _claimResources(ModuleBase* m);
};

// Global shortcut — equivalent to ModuleRegistry::instance()
inline ModuleRegistry& registry() { return ModuleRegistry::instance(); }
