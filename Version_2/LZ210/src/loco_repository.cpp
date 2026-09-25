// ═══════════════════════════════════════════════════════════════
//  loco_repository.cpp  —  Locomotive data repository
//
//  Thread-safe storage for locomotive speed, direction and function
//  state. Shared between core0 (DCC HAL) and core1 (protocol modules).
//  Uses a RP2350 hardware spinlock for cross-core mutual exclusion.
// ═══════════════════════════════════════════════════════════════
#include "loco_repository.h"
#include <cstring>
#include <Arduino.h>

// ─────────────────────────────────────────────────────────────
//  begin() — initialise the repository and claim a hardware spinlock
//
//  Clears all loco slots and dirty flags. Claims an unused RP2350
//  hardware spinlock for thread-safe cross-core access.
//  Safe to call multiple times — returns immediately if already
//  initialised (spinlock already claimed).
// ─────────────────────────────────────────────────────────────
void LocoRepository::begin() {
    if (_spinlock != nullptr) return;  // already initialised
    memset(_locos, 0, sizeof(_locos));
    memset(_dirty, 0, sizeof(_dirty));
    memset(_dirtySpeed, 0, sizeof(_dirtySpeed));
    memset(_dirtyFn,    0, sizeof(_dirtyFn));
    _count = 0;
    _spinlock = spin_lock_init(spin_lock_claim_unused(true));
}

// ─────────────────────────────────────────────────────────────
//  registerExtension() — register a per-loco extension data block
//
//  Allows protocol modules (e.g. LocoNet) to attach private data
//  to each loco entry. factory() is called when a loco slot is
//  created; destructor() is called when the slot is released.
//  size is the size in bytes of the extension data block.
//
//  Note: only a single extension registration is supported at a
//  time (there is one _extFactory/_extDestructor/_extSize/
//  _extModuleId set of fields, not a list) — a second call simply
//  overwrites the first module's registration.
// ─────────────────────────────────────────────────────────────
void LocoRepository::registerExtension(uint8_t moduleId,
                                        ExtFactory factory,
                                        ExtDestructor destructor,
                                        size_t size) {
    _extModuleId   = moduleId;
    _extFactory    = factory;
    _extDestructor = destructor;
    _extSize       = size;
}

// ─────────────────────────────────────────────────────────────
//  _indexOf() — find the array index for a given DCC address
//
//  Returns the index (0..LOCO_REPO_SIZE-1) if found, or -1 if
//  no active slot exists for this address.
// ─────────────────────────────────────────────────────────────
int8_t LocoRepository::_indexOf(uint16_t address) const {
    for (uint8_t i = 0; i < _count; i++)
        if (_locos[i].active && _locos[i].address == address)
            return (int8_t)i;
    return -1;
}

// ─────────────────────────────────────────────────────────────
//  find() — look up a loco by DCC address (read-only)
//
//  Returns a pointer to the LocoBase entry, or nullptr if the
//  address is not currently in the repository.
// ─────────────────────────────────────────────────────────────
const LocoBase* LocoRepository::find(uint16_t address) const {
    int8_t idx = _indexOf(address);
    return idx >= 0 ? &_locos[idx] : nullptr;
}

// ─────────────────────────────────────────────────────────────
//  at() — access a loco by array index (read-only)
//
//  Returns nullptr if index is out of range.
//  Used by the web interface to iterate all repository entries.
// ─────────────────────────────────────────────────────────────
const LocoBase* LocoRepository::at(uint8_t index) const {
    if (index >= _count) return nullptr;
    return &_locos[index];
}

// ─────────────────────────────────────────────────────────────
//  findOrCreate() — look up or allocate a loco slot
//
//  Returns the existing entry if the address is already in the
//  repository. Otherwise allocates the first free slot, initialises
//  it with default values (step mode 128, direction reverse per DR5000
//  convention, no functions) and calls the extension factory if set.
//  Returns nullptr if the repository is full (LOCO_REPO_SIZE).
//
//  On allocation: the entire LocoBase struct is first zeroed (so
//  every field not explicitly set below — functions, momentary flags,
//  binary states, consist info, ln.* fields other than slot/stat —
//  starts out at 0/false), then the address, active flag, default
//  128-step mode, current timestamp and LN_NO_SLOT/LN_STAT_FREE
//  LocoNet placeholders are filled in. If the newly used slot index
//  extends beyond the previously known highest-allocated index
//  (_count), _count is advanced accordingly, so at()/nextDirty()/etc.
//  iteration bounds stay correct without needing a separate compaction
//  step.
// ─────────────────────────────────────────────────────────────
LocoBase* LocoRepository::findOrCreate(uint16_t address) {
    int8_t idx = _indexOf(address);
    if (idx >= 0) return &_locos[idx];

    for (uint8_t i = 0; i < LOCO_REPO_SIZE; i++) {
        if (!_locos[i].active) {
            memset(&_locos[i], 0, sizeof(LocoBase));
            _locos[i].address    = address;
            _locos[i].active     = true;
            _locos[i].stepMode   = 4;          // 128 speed steps
            _locos[i].lastUsedMs = millis();
            _locos[i].forward    = false;       // initially reverse (DR5000 convention)
            _locos[i].ln.slot    = LN_NO_SLOT;
            _locos[i].ln.stat    = LN_STAT_FREE;
            if (_extFactory)
                _locos[i].extension = _extFactory();
            if (i >= _count) _count = i + 1;
            return &_locos[i];
        }
    }
    return nullptr;  // repository full
}

// ─────────────────────────────────────────────────────────────
//  markDirty() — mark a loco as needing a full DCC packet refresh
//
//  Marks both speed and function dirty flags. Called when a loco's
//  state changes and the DCC HAL needs to resend its full state.
//  Also refreshes lastUsedMs, since marking a loco dirty implies
//  some form of activity on it.
// ─────────────────────────────────────────────────────────────
void LocoRepository::markDirty(uint16_t address) {
    int8_t idx = _indexOf(address);
    if (idx >= 0) {
        _dirty[idx] = true;
        _dirtySpeed[idx] = true;
        _dirtyFn[idx]    = true;
        _locos[idx].lastUsedMs = millis();
    }
}

// ─────────────────────────────────────────────────────────────
//  markDirtySpeed() — mark only the speed/direction as dirty
//
//  Used when only speed or direction changes, not functions.
// ─────────────────────────────────────────────────────────────
void LocoRepository::markDirtySpeed(uint16_t address) {
    int8_t idx = _indexOf(address);
    if (idx >= 0) {
        _dirty[idx]      = true;
        _dirtySpeed[idx] = true;
        _locos[idx].lastUsedMs = millis();
    }
}

// ─────────────────────────────────────────────────────────────
//  markDirtyFn() — mark only the function state as dirty
//
//  Used when only function state changes, not speed or direction.
// ─────────────────────────────────────────────────────────────
void LocoRepository::markDirtyFn(uint16_t address) {
    int8_t idx = _indexOf(address);
    if (idx >= 0) {
        _dirty[idx]   = true;
        _dirtyFn[idx] = true;
        _locos[idx].lastUsedMs = millis();
    }
}

// ─────────────────────────────────────────────────────────────
//  clearAll() — reset all loco slots to empty
//
//  Called by factory reset. Does not release extension memory (any
//  extension blocks previously allocated via _extFactory() are
//  effectively leaked here, since _extDestructor() is never called —
//  acceptable for a factory reset, where the device is expected to
//  reinitialise from a clean state anyway). Note that this also does
//  NOT reset _count back to 0, so at()/iteration will still walk
//  through the same number of (now-inactive) slots as before.
// ─────────────────────────────────────────────────────────────
void LocoRepository::clearAll() {
    for (uint16_t i = 0; i < LOCO_REPO_SIZE; i++) {
        _locos[i] = LocoBase{};
    }
}

// ─────────────────────────────────────────────────────────────
//  release() — free a loco slot and release extension memory
//
//  Calls the extension destructor if one is registered,
//  then clears the slot so it can be reused.
//  Does nothing if the address is not currently in the repository.
//  Also clears this slot's any-change dirty flag (but not its
//  speed/function-specific dirty flags, which are left as-is —
//  harmless, since active==false will cause nextDirty() to skip this
//  slot regardless of those flags' state).
// ─────────────────────────────────────────────────────────────
void LocoRepository::release(uint16_t address) {
    int8_t idx = _indexOf(address);
    if (idx < 0) return;
    if (_extDestructor && _locos[idx].extension)
        _extDestructor(_locos[idx].extension);
    memset(&_locos[idx], 0, sizeof(LocoBase));
    _dirty[idx] = false;
}

// ─────────────────────────────────────────────────────────────
//  nextDirty() — return the next loco slot that needs a DCC refresh
//
//  Iterates the repository and returns the first active slot with
//  a dirty flag set. Clears the dirty flags for that slot.
//  speedDirty and fnDirty are set to indicate which aspects changed.
//  Returns nullptr if no dirty slots remain.
//  Called by DccHal::_processDirtyLocos() on every loop iteration.
//
//  Note this is a one-at-a-time API: each call finds and clears (at
//  most) a single dirty slot, rather than returning every dirty slot
//  at once — the caller is expected to call this repeatedly in a loop
//  until it returns nullptr, to drain every currently dirty entry.
// ─────────────────────────────────────────────────────────────
LocoBase* LocoRepository::nextDirty(bool* speedDirty, bool* fnDirty) {
    for (uint8_t i = 0; i < _count; i++) {
        if (_dirty[i] && _locos[i].active) {
            _dirty[i] = false;
            if (speedDirty) { *speedDirty = _dirtySpeed[i]; _dirtySpeed[i] = false; }
            if (fnDirty)    { *fnDirty    = _dirtyFn[i];    _dirtyFn[i]    = false; }
            return &_locos[i];
        }
    }
    return nullptr;
}

// ─────────────────────────────────────────────────────────────
//  nextLnDirty() — return the next loco needing a LocoNet update
//
//  Returns the first active loco with ln.dirty set and clears the
//  flag. Called by the LocoNet module to sync slot state.
//  Returns nullptr if no LocoNet-dirty slots remain. Like
//  nextDirty(), intended to be called repeatedly in a loop until it
//  returns nullptr, to drain every currently LocoNet-dirty entry.
// ─────────────────────────────────────────────────────────────
LocoBase* LocoRepository::nextLnDirty() {
    for (uint8_t i = 0; i < _count; i++) {
        if (_locos[i].active && _locos[i].ln.dirty) {
            _locos[i].ln.dirty = false;
            return &_locos[i];
        }
    }
    return nullptr;
}

// ─────────────────────────────────────────────────────────────
//  findBySlot() — look up a loco by LocoNet slot number
//
//  Returns the loco assigned to the given LocoNet slot, or nullptr
//  if no loco is assigned to that slot or slot is LN_NO_SLOT.
// ─────────────────────────────────────────────────────────────
LocoBase* LocoRepository::findBySlot(uint8_t slot) {
    if (slot == LN_NO_SLOT) return nullptr;
    for (uint8_t i = 0; i < _count; i++)
        if (_locos[i].active && _locos[i].ln.slot == slot)
            return &_locos[i];
    return nullptr;
}

// ─────────────────────────────────────────────────────────────
//  lockRead() / lockWrite() — acquire the cross-core spinlock
//
//  Spins until the spinlock is available or timeoutUs microseconds
//  have elapsed. Returns false on timeout.
//  Always pair with unlock() to release the spinlock.
//
//  lockRead(): busy-waits (via tight_loop_contents(), a hint to the
//  compiler/CPU that this is a spin-wait loop) while is_spin_locked()
//  reports the lock as currently held by the other core, checking the
//  elapsed time against timeoutUs on each iteration and giving up
//  (returning false) if it is exceeded. Once the lock appears free,
//  actually claims it via spin_lock_blocking() (which also disables
//  interrupts for the critical section) and stores the resulting
//  saved interrupt state in _savedIrq for unlock() to restore later.
//
//  lockWrite(): simply calls lockRead() — this repository does not
//  distinguish between shared (read) and exclusive (write) locking;
//  every lock is fully exclusive regardless of which name is used to
//  acquire it.
// ─────────────────────────────────────────────────────────────
bool LocoRepository::lockRead(uint32_t timeoutUs) {
    uint32_t t0 = time_us_32();
    while (is_spin_locked(_spinlock)) {
        if (time_us_32() - t0 > timeoutUs) return false;
        tight_loop_contents();
    }
    _savedIrq = spin_lock_blocking(_spinlock);
    return true;
}

bool LocoRepository::lockWrite(uint32_t timeoutUs) {
    return lockRead(timeoutUs);
}

// ─────────────────────────────────────────────────────────────
//  unlock() — release the cross-core spinlock
//
//  Must be called after every successful lockRead()/lockWrite().
//  Restores the interrupt state that was saved at lock time
//  (_savedIrq) as part of releasing the spinlock.
// ─────────────────────────────────────────────────────────────
void LocoRepository::unlock() {
    spin_unlock(_spinlock, _savedIrq);
}

// ─────────────────────────────────────────────────────────────
//  getFunctions() — return the function bitmap for a loco address
//
//  Returns 0 if the address is not in the repository.
//  Bit0=F0, Bit1=F1, ... Bit28=F28.
// ─────────────────────────────────────────────────────────────
uint32_t LocoRepository::getFunctions(uint16_t address) const {
    const LocoBase* l = find(address);
    return l ? l->functions : 0;
}

// ─────────────────────────────────────────────────────────────
//  setFunctions() — update the function bitmap for a loco address
//
//  Marks the entry dirty so the DCC HAL resends function packets.
//  Does nothing if the address is not currently in the repository.
//  Note: this sets the generic _dirty flag (a full refresh) rather
//  than specifically _dirtyFn — functionally harmless (nextDirty()
//  callers typically act on whichever of speedDirty/fnDirty is true,
//  and a full refresh is always a safe superset), but means a caller
//  using this convenience method will always trigger both a speed and
//  a function resend together rather than a function-only one.
// ─────────────────────────────────────────────────────────────
void LocoRepository::setFunctions(uint16_t address, uint32_t fn) {
    int8_t idx = _indexOf(address);
    if (idx >= 0) {
        _locos[idx].functions = fn;
        _dirty[idx] = true;
    }
}

// ─────────────────────────────────────────────────────────────
//  setSpeed() — update speed and direction for a loco address
//
//  Marks the entry dirty so the DCC HAL resends the speed packet.
//  Does nothing if the address is not currently in the repository.
//  As with setFunctions(), this sets the generic _dirty flag rather
//  than specifically _dirtySpeed.
// ─────────────────────────────────────────────────────────────
void LocoRepository::setSpeed(uint16_t address, uint8_t speed, bool fwd) {
    int8_t idx = _indexOf(address);
    if (idx >= 0) {
        _locos[idx].speed   = speed;
        _locos[idx].forward = fwd;
        _dirty[idx] = true;
    }
}
