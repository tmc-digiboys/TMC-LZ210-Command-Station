#pragma once
// ═══════════════════════════════════════════════════════════════
//  loco_repository.h  —  Thread-safe locomotive data store
//
//  Central storage for all active locomotive state: DCC address,
//  speed, direction, functions F0-F68, LocoNet slot assignment
//  and protocol extension data.
//
//  Thread safety:
//    Shared between core0 (DCC HAL, dirty-poll) and core1 (protocol
//    modules). Access is protected by a RP2350 hardware spinlock.
//    Always pair lockRead()/lockWrite() with unlock().
//
//  Dirty flags:
//    Three separate dirty flags per slot:
//    _dirty:      any change (superset of speed and fn)
//    _dirtySpeed: speed or direction changed
//    _dirtyFn:    function state changed
//    DccHal polls nextDirty() every loop to resend DCC packets.
//    SlotServer polls nextLnDirty() to resend LocoNet SL_RD_DATA.
//
//  Extension data:
//    Modules can attach private per-loco data via registerExtension().
//    The extension factory/destructor are called on slot creation/release.
// ═══════════════════════════════════════════════════════════════

#include "module_arch.h"
#include <pico/sync.h>

// Maximum number of simultaneous active locos
#define LOCO_REPO_SIZE  128

// ─────────────────────────────────────────────────────────────
//  LocoNet slot status constants
// ─────────────────────────────────────────────────────────────
constexpr uint8_t LN_STAT_FREE    = 0x00;  // Slot is unallocated
constexpr uint8_t LN_STAT_COMMON  = 0x01;  // Slot allocated, no throttle
constexpr uint8_t LN_STAT_IDLE    = 0x02;  // Slot idle (throttle released)
constexpr uint8_t LN_STAT_IN_USE  = 0x03;  // Slot in use by a throttle

// ─────────────────────────────────────────────────────────────
//  LocoNet speed step mode constants
//  Logical constants; bit translation happens in SlotServer::_fillSlotMsg
// ─────────────────────────────────────────────────────────────
constexpr uint8_t LN_SPD_14   = 0;  // 14 speed steps
constexpr uint8_t LN_SPD_28T  = 1;  // 28 steps Trinary
constexpr uint8_t LN_SPD_28   = 2;  // 28 speed steps
constexpr uint8_t LN_SPD_128  = 3;  // 128 speed steps (default)
constexpr uint8_t LN_SPD_28A  = 4;  // 28 steps Advanced
constexpr uint8_t LN_SPD_128A = 5;  // 128 steps Advanced

constexpr uint8_t  LN_NO_SLOT         = 0;        // Slot not assigned
constexpr uint32_t LN_SLOT_TIMEOUT_MS = 120000;   // 2 minutes idle → release

// ─────────────────────────────────────────────────────────────
//  LocoBase — one locomotive entry in the repository
// ─────────────────────────────────────────────────────────────
struct LocoBase {
    uint16_t address;    // DCC address (1-9999)
    bool     active;     // True if this slot is in use

    // ── DCC drive state ───────────────────────────────────
    uint8_t  speed;      // Current speed step (0=stop, 1=estop, 2..127=speed)
    bool     forward;    // Direction: true=forward, false=reverse
    uint8_t  stepMode;   // 0=DCC14, 2=DCC28, 4=DCC128

    // Function state F0-F31 (bit0=F0, bit1=F1, ... bit31=F31)
    uint32_t functions;

    // Extended functions F32-F68 (packed, 5 bytes × 8 bits = F32..F71)
    uint8_t  functionsExt[5];

    // Momentary (tastend) function flags F0-F31 (per NMRA DCC RP 9.2.1)
    uint32_t momentary;

    // Momentary (tastend) flags F32-F68 (packed, 5 bytes x 8 bits = F32..F71)
    // Parallel to functionsExt; bit layout identical (bit0 = F32, etc.)
    uint8_t  momentaryExt[5];

    // Binary States (DCC RP 9.2.1) — stub storage
    // States 1-32 stored as bitmask. States >32 are not stored.
    uint32_t binaryStates;

    // ── Consist and stack metadata ────────────────────────
    bool     busy;          // True if loco is in a consist
    bool     consist;       // True if consist is active
    uint8_t  consistAddr;   // Consist DCC address
    uint32_t lastUsedMs;    // Timestamp of last command (millis)

    // Per-loco extension data block (allocated by ExtFactory)
    void*    extension;

    // ── LocoNet slot assignment ───────────────────────────
    // Only SlotServer on core0 writes to ln.* fields.
    struct {
        uint8_t  slot;           // LocoNet slot number 1..120, or LN_NO_SLOT
        uint8_t  stat;           // Slot status: LN_STAT_FREE/COMMON/IDLE/IN_USE
        uint8_t  spdMode;        // Speed step mode: LN_SPD_*
        uint8_t  id1;            // Throttle ID low byte
        uint8_t  id2;            // Throttle ID high byte
        uint32_t lastActivityMs; // Timestamp of last throttle activity
        bool     dirty;          // True if SlotServer needs to send SL_RD_DATA
    } ln;
};

// ─────────────────────────────────────────────────────────────
//  Extension factory/destructor function pointer types
// ─────────────────────────────────────────────────────────────
typedef void* (*ExtFactory)();         // Returns a new extension data block
typedef void  (*ExtDestructor)(void*); // Frees an extension data block

// ─────────────────────────────────────────────────────────────
//  LocoRepository — thread-safe loco data store (singleton)
// ─────────────────────────────────────────────────────────────
class LocoRepository {
public:
    // instance() — returns the single shared LocoRepository instance,
    // constructed lazily as a function-local static on first call
    // (C++11 guarantees this initialisation is itself thread-safe).
    // Every module on either core uses this same instance.
    static LocoRepository& instance() {
        static LocoRepository inst;
        return inst;
    }

    // Initialises the repository and claims a hardware spinlock.
    // Safe to call multiple times — returns immediately if already done.
    void begin();

    // Registers per-loco extension data for a module.
    // factory():     allocates a new block when a loco slot is created
    // destructor():  frees the block when a slot is released
    // size:          size in bytes of the extension block
    void registerExtension(uint8_t moduleId,
                           ExtFactory factory,
                           ExtDestructor destructor,
                           size_t size);

    // ── Read access (call within lockRead/unlock) ─────────

    // Returns the loco entry for the given DCC address, or nullptr.
    const LocoBase* find(uint16_t address) const;

    // Returns the loco entry at array index, or nullptr if out of range.
    // Iterates only up to count() (not the full LOCO_REPO_SIZE array),
    // since indices beyond that are guaranteed never to have been
    // allocated.
    const LocoBase* at(uint8_t index) const;

    // Returns the number of allocated loco slots (including inactive)
    uint8_t count() const { return _count; }

    // ── Write access (call within lockWrite/unlock) ───────

    // Returns the existing slot for address, or allocates a new one.
    // Returns nullptr if the repository is full (LOCO_REPO_SIZE).
    LocoBase* findOrCreate(uint16_t address);

    // Marks both speed and function dirty — full DCC packet resend
    void markDirty(uint16_t address);

    // Marks only speed/direction dirty — resend speed packet only
    void markDirtySpeed(uint16_t address);

    // Marks only function state dirty — resend function packets only
    void markDirtyFn(uint16_t address);

    // Releases a loco slot (calls extension destructor if set)
    void release(uint16_t address);

    // Clears all loco slots (factory reset)
    void clearAll();

    // ── Dirty polling ─────────────────────────────────────

    // Returns the next dirty loco for DCC HAL resend, or nullptr.
    // Clears the dirty flags for the returned entry.
    // speedDirty/fnDirty indicate which aspects changed.
    LocoBase* nextDirty(bool* speedDirty = nullptr, bool* fnDirty = nullptr);

    // Returns the next loco needing a LocoNet SL_RD_DATA resend, or nullptr.
    LocoBase* nextLnDirty();

    // Returns the loco assigned to the given LocoNet slot, or nullptr.
    LocoBase* findBySlot(uint8_t slot);

    // ── Thread-safe cross-core locking ────────────────────

    // Acquires the spinlock. Spins up to timeoutUs microseconds.
    // Returns false on timeout. Always pair with unlock().
    bool lockRead (uint32_t timeoutUs = 500);
    // lockWrite() — identical implementation to lockRead(); kept as a
    // separate, distinctly named entry point purely for readability
    // at call sites (this repository does not distinguish between
    // shared/exclusive locking — every lock is fully exclusive).
    bool lockWrite(uint32_t timeoutUs = 500);  // Same as lockRead

    // Releases the spinlock acquired by lockRead/lockWrite
    void unlock();

    // ── Convenience helpers ───────────────────────────────

    // Returns the function bitmap for a loco address, or 0 if not found
    uint32_t getFunctions(uint16_t address) const;

    // Sets the function bitmap and marks the entry dirty
    void setFunctions(uint16_t address, uint32_t fn);

    // Sets speed and direction and marks the entry dirty
    void setSpeed(uint16_t address, uint8_t speed, bool fwd);

private:
    LocoRepository() = default;

    LocoBase _locos[LOCO_REPO_SIZE];     // Loco slot array
    bool     _dirty[LOCO_REPO_SIZE];     // Any-change dirty flag per slot
    bool     _dirtySpeed[LOCO_REPO_SIZE]; // Speed-change dirty flag per slot
    bool     _dirtyFn[LOCO_REPO_SIZE];   // Function-change dirty flag per slot
    uint8_t  _count = 0;                 // Highest allocated slot index + 1

    ExtFactory     _extFactory    = nullptr;  // Extension block allocator
    ExtDestructor  _extDestructor = nullptr;  // Extension block deallocator
    size_t         _extSize       = 0;
    uint8_t        _extModuleId   = 0;

    spin_lock_t* _spinlock = nullptr;    // RP2350 hardware spinlock
    uint32_t     _savedIrq = 0;          // Saved interrupt state (for unlock)

    // _indexOf() — linear scan of the first count() slots for one
    // that is active and matches `address` exactly. Returns array
    // index for a DCC address, or -1 if not found. The underlying
    // linear search is acceptable given LOCO_REPO_SIZE is small
    // (128) and lookups are infrequent relative to the main DCC/
    // LocoNet packet-generation loops.
    int8_t _indexOf(uint16_t address) const;
};

// Global shortcut — equivalent to LocoRepository::instance()
inline LocoRepository& locoRepo() { return LocoRepository::instance(); }
