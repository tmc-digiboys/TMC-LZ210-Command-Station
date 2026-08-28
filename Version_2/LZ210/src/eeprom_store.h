#pragma once
#include "module_arch.h"

// ═══════════════════════════════════════════════════════════════
//  EepromStore — persistent key-value storage via LittleFS (RP2350 flash)
//
//  Provides EEPROM-like behaviour using flash storage via LittleFS.
//  Stores typed key-value pairs; the web interface reads and writes
//  via the same API.
//
//  Modules register their parameters with a key and default value
//  during begin(). The web interface can then display and modify them.
//
//  Usage:
//    EepromStore& store = EepromStore::instance();
//    store.registerUint16("net.port", 5550, ModuleId::LENZ_LAN);
//    uint16_t port = store.getUint16("net.port", 5550);
//    store.commit();  // flush to flash
//
//  Persistence:
//    Values are loaded from flash in begin().
//    commit() writes all dirty values back to flash.
//    Auto-commit: triggered after EEPROM_COMMIT_DELAY_MS if dirty.
// ═══════════════════════════════════════════════════════════════

#define EEPROM_STORE_FILE      "/config.bin"
#define ACCESSORIES_FILE       "/accessories.bin"  // persistent turnout states
#define EEPROM_MAX_PARAMS      64
#define EEPROM_KEY_MAX_LEN     24
#define EEPROM_COMMIT_DELAY_MS 5000   // write 5s after the last change

enum class ParamType : uint8_t {
    UINT8  = 1,
    UINT16 = 2,
    UINT32 = 3,
    BOOL   = 4,
    STRING = 5,
    BYTES  = 6,
    ENUM   = 7,   // uint8_t value with a dropdown, options in enumOpts
};

struct ParamDef {
    char      key[EEPROM_KEY_MAX_LEN];
    ParamType type;
    uint8_t   size;          // size in bytes for BYTES/STRING types
    uint8_t   moduleId;      // owning module
    bool      dirty;         // Modified but not yet written to flash
    const char* const* enumOpts;  // For ENUM: null-terminated array of option labels
    union {
        uint8_t  u8;
        uint16_t u16;
        uint32_t u32;
        bool     b;
        uint8_t  bytes[20];   // 20 bytes: plenty for a MAC (17+null), an IP, etc.
    } value;
    union {
        uint8_t  u8;
        uint16_t u16;
        uint32_t u32;
        bool     b;
    } defaultValue;
};

class EepromStore : public ModuleBase {
public:
    static EepromStore& instance();

    // ── ModuleBase interface ───────────────────────────────
    void begin() override;
    void loop()  override;    // auto-commit processing

    // ── Parameter registration ──────────────────────────────
    // Must be called before or immediately after begin()

    // registerUint8/16/32/Bool/String() — registers a new parameter
    // under the given key, with the given default value and owning
    // module ID. Fails (returns false) if the parameter table is
    // full, the key is too long, or the key is already registered.
    // The parameter's current value is initialised to the supplied
    // default; a later load() may overwrite it with a previously
    // persisted value for the same key. registerString() additionally
    // clamps `size` to 20 bytes (the fixed size of the value union's
    // byte buffer) and copies the default string in, always leaving
    // it null-terminated even if the default was longer than fits.
    bool registerUint8 (const char* key, uint8_t  defaultVal, uint8_t moduleId);
    bool registerUint16(const char* key, uint16_t defaultVal, uint8_t moduleId);
    bool registerUint32(const char* key, uint32_t defaultVal, uint8_t moduleId);
    bool registerBool  (const char* key, bool     defaultVal, uint8_t moduleId);
    bool registerString(const char* key, const char* defaultVal,
                        uint8_t size, uint8_t moduleId);

    // ── Reading ──────────────────────────────────────────────

    // getUint8/16/32/Bool/String() — looks up the given key and
    // returns its currently stored value, provided a parameter with
    // that exact key AND matching type is registered. Returns the
    // supplied default value instead if the key is not found, or if
    // it was registered under a different type (this type check
    // exists to catch accidental cross-type lookups rather than
    // silently returning garbage).
    uint8_t     getUint8 (const char* key, uint8_t  def = 0)     const;
    uint16_t    getUint16(const char* key, uint16_t def = 0)     const;
    uint32_t    getUint32(const char* key, uint32_t def = 0)     const;
    bool        getBool  (const char* key, bool     def = false) const;
    const char* getString(const char* key, const char* def = "") const;

    // ── Registration ────────────────────────────────────────

    // registerEnum() — registers a uint8_t-backed parameter that the
    // web interface should present as a dropdown, using the given
    // null-terminated array of option label strings (enumOpts) rather
    // than a free-form numeric input.
    bool registerEnum(const char* key, uint8_t moduleId,
                      uint8_t defaultVal,
                      const char* const* opts);  // ENUM dropdown

    // ── Writing ──────────────────────────────────────────────

    // setUint8/16/32/Bool/String() — updates the value of an already-
    // registered parameter, marks both that parameter and the whole
    // store as dirty, and records the current time as the last-change
    // timestamp (used by loop() to schedule an auto-commit). Returns
    // false without changing anything if the key is not registered.
    // setEnum() is a plain alias for setUint8(), since ENUM values are
    // stored the same way as UINT8 values.
    bool setUint8 (const char* key, uint8_t  value);
    bool setEnum  (const char* key, uint8_t  value);  // alias for setUint8
    bool setUint16(const char* key, uint16_t value);
    bool setUint32(const char* key, uint32_t value);
    bool setBool  (const char* key, bool     value);
    bool setString(const char* key, const char* value);

    // ── Persistence ───────────────────────────────────────

    bool commit();              // write directly to flash
    bool load();                // load from flash (called by begin())
    bool hasChanges() const { return _dirty; }

    // Persistent turnout states (separate file, max 1KB)
    bool saveAccessories();     // write gAccessories[] to flash
    bool loadAccessories();     // load gAccessories[] from flash on startup
    void resetToDefaults();     // restore all values to defaults
    bool clearStorage();        // erase the config file from flash
                                // call before begin() for a clean start

    // ── Iteration support for the web interface ────────────────────────────
    uint8_t          paramCount()              const { return _count; }
    const ParamDef*  paramAt(uint8_t index)    const;
    const ParamDef*  findParam(const char* key) const;

private:
public:
    EepromStore() : ModuleBase("EepromStore", ModuleId::EEPROM_STORE,
                                ModuleCore::BOTH) {}

    ParamDef _params[EEPROM_MAX_PARAMS];
    uint8_t  _count = 0;
    bool     _dirty = false;
    bool     _fsOk  = false;   // false = no LittleFS partition available
    uint32_t _lastChangeMs = 0;

    // _indexOf() — linear search of the parameter table for an exact
    // key match, returning its index, or -1 if no such key is
    // registered.
    int8_t _indexOf(const char* key) const;

    // _registerParam() — shared bookkeeping for all register*()
    // methods: validates that there is a free slot, the key fits
    // within EEPROM_KEY_MAX_LEN, and the key is not already
    // registered, then allocates a new ParamDef entry with the given
    // type/size/moduleId, a cleared dirty flag, no enum options, and
    // a zeroed value/defaultValue. The caller (e.g. registerUint8())
    // fills in the actual default value afterwards.
    bool   _registerParam(const char* key, ParamType type,
                          uint8_t size, uint8_t moduleId);
};

// Global shortcut
inline EepromStore& eepromStore() { return EepromStore::instance(); }
