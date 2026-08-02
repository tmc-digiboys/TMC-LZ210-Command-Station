#include "eeprom_store.h"
#include "xpressnet_handler.h"   // gAccessories[], MAX_ACCESSORIES

EepromStore _gEepromStore;

EepromStore& EepromStore::instance() {
    return _gEepromStore;
}

#include <LittleFS.h>
#include <string.h>

// ─────────────────────────────────────────────────────────────
//  begin() — mount LittleFS and load persisted parameter values
//
//  Attempts to mount the LittleFS filesystem. If mounting fails, this
//  most commonly means no flash filesystem partition was configured
//  in the Arduino IDE (Tools → Flash Size → an option that reserves
//  space for a filesystem), so a format is attempted as a recovery
//  step. If even that fails, the store falls back to running without
//  any persistence at all: _fsOk is left false, dirty/lastChange
//  state is reset, and begin() returns early — every parameter simply
//  keeps whatever default value it was registered with, and any
//  later commit()/load() call will be a no-op (since they check
//  _fsOk first).
//
//  If mounting succeeds (either immediately or after a format),
//  _fsOk is set true and load() is called to restore any previously
//  persisted values over the registered defaults, before clearing the
//  dirty state so the freshly loaded configuration is not immediately
//  re-written to flash.
// ─────────────────────────────────────────────────────────────
void EepromStore::begin() {
    if (!LittleFS.begin()) {
        // LittleFS mount failed — try to format
        // Most common cause: no FS partition configured
        // in the Arduino IDE (Tools → Flash Size → choose an option with FS)
        Serial2.println("LittleFS: start mislukt, probeer format...");
        if (!LittleFS.format() || !LittleFS.begin()) {
            Serial2.println("LittleFS: format mislukt — "
                            "gebruik standaard waarden (geen persistentie)");
            Serial2.println("Fix: Arduino IDE → Tools → Flash Size → "
                            "kies optie met FS (bijv. 2MB Sketch + 1MB FS)");
            // Continue without persistence — default values will be used
            _fsOk = false;
            _lastChangeMs = 0;
            _dirty = false;
            return;
        }
        Serial2.println("LittleFS: format geslaagd");
    }
    _fsOk = true;
    load();
    _lastChangeMs = 0;
    _dirty = false;
}

// ─────────────────────────────────────────────────────────────
//  loop() — auto-commit any pending changes after a quiet period
//
//  Does nothing if nothing is currently dirty. Otherwise, once
//  EEPROM_COMMIT_DELAY_MS has elapsed since the last change without
//  any further changes arriving, writes everything to flash via
//  commit(). This batches up rapid successive changes (e.g. someone
//  adjusting several settings in the web interface) into a single
//  flash write instead of writing on every individual change.
// ─────────────────────────────────────────────────────────────
void EepromStore::loop() {
    if (!_dirty) return;
    if (millis() - _lastChangeMs > EEPROM_COMMIT_DELAY_MS)
        commit();
}

// ─────────────────────────────────────────────────────────────
//  Parameter registration
// ─────────────────────────────────────────────────────────────

// ─────────────────────────────────────────────────────────────

// _registerParam() — shared setup logic used by every register*()
// method. Refuses to register anything if the parameter table is
// already full (EEPROM_MAX_PARAMS reached), if the key is too long to
// fit in EEPROM_KEY_MAX_LEN, or if a parameter with this exact key is
// already registered (duplicate registration is treated as an error,
// not silently ignored or overwritten). On success, allocates the
// next free ParamDef slot, copies in the key, type, size and owning
// module ID, clears the dirty flag, clears enumOpts, and zeroes both
// the value and defaultValue unions (the caller is expected to fill
// in the actual default value immediately afterwards).
// ─────────────────────────────────────────────────────────────

bool EepromStore::_registerParam(const char* key, ParamType type,
                                  uint8_t size, uint8_t moduleId) {
    if (_count >= EEPROM_MAX_PARAMS)    return false;
    if (strlen(key) >= EEPROM_KEY_MAX_LEN) return false;
    if (_indexOf(key) >= 0)             return false;

    ParamDef& p = _params[_count++];
    strncpy(p.key, key, EEPROM_KEY_MAX_LEN - 1);
    p.type     = type;
    p.size     = size;
    p.moduleId = moduleId;
    p.dirty    = false;
    p.enumOpts = nullptr;
    memset(&p.value, 0, sizeof(p.value));
    memset(&p.defaultValue, 0, sizeof(p.defaultValue));
    return true;
}

// ─────────────────────────────────────────────────────────────
// registerEnum() — registers a new ENUM-typed (uint8_t-backed)
// parameter via _registerParam(), then stores the default value in
// both value.u8 and defaultValue.u8, and records the supplied
// enumOpts array so the web interface can render this parameter as a
// dropdown of labelled options instead of a raw number.
// ─────────────────────────────────────────────────────────────

bool EepromStore::registerEnum(const char* key, uint8_t moduleId,
                                uint8_t defaultVal,
                                const char* const* opts) {
    if (!_registerParam(key, ParamType::ENUM, 1, moduleId)) return false;
    ParamDef& p = _params[_count - 1];
    p.value.u8        = defaultVal;
    p.defaultValue.u8 = defaultVal;
    p.enumOpts        = opts;
    return true;
}

// ─────────────────────────────────────────────────────────────

// setEnum() — plain alias for setUint8(), since ENUM parameters are
// stored in exactly the same way (value.u8) as UINT8 parameters.
// ─────────────────────────────────────────────────────────────
bool EepromStore::setEnum(const char* key, uint8_t value) {
    return setUint8(key, value);
}

// ─────────────────────────────────────────────────────────────
// registerUint8() — registers a new UINT8 parameter via
// _registerParam() and initialises both its current and default value
// to `def`.
// ─────────────────────────────────────────────────────────────
bool EepromStore::registerUint8(const char* key, uint8_t def, uint8_t mid) {
    if (!_registerParam(key, ParamType::UINT8, 1, mid)) return false;
    _params[_count-1].value.u8 = def;
    _params[_count-1].defaultValue.u8 = def;
    return true;
}

// ─────────────────────────────────────────────────────────────
// registerUint16() — registers a new UINT16 parameter via
// _registerParam() and initialises both its current and default value
// to `def`.
// ─────────────────────────────────────────────────────────────
bool EepromStore::registerUint16(const char* key, uint16_t def, uint8_t mid) {
    if (!_registerParam(key, ParamType::UINT16, 2, mid)) return false;
    _params[_count-1].value.u16 = def;
    _params[_count-1].defaultValue.u16 = def;
    return true;
}

// ─────────────────────────────────────────────────────────────
// registerUint32() — registers a new UINT32 parameter via
// _registerParam() and initialises both its current and default value
// to `def`.
// ─────────────────────────────────────────────────────────────
bool EepromStore::registerUint32(const char* key, uint32_t def, uint8_t mid) {
    if (!_registerParam(key, ParamType::UINT32, 4, mid)) return false;
    _params[_count-1].value.u32 = def;
    _params[_count-1].defaultValue.u32 = def;
    return true;
}

// ─────────────────────────────────────────────────────────────
// registerBool() — registers a new BOOL parameter via _registerParam()
// and initialises both its current and default value to `def`.
// ─────────────────────────────────────────────────────────────
bool EepromStore::registerBool(const char* key, bool def, uint8_t mid) {
    if (!_registerParam(key, ParamType::BOOL, 1, mid)) return false;
    _params[_count-1].value.b = def;
    _params[_count-1].defaultValue.b = def;
    return true;
}

// ─────────────────────────────────────────────────────────────
// registerString() — registers a new STRING parameter via
// _registerParam(), clamping `size` to at most 20 bytes (the fixed
// capacity of the value union's byte buffer), and copies the default
// string into the value buffer, always leaving it null-terminated
// (even if the supplied default string was longer than `size - 1`
// characters, in which case it is truncated to fit).
// ─────────────────────────────────────────────────────────────
bool EepromStore::registerString(const char* key, const char* def,
                                  uint8_t size, uint8_t mid) {
    if (size > 20) size = 20;   // max 20 bytes (plenty for a MAC 17+null, an IP, etc.)
    if (!_registerParam(key, ParamType::STRING, size, mid)) return false;
    strncpy((char*)_params[_count-1].value.bytes, def, size - 1);
    _params[_count-1].value.bytes[size - 1] = '\0';  // guarantee null-termination
    return true;
}

// ─────────────────────────────────────────────────────────────
//  Reading
// ─────────────────────────────────────────────────────────────

// ─────────────────────────────────────────────────────────────
// _indexOf() — performs a linear scan of the registered parameter
// table for an entry whose key matches exactly (up to
// EEPROM_KEY_MAX_LEN characters), returning its index if found, or -1
// if no such key is registered.
// ─────────────────────────────────────────────────────────────
int8_t EepromStore::_indexOf(const char* key) const {
    for (uint8_t i = 0; i < _count; i++)
        if (strncmp(_params[i].key, key, EEPROM_KEY_MAX_LEN) == 0)
            return (int8_t)i;
    return -1;
}

// ─────────────────────────────────────────────────────────────
// getUint8() — looks up `key`; if found and its registered type is
// either UINT8 or ENUM (both of which store their value in value.u8),
// returns that stored value. Otherwise (key not found, or registered
// under a different type) returns `def`.
// ─────────────────────────────────────────────────────────────
uint8_t EepromStore::getUint8(const char* key, uint8_t def) const {
    int8_t i = _indexOf(key);
    if (i < 0) return def;
    // ENUM and UINT8 both store their value in value.u8
    if (_params[i].type == ParamType::UINT8 || _params[i].type == ParamType::ENUM)
        return _params[i].value.u8;
    return def;
}

// ─────────────────────────────────────────────────────────────
// getUint16() — looks up `key`; if found and registered as UINT16,
// returns its stored value, otherwise returns `def`.
// ─────────────────────────────────────────────────────────────
uint16_t EepromStore::getUint16(const char* key, uint16_t def) const {
    int8_t i = _indexOf(key);
    return (i >= 0 && _params[i].type == ParamType::UINT16)
           ? _params[i].value.u16 : def;
}

// ─────────────────────────────────────────────────────────────
// getUint32() — looks up `key`; if found and registered as UINT32,
// returns its stored value, otherwise returns `def`.
// ─────────────────────────────────────────────────────────────
uint32_t EepromStore::getUint32(const char* key, uint32_t def) const {
    int8_t i = _indexOf(key);
    return (i >= 0 && _params[i].type == ParamType::UINT32)
           ? _params[i].value.u32 : def;
}

// ─────────────────────────────────────────────────────────────
// getBool() — looks up `key`; if found and registered as BOOL,
// returns its stored value, otherwise returns `def`.
// ─────────────────────────────────────────────────────────────
bool EepromStore::getBool(const char* key, bool def) const {
    int8_t i = _indexOf(key);
    return (i >= 0 && _params[i].type == ParamType::BOOL)
           ? _params[i].value.b : def;
}

// ─────────────────────────────────────────────────────────────
// getString() — looks up `key`; if found and registered as STRING,
// returns a pointer to its internally stored, null-terminated string
// buffer, otherwise returns `def`.
// ─────────────────────────────────────────────────────────────
const char* EepromStore::getString(const char* key, const char* def) const {
    int8_t i = _indexOf(key);
    return (i >= 0 && _params[i].type == ParamType::STRING)
           ? (const char*)_params[i].value.bytes : def;
}

// ─────────────────────────────────────────────────────────────
//  Writing
// ─────────────────────────────────────────────────────────────

// ─────────────────────────────────────────────────────────────
// setUint8() — looks up `key`; if not registered, returns false
// without side effects. Otherwise stores `value`, marks this
// parameter and the whole store dirty, records the current time as
// the last-change timestamp (used by loop() to schedule an
// auto-commit), and returns true.
// ─────────────────────────────────────────────────────────────
bool EepromStore::setUint8(const char* key, uint8_t value) {
    int8_t i = _indexOf(key);
    if (i < 0) return false;
    _params[i].value.u8 = value;
    _params[i].dirty = true;
    _dirty = true;
    _lastChangeMs = millis();
    return true;
}

// ─────────────────────────────────────────────────────────────
// setUint16() — same behaviour as setUint8(), for a UINT16-typed
// parameter.
// ─────────────────────────────────────────────────────────────
bool EepromStore::setUint16(const char* key, uint16_t value) {
    int8_t i = _indexOf(key);
    if (i < 0) return false;
    _params[i].value.u16 = value;
    _params[i].dirty = true;
    _dirty = true;
    _lastChangeMs = millis();
    return true;
}

// ─────────────────────────────────────────────────────────────
// setUint32() — same behaviour as setUint8(), for a UINT32-typed
// parameter.
// ─────────────────────────────────────────────────────────────
bool EepromStore::setUint32(const char* key, uint32_t value) {
    int8_t i = _indexOf(key);
    if (i < 0) return false;
    _params[i].value.u32 = value;
    _params[i].dirty = true;
    _dirty = true;
    _lastChangeMs = millis();
    return true;
}

// ─────────────────────────────────────────────────────────────
// setBool() — same behaviour as setUint8(), for a BOOL-typed
// parameter.
// ─────────────────────────────────────────────────────────────
bool EepromStore::setBool(const char* key, bool value) {
    int8_t i = _indexOf(key);
    if (i < 0) return false;
    _params[i].value.b = value;
    _params[i].dirty = true;
    _dirty = true;
    _lastChangeMs = millis();
    return true;
}

// ─────────────────────────────────────────────────────────────
// setString() — looks up `key`; if not registered, returns false
// without side effects. Otherwise copies `value` into the parameter's
// internal byte buffer (truncated to its registered size, always
// left null-terminated), marks it and the whole store dirty, records
// the change timestamp, and returns true.
// ─────────────────────────────────────────────────────────────
bool EepromStore::setString(const char* key, const char* value) {
    int8_t i = _indexOf(key);
    if (i < 0) return false;
    strncpy((char*)_params[i].value.bytes, value, _params[i].size - 1);
    _params[i].value.bytes[_params[i].size - 1] = '\0';  // guarantee null-termination
    _params[i].dirty = true;
    _dirty = true;
    _lastChangeMs = millis();
    return true;
}

// ─────────────────────────────────────────────────────────────
//  Persistence — binary format
//
//  File format: [magic 4B][count 1B] + per param:
//    [key EEPROM_KEY_MAX_LEN B][type 1B][size 1B][value size B]
// ─────────────────────────────────────────────────────────────
static const uint32_t MAGIC = 0x584E4C53;  // 'XNLS'

// ─────────────────────────────────────────────────────────────
//  commit() — write every registered parameter to flash
//
//  Does nothing (returns false) if the filesystem is not mounted.
//  Otherwise (re)creates EEPROM_STORE_FILE and writes the fixed
//  4-byte magic number followed by the current parameter count, then
//  for every registered parameter writes its key (padded/truncated to
//  EEPROM_KEY_MAX_LEN bytes), its type byte, its size byte, and the
//  raw bytes of its value union — clearing that parameter's own dirty
//  flag as it is written. After all parameters have been written,
//  closes the file, clears the store-wide dirty flag, and returns
//  true. Note that this always rewrites the entire file (there is no
//  partial/incremental write), which is acceptable given the small
//  size of the parameter table.
// ─────────────────────────────────────────────────────────────
bool EepromStore::commit() {
    if (!_fsOk) return false;
    File f = LittleFS.open(EEPROM_STORE_FILE, "w");
    if (!f) return false;

    f.write((uint8_t*)&MAGIC, 4);
    f.write(&_count, 1);

    for (uint8_t i = 0; i < _count; i++) {
        ParamDef& p = _params[i];
        f.write((uint8_t*)p.key, EEPROM_KEY_MAX_LEN);
        f.write((uint8_t*)&p.type, 1);
        f.write(&p.size, 1);
        f.write((uint8_t*)&p.value, sizeof(p.value));
        p.dirty = false;
    }
    f.close();
    _dirty = false;
    return true;
}

// ─────────────────────────────────────────────────────────────
//  load() — read persisted parameter values from flash
//
//  Does nothing (returns false) if the filesystem is not mounted, or
//  if EEPROM_STORE_FILE does not exist yet (e.g. first boot). Reads
//  the 4-byte magic number and bails out (closing the file, returning
//  false) if it does not match the expected constant — this guards
//  against reading a corrupted or foreign file as if it were valid
//  parameter data.
//
//  Otherwise reads the stored parameter count, then for each stored
//  entry reads back its key, type, size, and raw value bytes into
//  local variables. Looks up whether a parameter with that exact key
//  is CURRENTLY registered (via _indexOf()); if so, copies the loaded
//  value bytes over that parameter's in-memory value and clears its
//  dirty flag (so the freshly loaded value is not immediately written
//  back out again). If no currently-registered parameter matches that
//  key, the stored entry is simply skipped — this is how parameters
//  that have since been removed from the firmware are silently
//  dropped rather than causing an error.
//
//  Returns true once every stored entry has been processed this way
//  (regardless of how many keys matched), as long as the file itself
//  was successfully opened and had a valid magic number.
// ─────────────────────────────────────────────────────────────
bool EepromStore::load() {
    if (!_fsOk) return false;
    File f = LittleFS.open(EEPROM_STORE_FILE, "r");
    if (!f) return false;

    uint32_t magic = 0;
    f.read((uint8_t*)&magic, 4);
    if (magic != MAGIC) { f.close(); return false; }

    uint8_t count = 0;
    f.read(&count, 1);

    for (uint8_t i = 0; i < count; i++) {
        char      key[EEPROM_KEY_MAX_LEN];
        ParamType type;
        uint8_t   size;
        uint8_t   value[sizeof(((ParamDef*)0)->value)];

        f.read((uint8_t*)key, EEPROM_KEY_MAX_LEN);
        f.read((uint8_t*)&type, 1);
        f.read(&size, 1);
        f.read(value, sizeof(value));

        // Find the existing registration and overwrite its value
        int8_t idx = _indexOf(key);
        if (idx >= 0) {
            memcpy(&_params[idx].value, value, sizeof(value));
            _params[idx].dirty = false;
        }
        // Unregistered keys are ignored (removed parameters)
    }
    f.close();
    return true;
}

// ─────────────────────────────────────────────────────────────
//  clearStorage() — delete the config file from flash
//
//  Mounts LittleFS if it is not already mounted (this method may be
//  called before begin(), for a clean-slate reset), removes
//  EEPROM_STORE_FILE if it exists, unmounts LittleFS again, and
//  returns whether the removal succeeded. Intended to be called
//  before begin() when the caller wants every parameter to start out
//  at its registered default value on the next boot, ignoring
//  whatever was previously persisted.
// ─────────────────────────────────────────────────────────────
bool EepromStore::clearStorage() {
    // Delete the config file — LittleFS does not need to be mounted
    if (!LittleFS.begin()) return false;
    bool ok = LittleFS.remove(EEPROM_STORE_FILE);
    LittleFS.end();
    return ok;
}

// ─────────────────────────────────────────────────────────────
//  resetToDefaults() — restore every parameter's default value
//
//  Overwrites every registered parameter's current value with its
//  stored defaultValue (copied byte-for-byte, since both are the same
//  union type), then marks the whole store dirty and records the
//  current time as the last-change timestamp, so the next loop() call
//  will (after the usual commit delay) persist these restored
//  defaults to flash.
// ─────────────────────────────────────────────────────────────
void EepromStore::resetToDefaults() {
    for (uint8_t i = 0; i < _count; i++)
        memcpy(&_params[i].value, &_params[i].defaultValue,
               sizeof(_params[i].value));
    _dirty = true;
    _lastChangeMs = millis();
}

// paramAt() — returns a pointer to the parameter at the given index
// (in registration order), or nullptr if the index is out of range.
// Used by the web interface to iterate over every registered
// parameter without needing to know their keys in advance.
const ParamDef* EepromStore::paramAt(uint8_t index) const {
    if (index >= _count) return nullptr;
    return &_params[index];
}

// findParam() — returns a pointer to the registered parameter with
// the given key, or nullptr if no such key is registered. Unlike the
// getUint8()/getBool()/etc. accessors, this gives direct access to
// the full ParamDef (type, size, moduleId, dirty flag, enum options),
// which the web interface needs in order to render each parameter
// appropriately.
const ParamDef* EepromStore::findParam(const char* key) const {
    int8_t i = _indexOf(key);
    return i >= 0 ? &_params[i] : nullptr;
}

// ─────────────────────────────────────────────────────────────
//  saveAccessories() — save gAccessories[] as a binary file
//
//  Format: [magic4][count2][state bytes...]
//  Each byte = (state & 0x03) | (known ? 0x04 : 0x00) | (mode << 3)
//  Only turnouts with known=true are meaningfully stored;
//  the rest is stored as 0 (unknown on reload).
//
//  Called: after every switching action (via handleTurnout).
//  Writes ~1KB to flash — fast enough for real-time use.
// ─────────────────────────────────────────────────────────────
bool EepromStore::saveAccessories() {
    if (!_fsOk) return false;

    File f = LittleFS.open(ACCESSORIES_FILE, "w");
    if (!f) return false;

    const uint32_t magic = 0x53574143;  // "CAWS"
    const uint16_t count = MAX_ACCESSORIES;
    f.write((const uint8_t*)&magic, 4);
    f.write((const uint8_t*)&count, 2);

    for (uint16_t i = 0; i < MAX_ACCESSORIES; i++) {
        uint8_t b = (gAccessories[i].state & 0x03)
                  | (gAccessories[i].known ? 0x04 : 0x00)
                  | ((gAccessories[i].mode  & 0x01) << 3);
        f.write(&b, 1);
    }

    f.close();
    Serial2.println("EepromStore: accessories opgeslagen");
    return true;
}

// ─────────────────────────────────────────────────────────────
//  loadAccessories() — load gAccessories[] from flash
//
//  Called in setup() after eepromStore().begin().
//  On an invalid magic number or a mismatched count: no data is
//  loaded, all turnouts remain unknown (known=false).
// ─────────────────────────────────────────────────────────────
bool EepromStore::loadAccessories() {
    if (!_fsOk) return false;

    File f = LittleFS.open(ACCESSORIES_FILE, "r");
    if (!f) {
        Serial2.println("EepromStore: geen accessories bestand");
        return false;
    }

    uint32_t magic = 0; uint16_t count = 0;
    f.read((uint8_t*)&magic, 4);
    f.read((uint8_t*)&count, 2);

    if (magic != 0x53574143 || count != MAX_ACCESSORIES) {
        f.close();
        Serial2.println("EepromStore: accessories formaat ongeldig");
        return false;
    }

    uint16_t loaded = 0;
    for (uint16_t i = 0; i < MAX_ACCESSORIES; i++) {
        uint8_t b = 0;
        f.read(&b, 1);
        gAccessories[i].state = b & 0x03;
        gAccessories[i].known = (b >> 2) & 0x01;
        gAccessories[i].mode  = (b >> 3) & 0x01;
        if (gAccessories[i].known) loaded++;
    }

    f.close();
    Serial2.printf("EepromStore: %u wissels geladen\n", loaded);
    return true;
}
