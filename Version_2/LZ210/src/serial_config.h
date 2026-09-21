#pragma once
#include "eeprom_store.h"
// ─────────────────────────────────────────────────────────────
//  serial_config.h — data bits / parity / stop bits -> SERIAL_xxx
//
//  Small shared helper so both the debug/trace UART (Serial2, set up
//  directly in LZ210.ino's setup()) and the USB CDC port (Serial, set
//  up in LenzUsb::begin()) can turn their EEPROM-configured framing
//  settings ("usb.databits"/"usb.parity"/"usb.stopbits" and their
//  "dbg."-prefixed equivalents) into the config value HardwareSerial::
//  begin(baud, config) expects, without duplicating this mapping in
//  both files.
//
//  Every legal combination this project's web interface offers (5-8
//  data bits, N/E/O parity, 1-2 stop bits — 24 combinations total) is
//  listed explicitly via nested switches, rather than computed from a
//  formula, since the exact bit-packing of the SERIAL_xxx constants is
//  an Arduino-core implementation detail this project shouldn't rely
//  on staying formula-shaped across core versions.
//
//  Note (Rob): data bits/parity/stop bits are applied here for the USB
//  CDC port too, since that's what was asked for, but most CDC-ACM
//  drivers only report/track these values rather than actually
//  applying them to real framing — unlike Serial2 (a genuine UART),
//  where they matter for real.
// ─────────────────────────────────────────────────────────────

// parity: 0=None, 1=Even, 2=Odd (matches the "usb.parity"/"dbg.parity"
// enum's option order in LZ210.ino's registerEnum() call)
inline uint16_t serialConfigFor(uint8_t dataBits, uint8_t parity, uint8_t stopBits) {
    switch (dataBits) {
    case 5:
        switch (parity) {
        case 1:  return stopBits == 2 ? SERIAL_5E2 : SERIAL_5E1;
        case 2:  return stopBits == 2 ? SERIAL_5O2 : SERIAL_5O1;
        default: return stopBits == 2 ? SERIAL_5N2 : SERIAL_5N1;
        }
    case 6:
        switch (parity) {
        case 1:  return stopBits == 2 ? SERIAL_6E2 : SERIAL_6E1;
        case 2:  return stopBits == 2 ? SERIAL_6O2 : SERIAL_6O1;
        default: return stopBits == 2 ? SERIAL_6N2 : SERIAL_6N1;
        }
    case 7:
        switch (parity) {
        case 1:  return stopBits == 2 ? SERIAL_7E2 : SERIAL_7E1;
        case 2:  return stopBits == 2 ? SERIAL_7O2 : SERIAL_7O1;
        default: return stopBits == 2 ? SERIAL_7N2 : SERIAL_7N1;
        }
    default:  // 8 — also the fallback for any unexpected value
        switch (parity) {
        case 1:  return stopBits == 2 ? SERIAL_8E2 : SERIAL_8E1;
        case 2:  return stopBits == 2 ? SERIAL_8O2 : SERIAL_8O1;
        default: return stopBits == 2 ? SERIAL_8N2 : SERIAL_8N1;
        }
    }
}

// Reads a port's three "<prefix>.databits"/"<prefix>.parity"/
// "<prefix>.stopbits" EEPROM keys and returns the matching config
// value. dataBits enum index 0-3 -> 5-8 data bits (matches
// registerEnum()'s option order); parity and stopBits are used
// directly (0/1/2 and 0/1, the latter meaning 1 or 2 stop bits).
inline uint16_t serialConfigFromEeprom(const char* keyPrefix) {
    char key[24];
    snprintf(key, sizeof(key), "%s.databits", keyPrefix);
    uint8_t dataBits = 5 + eepromStore().getUint8(key, 3);  // index 3 -> 8
    snprintf(key, sizeof(key), "%s.parity", keyPrefix);
    uint8_t parity   = eepromStore().getUint8(key, 0);
    snprintf(key, sizeof(key), "%s.stopbits", keyPrefix);
    uint8_t stopBits = eepromStore().getUint8(key, 0) == 1 ? 2 : 1;
    return serialConfigFor(dataBits, parity, stopBits);
}
