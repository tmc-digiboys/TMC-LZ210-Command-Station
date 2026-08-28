// ═══════════════════════════════════════════════════════════════
//  rs_bus_hal.h  —  RS-Bus feedback HAL
//
//  Manages the RS-Bus master via PIO using the RSbusMaster library.
//  Delivers feedback module data as FEEDBACK_BULK events on the
//  EventBus whenever a module's state changes.
//
//  Current-state queries (isSeen/getNibble/getSensorState) read
//  directly from rsBus.fbState[] in the RSbusMaster library itself
//  (which already tracks this: nibble values, receive counter and
//  parity/sampling errors per address) — we therefore keep NO own
//  copy of that information here anymore. Only for the broadcast
//  decision (has the client already seen this?) do we keep a small,
//  separate _lastSent cache; that is a different concept from "what
//  is the current value" (fbState) — see loop().
//
//  RS-Bus addressing — NOTE, two different numbering schemes:
//    RS-Bus hardware (RSbusMaster, this file):       1-128
//    XpressNet/LAN protocol (outward-facing):        0-127 (§2.15)
//    RS-Bus module 1 = protocol address 0, module 128 = protocol address 127.
//    Each module has 8 inputs = 2 nibbles of 4 bits
//
//  EventBus format (FEEDBACK_BULK), per §2.15 "Schaltinformation"
//  (which §2.4.5 "BC Rückmeldung" refers to for the ADR/DAT format):
//    fbBulkAddr[]: protocol address (0-127, i.e. RS-Bus hw address -1)
//    fbBulkData[]: ITNZ byte — I(0) TT(10=Rückmeldebaustein) N(nibble) ZZZZ(value)
//
//  XpressNet 0x42 query format (handleSwitchQuery, xpressnet_handler.cpp):
//    Variant B (feedback module query) fixed on 25 June — previously
//    used the turnout-address formula (module-1)/4, which is only
//    correct for turnouts. Now: protocol address (+1 → RS-Bus hw
//    address) plus the correct ITNZ response (§2.15). Confirmed
//    against real LH100 traffic.
// ═══════════════════════════════════════════════════════════════
#pragma once
#include "module_arch.h"
#include "event_bus.h"
#include <RSbusMaster.h>

// rsBus is the library-global RSbusMaster instance, defined in
// RSbusMaster.cpp. This extern declaration must be HERE (not only in
// rs_bus_hal.cpp) because the inline member functions in THIS class
// (isSeen/getNibble/getMessageCount/...) use it directly — otherwise
// rsBus would not yet be known at the point the compiler processes
// those inline functions, which is exactly the error we just saw.
extern RsMaster rsBus;

// Pin and PIO configuration (adjust to match hardware)
#include "hardware_config.h"

#define RS_BUS_TX_PIN   HW_RSBUS_TX_PIN
#define RS_BUS_RX_PIN   HW_RSBUS_RX_PIN
#define RS_BUS_PIO      pio1  // PIO block (pio0/pio1/pio2)

class RsBusHal : public HardwareModule {
public:
    RsBusHal() : HardwareModule("RsBusHal", ModuleId::RS_BUS_HAL,
                                 ModuleCore::CORE0) {}

    // Initialises the PIO RS-Bus master, configures TX/RX pins
    void begin() override;

    // Polls RS-Bus feedback every 10ms, publishes FEEDBACK_BULK
    // events for any modules whose state has changed
    void loop()  override;

    // Reacts to POWER_ON (starts the RS-Bus, resets the cache) and
    // POWER_OFF (stops the RS-Bus gracefully).
    void onEvent(const Event& ev) override;

    // ── Sensor state queries ──────────────────────────────────
    // Read directly from rsBus.fbState[] (the RSbusMaster library) —
    // no own copy of the current state is kept anymore.

    // Returns the cached (last known) state of a single sensor input.
    // module: 1-128, input: 0-7 (4 inputs per nibble)
    bool    getSensorState(uint8_t module, uint8_t input) const;

    // Resets only what we ourselves still keep track of (_type,
    // _lastSent for delta detection). rsBus.fbState[] itself is also
    // reset here (see the .cpp) so a factory reset does not leave
    // stale library data behind.
    void    clearAll();

    // Returns true if the module at address addr (1-128) has ever
    // sent feedback — derived from the library's own receive counter.
    bool    isSeen(uint8_t addr) const {
        if (addr < 1 || addr > 128) return false;
        return rsBus.fbState[addr].count > 0;
    }

    // Returns one nibble (0 or 1) as a 4-bit value — read directly
    // from the library's fbState. Used by LenzLAN for the initial
    // RS-Bus state dump on connect.
    uint8_t getNibble(uint8_t addr, uint8_t nibble) const {
        if (addr < 1 || addr > 128 || nibble > 1) return 0;
        return rsBus.fbState[addr].nibble[nibble];
    }

    // Returns the 2-bit decoder type from the last RS-Bus message
    // received from this address (bits 6-5 of RsEntry.data, see
    // RSbusMaster.cpp's ISR). Numeric meaning (00/01/10/11 → which
    // decoder kind) is NOT yet confirmed — only kept for inspection,
    // not yet used functionally elsewhere (e.g. not in the XpressNet
    // 0x42 TT response). Not part of the library's fbState, so we
    // still keep track of this ourselves.
    uint8_t getType(uint8_t addr) const {
        if (addr < 1 || addr > 128) return 0;
        return _type[addr];
    }

    // Diagnostics: number of received messages and parity/sampling
    // errors respectively for this address, read directly from the
    // library's fbState.
    uint32_t getMessageCount(uint8_t addr) const {
        if (addr < 1 || addr > 128) return 0;
        return rsBus.fbState[addr].count;
    }
    uint16_t getParityErrors(uint8_t addr) const {
        if (addr < 1 || addr > 128) return 0;
        return rsBus.fbState[addr].errorsParity;
    }
    uint16_t getSamplingErrors(uint8_t addr) const {
        if (addr < 1 || addr > 128) return 0;
        return rsBus.fbState[addr].errorsSampling;
    }

private:
    uint8_t  _type[129] = {};      // Last received decoder type per address (2 bits, unverified meaning) — not in fbState[], so kept ourselves
    uint8_t  _lastSent[129][2] = {};  // Last nibble value BROADCAST TO CLIENTS — for delta detection in loop(), a different concept than "current value" (that is fbState)

    // Throttle timer for 10ms poll interval
    uint32_t _lastPollMs = 0;
};

extern RsBusHal gRsBusHal;
