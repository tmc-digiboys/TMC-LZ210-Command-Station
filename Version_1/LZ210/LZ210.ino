// ═══════════════════════════════════════════════════════════════
//  LZ210 v1.0 — Modular architecture, top-level sketch
//
//  This is the Arduino entry-point file. It does not contain any
//  protocol logic itself — that all lives in the individual modules
//  under src/ (DccHal, LenzLan, Z21Lan, XpressNetRs485, LocoNet,
//  RsBusHal, Webserver, ...). This file is only responsible for:
//    1. Bringing up the shared infrastructure (CommandBus, EventBus,
//       LocoRepository, EepromStore) in the right order.
//    2. Registering every module with the ModuleRegistry, which then
//       drives each module's begin()/loop() on the correct RP2350 core.
//    3. Bringing up Ethernet (with DHCP/static-IP fallback) and mDNS
//       on core1, after core0 has already finished its own setup.
//    4. The two top-level loop() functions (one per core) that keep
//       the CommandBus/EventBus flowing and feed the watchdog.
//
//  RP2350 dual-core split:
//    Core 0: DccHal, RsBusHal, EepromStore, OledDisplay, LocoNet
//            (all latency-sensitive DCC/LocoNet/RS-Bus hardware work)
//    Core 1: LenzLan, LenzUsb, Z21Lan, XpressNetRs485, Webserver
//            (all client-facing network/USB/RS-485 protocol work)
//
//  setup()/loop() run on core0 (the Arduino-Pico core's default);
//  setup1()/loop1() run on core1 and are started automatically by
//  the core once setup() on core0 returns.
// ═══════════════════════════════════════════════════════════════
//
//  !! FACTORY RESET: uncomment the line below, flash, reboot !!
//  !! Then immediately comment it out again and reflash       !!
//#define FACTORY_RESET

#include "src/module_arch.h"
#include "src/hardware_config.h"
#include "src/module_registry.h"
#include "src/loconet_module.h"
#include "src/command_bus.h"
#include "src/event_bus.h"
#include "src/loco_repository.h"
#include "src/eeprom_store.h"
#include "src/dcc_hal.h"
#include "src/led_status.h"
#include "src/rs_bus_hal.h"
#include "src/xpressnet_rs485.h"
#include "src/lenz_lan.h"
#include "src/lenz_usb.h"
#include "src/z21_lan.h"
#include "src/webserver.h"
#include "src/oled_display.h"
#include <Ethernet.h>
#include <EthernetBonjour.h>
#include "hardware/watchdog.h"

// Fallback MAC address and IP, used only if the corresponding
// EEPROM-stored values ("net.mac"/"net.ip") are missing or invalid —
// see _loadNetworkConfig() and setup1() below.
static byte      mac[]    = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
static IPAddress staticIp(192, 168, 54, 200);

// gCore0Ready — set true at the very end of setup() (core0), after
// every module has been registered and had begin() called on it, and
// after gCentrale's initial power state has been set. setup1() (core1)
// busy-waits on this flag before doing anything at all, so core1 never
// races ahead of core0's own initialisation (e.g. never tries to use
// EepromStore/LocoRepository/CommandBus before core0 has brought them up).
volatile bool gCore0Ready      = false;

// gEthernetUsesDhcp — set true in setup1() only if Ethernet.begin()
// with DHCP actually succeeded. Read by ModuleRegistry::loopCore1()
// to decide whether to periodically call Ethernet.maintain() at all
// (a static-IP configuration never needs to renew a DHCP lease, and
// calling maintain() anyway would just waste time blocking on the
// DHCP client for no reason).
bool          gEthernetUsesDhcp = false;

// gResetRequested — set true by XpressNetHandler::handleReset() when a
// client explicitly requests a command-station reset. loop() (core0)
// checks this flag on every iteration: once set, it stops feeding the
// watchdog and spins forever, so the watchdog timer (armed for 8s in
// setup()) fires and performs an actual hardware reset shortly after.
volatile bool gResetRequested   = false;

// gSerial2Ready — set true as the very first thing setup() (core0)
// does, immediately after Serial2.begin(). setup1() (core1) busy-waits
// on this before printing anything to Serial2 itself, since the two
// cores start executing concurrently and core1 must not touch the
// UART before core0 has initialised it.
volatile bool gSerial2Ready = false;

// ─────────────────────────────────────────────────────────────
//  setup() — core0
//
//  Runs once, before core1's setup1() is allowed to start any real
//  work (see gCore0Ready above). Performs, in strict order:
//
//  1. Bring up Serial2 (the debug/log UART) and print a boot banner.
//  2. Initialise the LED status controller.
//  3. Initialise the three core cross-cutting subsystems: CommandBus,
//     EventBus and LocoRepository — these must exist before any
//     module's begin() runs, since several modules register handlers
//     with them or immediately look up/create loco entries.
//  4. Register every EEPROM-backed parameter's key, type and default
//     value via _registerDefaultParams() (this does NOT yet load any
//     previously persisted values — it only declares what parameters
//     exist), then call EepromStore::begin(), which mounts the flash
//     filesystem and loads any previously saved values on top of
//     those defaults.
//  5. If FACTORY_RESET was uncommented for this build, wipe the
//     stored configuration file and reset every parameter back to its
//     just-registered default (including the fixed IP address above),
//     so the very next boot after this one starts from a known state.
//  6. Load the previously persisted turnout (accessory) states from
//     flash via EepromStore::loadAccessories().
//  7. Bring up the OLED display FIRST (before registering any other
//     module), so the splash screen and then a live "module: ok/fail"
//     status line can be shown for every module as it is registered —
//     this gives visible, on-device feedback if a specific module
//     fails to register (e.g. due to a GPIO/PIO conflict) without
//     needing a serial terminal attached.
//  8. Register every module with the ModuleRegistry via registry().add(),
//     showing an OLED status line after each one (or each small group).
//     Ethernet-dependent modules (LenzLan, LenzUsb, Z21Lan) are
//     conditionally compiled out entirely when NO_ETHERNET is defined
//     (useful for bench-testing DCC/LocoNet/RS-Bus without a W5500
//     board attached). add() returns false on a resource conflict or
//     a full registry table; every returned bool is logged to Serial2
//     in one summary line for diagnostics.
//  9. Call registry().beginCore0(), which calls begin() on every
//     registered module whose declared core affinity is CORE0 or BOTH
//     — including gOled again, which is safe because OledDisplay::begin()
//     is idempotent (guarded by its own _initialized flag).
//  10. Clear the OLED and show the initial (power-off) power screen.
//  11. Set the initial gCentrale flags: track power starts OFF and no
//      emergency stop is active, matching the physical reality that the
//      DCC output has not been enabled yet at this point.
//  12. Set gCore0Ready = true (with an explicit __dmb() data memory
//      barrier both before and after, to guarantee this write and the
//      gCentrale writes just before it are actually visible to core1
//      before core1's busy-wait loop in setup1() can observe the flag).
//  13. Arm the hardware watchdog with an 8-second timeout. From this
//      point on, loop() must call watchdog_update() regularly (see
//      below) or the RP2350 will hard-reset itself.
// ─────────────────────────────────────────────────────────────
void setup() {
    Serial2.begin(115200);
    gSerial2Ready = true;
    Serial2.println("LZ210 v1.0 boot");

    gLeds.begin();  // initialise the status LEDs

    // Bring up core infrastructure
    CommandBus::instance().begin();
    EventBus::instance().begin();
    LocoRepository::instance().begin();

    _registerDefaultParams();
    EepromStore::instance().begin();

    // ── Factory reset check ──────────────────────────────────
    // Uncomment #define FACTORY_RESET at the top of this file,
    // flash, reboot. Then immediately comment it out again.
    #ifdef FACTORY_RESET
    EepromStore::instance().clearStorage();   // delete the stored config file
    EepromStore::instance().resetToDefaults(); // reset _params back to their defaults
    Serial2.println("*** FACTORY RESET uitgevoerd — default IP 192.168.54.200 ***");
    #endif

    // Load persisted turnout states from flash
    EepromStore::instance().loadAccessories();

    // ── OLED first — show the splash screen before module registration ──
    gOled.begin();
    gOled.showSplash("v2.0");
    delay(2000);

    // ── Register modules, with status shown on the display ───────────────
    bool r0 = registry().add(&gOled);
    bool r1 = registry().add(&EepromStore::instance());
    gOled.showStatus("EepromStore", r1);

    bool r2  = registry().add(&gDccHal);
    bool r2b = registry().add(&gRsBusHal);
    gOled.showStatus("DccHal",   r2);
    gOled.showStatus("RsBusHal", r2b);

#ifndef NO_ETHERNET
    bool r3 = registry().add(&gLenzLan);
    bool r4 = registry().add(&gLenzUsb);
    bool r5 = registry().add(&gZ21Lan);
    gOled.showStatus("LenzLan", r3);
    gOled.showStatus("LenzUsb", r4);
    gOled.showStatus("Z21Lan",  r5);
#endif

    bool r6 = registry().add(&gXpressNetRs485);
    bool r7 = registry().add(&gWebserver);
    bool r8 = registry().add(&gLocoNet);
    gOled.showStatus("XpressNet", r6);
    gOled.showStatus("Webserver", r7);
    gOled.showStatus("LocoNet",   r8);

    Serial2.printf("add: oled=%d eep=%d dcc=%d rsbus=%d xn=%d web=%d ln=%d\n",
                   r0, r1, r2, r2b, r6, r7, r8);

    delay(1500);  // leave the status screen readable for a moment

    // ── Start core0 ──────────────────────────────────────────
    // gOled's begin() is called again here but is idempotent
    // thanks to its own _initialized guard
    registry().beginCore0();

    // Show the power status screen
    gOled.clear();
    gOled.showPower(false);

    Serial2.println("Core0 klaar");
    __dmb();

    gCentrale.trackPowerOff = true;   // start with track power off
    gCentrale.emergencyStop = false;
    gCore0Ready = true;
    __dmb();

    // Periodic watchdog — resets the command station if core0 hangs (8 second timeout)
    watchdog_enable(8000, true);
}

// ─────────────────────────────────────────────────────────────
//  loop() — core0
//
//  Runs repeatedly for as long as the device is powered and not mid-
//  reset. Each iteration:
//
//  1. Checks gResetRequested first, before anything else: if a reset
//     has been requested (see XpressNetHandler::handleReset()), this
//     branch logs it, flushes the log so the message is not lost, and
//     then spins forever in a tight loop WITHOUT feeding the watchdog
//     — the watchdog (armed in setup() with an 8-second timeout) will
//     therefore fire and perform an actual hardware reset shortly
//     after this point, which is the intended mechanism for a clean,
//     full restart triggered from a client command.
//  2. Otherwise, feeds the watchdog (watchdog_update()) so the RP2350
//     does not reset itself under normal operation.
//  3. Advances the LED status controller's own time-based blink/pulse
//     state machine.
//  4. Drains the CommandBus ring buffer, delivering any commands that
//     protocol modules on core1 have dispatched since the last call to
//     their registered core0 handlers (this happens here, additionally
//     to the poll() ModuleRegistry::loopCore0() itself performs, giving
//     commands a chance to be processed slightly earlier in the cycle;
//     ModuleRegistry::loopCore0()'s own poll() call afterwards will
//     then simply find the ring buffer already empty on most passes).
//  5. Calls registry().loopCore0(), which itself polls CommandBus again
//     and then calls loop() on every registered CORE0/BOTH module.
// ─────────────────────────────────────────────────────────────
void loop() {
    if (gResetRequested) {
        // Stop feeding the watchdog — it will trigger a reset after 8 seconds
        // (or sooner if watchdog_enable was called elsewhere with a shorter timeout)
        Serial2.println("RESET: wordt uitgevoerd...");
        Serial2.flush();
        while(true) tight_loop_contents();
    }
    watchdog_update();  // prevent a watchdog reset
    gLeds.loop();
    CommandBus::instance().poll();  // process core1 → core0 commands
    registry().loopCore0();
}

// ─────────────────────────────────────────────────────────────
//  setup1() — core1
//
//  Started automatically by the Arduino-Pico core once core0's setup()
//  has returned. Performs, in order:
//
//  1. Busy-waits on gSerial2Ready (core0 must have already initialised
//     Serial2 before core1 is allowed to print anything to it), then
//     busy-waits (with a short delay(10) between checks, rather than a
//     tight spin) on gCore0Ready, ensuring core0 has fully finished its
//     own setup() — including registering and initialising every
//     CORE0/BOTH module and setting the initial gCentrale power state —
//     before core1 does anything that might depend on that state.
//
//  2. Initialises the W5500 Ethernet controller on its chip-select pin.
//
//  3. Loads the persisted MAC address (falling back to the hardcoded
//     default `mac[]` if none is stored / the stored value is
//     malformed) via _loadNetworkConfig(), and reads the DHCP-enabled
//     flag plus the static IP/gateway/subnet values from EepromStore.
//
//  4. Brings up Ethernet:
//     - If DHCP is enabled: attempts Ethernet.begin() with an 8-second
//       overall timeout and a 4-second per-attempt responseTimeout. If
//       that fails (returns 0), the W5500 is explicitly re-initialised
//       (Ethernet.init()) before falling back to a static-IP
//       Ethernet.begin() call — re-initialising here works around the
//       W5500 sometimes being left in an inconsistent state after a
//       failed DHCP negotiation. On success, sets gEthernetUsesDhcp=true
//       so loopCore1() knows to periodically call Ethernet.maintain().
//     - If DHCP is disabled: goes straight to a static-IP
//       Ethernet.begin() call using the loaded IP/gateway/subnet
//       (gateway is deliberately passed as both the gateway AND DNS
//       server argument, since this project has no separate DNS
//       server configuration of its own).
//
//  5. If Ethernet ended up with a non-zero IP address, starts the
//     EthernetBonjour mDNS responder under the configured hostname
//     (falling back to "tmc-centrale" if none is set or it is empty),
//     and registers a "_z21._udp" style service record advertising
//     port 21105 (the Z21 LAN protocol's UDP port) so mDNS-aware
//     clients can discover this station automatically. If there is no
//     valid IP at all (Ethernet completely failed), mDNS is skipped
//     entirely rather than attempting to start it with a meaningless
//     address.
//
//  6. Waits an additional fixed 500ms — empirically needed to let the
//     W5500's SPI bus settle/be fully released before the PIO2-based
//     XpressNet RS-485 state machines are initialised in the modules
//     that follow, avoiding SPI/PIO resource contention at boot.
//
//  7. Calls registry().beginCore1(), which calls begin() on every
//     registered CORE1 module (LenzLan, LenzUsb, Z21Lan, XpressNetRs485,
//     Webserver, ...) — deliberately done only now, AFTER Ethernet is
//     already up, since the DHCP timeout above is now short enough
//     that starting the handheld-facing protocol modules any earlier
//     would not meaningfully speed up overall boot time, while doing
//     it in this order keeps the two phases (network bring-up, then
//     protocol module bring-up) cleanly separated for diagnostics.
// ─────────────────────────────────────────────────────────────
void setup1() {
    while (!gSerial2Ready) tight_loop_contents();
    Serial2.println("setup1 gestart");
    while (!gCore0Ready) delay(10);

    Ethernet.init(HW_ETH_CS_PIN);
    byte storedMac[6];
    _loadNetworkConfig(storedMac);

    bool useDhcp = eepromStore().getBool("net.dhcp", true);
    IPAddress ip, gateway, subnet;
    ip.fromString     (eepromStore().getString("net.ip",      "192.168.54.200"));
    gateway.fromString(eepromStore().getString("net.gateway", "192.168.54.1"));
    subnet.fromString (eepromStore().getString("net.subnet",  "255.255.255.0"));

    Serial2.print("Ethernet initialiseren... ");
    if (useDhcp) {
        Serial2.print("DHCP... ");
        if (Ethernet.begin(storedMac, 8000, 4000) == 0) {
            // DHCP failed — re-initialise the W5500 for a static IP
            Serial2.print("DHCP mislukt, statisch IP: ");
            Ethernet.init(HW_ETH_CS_PIN);
            delay(100);
            Ethernet.begin(storedMac, ip, gateway, gateway, subnet);
        } else {
            gEthernetUsesDhcp = true;  // DHCP succeeded
        }
    } else {
        Serial2.print("Statisch IP: ");
        Ethernet.begin(storedMac, ip, gateway, gateway, subnet);
    }
    Serial2.println(Ethernet.localIP());

    if (Ethernet.localIP() != IPAddress(0,0,0,0)) {
        const char* hostname = eepromStore().getString("net.hostname", "tmc-centrale");
        if (hostname == nullptr || strlen(hostname) == 0) hostname = "tmc-centrale";
        EthernetBonjour.begin(hostname);
        EthernetBonjour.addServiceRecord("TMC Centrale._z21", 21105, MDNSServiceUDP);
        Serial2.print("mDNS: "); Serial2.print(hostname); Serial2.println(".local");
    } else {
        Serial2.println("mDNS: geen geldig IP, overgeslagen");
    }

    // Wait until the W5500 SPI bus is fully released before initialising PIO2
    delay(500);
    // Start handhelds and XpressNet AFTER ethernet — the DHCP timeout is now short enough
    registry().beginCore1();
    Serial2.println("setup1: Core1 klaar");
}

// ─────────────────────────────────────────────────────────────
//  loop1() — core1
//
//  Runs repeatedly on core1. Each iteration:
//  1. Drains the EventBus ring buffer, delivering any events published
//     by core0 modules (power state, loco state, RS-Bus feedback, ...)
//     to every registered core1 handler — done here BEFORE the module
//     loops run (registry().loopCore1() itself also polls EventBus, but
//     afterwards; see module_registry.cpp for the full ordering
//     rationale), so freshly arrived events are available as early as
//     possible in this cycle for any module logic that might check
//     state derived from them.
//  2. Services the mDNS responder (EthernetBonjour.run()), which needs
//     to be called regularly to answer mDNS queries and keep the
//     service record alive.
//  3. Calls registry().loopCore1(), which calls loop() on every
//     registered CORE1 module, polls EventBus again to catch anything
//     published during those module loops, and periodically calls
//     Ethernet.maintain() when DHCP is in use.
// ─────────────────────────────────────────────────────────────
void loop1() {
    EventBus::instance().poll();  // process core0 → core1 events
    EthernetBonjour.run();
    registry().loopCore1();
}

// ─────────────────────────────────────────────────────────────
//  Parameter registration
//
//  _registerDefaultParams() — declares every EEPROM-backed
//  configuration parameter this firmware supports: its key, type,
//  default value, and which module "owns" it (used by the web
//  interface to group parameters by module). This function ONLY
//  registers the parameter definitions and their compiled-in
//  defaults — it does not yet apply any previously persisted values;
//  that happens in the final store.load() call at the end, which
//  reads the flash-stored config file (if any) and overwrites each
//  matching parameter's in-memory value with what was actually saved
//  last time, while leaving any parameter with no stored entry (e.g.
//  one newly added in this firmware version) at its just-registered
//  default.
//
//  Network parameters (net.mac, net.hostname, net.ip, net.gateway,
//  net.subnet, net.dhcp) are registered with moduleId 0, since they
//  are used directly by this sketch's own setup1() rather than by a
//  specific protocol module.
//
//  Several parameters use registerEnum() with a small, module-local
//  array of option label strings (kCdeModeOpts, kSwitchSchemaOpts,
//  kSwitchFormatOpts, kXnVersionOpts, kXnKennungOpts), letting the web
//  interface render them as a dropdown instead of a free-form numeric
//  field — see EepromStore::registerEnum()/Webserver::_printField().
//  Each array is declared `static` so it remains valid for the
//  lifetime of the program even though registerEnum() only stores a
//  pointer to it, not a copy.
// ─────────────────────────────────────────────────────────────
void _registerDefaultParams() {
    auto& store = eepromStore();
    store.registerString("net.mac",      "DE:AD:BE:EF:FE:ED", 18, 0);
    store.registerString("net.hostname", "tmc-centrale",       32, 0);
    store.registerString("net.ip",       "192.168.54.200",      16, 0);
    store.registerString("net.gateway",  "192.168.54.1",        16, 0);

    // System — CDE booster short-circuit response
    static const char* const kCdeModeOpts[] = {
        "Centrale uit (power-off + LocoNet GPOFF)",   // 0 = current default
        "Booster lokaal (centrale blijft aan)",        // 1 = passive
        nullptr
    };
    store.registerEnum("sys.cde_mode", 0, 0, kCdeModeOpts);
    store.registerUint16("sys.cde_short_us", 500, ModuleId::DCC_HAL); // short-circuit threshold on the E-line, was a fixed 500µs
    store.registerString("net.subnet",   "255.255.255.0",      16, 0);
    store.registerBool  ("net.dhcp",     true,                     0);
    store.registerUint16("lenz.port",    5550,  ModuleId::LENZ_LAN);
    store.registerUint8 ("lenz.maxconn", 8,     ModuleId::LENZ_LAN);
    store.registerUint16("z21.port",     21105, ModuleId::Z21_LAN);
    store.registerUint8 ("dcc.steps",       128,  ModuleId::DCC_HAL);
    store.registerBool  ("dcc.railcom",     false,ModuleId::DCC_HAL);
    store.registerUint16("dcc.shortma",     2000, ModuleId::DCC_HAL);
    store.registerUint8 ("dcc.progreadmode",3,    ModuleId::DCC_HAL); // 0=off,1=bit,2=byte,3=both
    store.registerUint8 ("dcc.progrepeat",  18,   ModuleId::DCC_HAL); // 7-64
    store.registerUint8 ("dcc.rstsrepeat",  25,   ModuleId::DCC_HAL); // 25-255
    store.registerUint8 ("dcc.rstcrepeat",  6,    ModuleId::DCC_HAL); // 6-64
    // Turnout addressing
    static const char* const kSwitchSchemaOpts[] = {
        "ROCO", "IB / Intellibox", "Lenz", nullptr
    };
    static const char* const kSwitchFormatOpts[] = {
        "Lineair", "Non-lineair", nullptr
    };
    store.registerEnum("dcc.swschema", ModuleId::DCC_HAL, 0, kSwitchSchemaOpts);
    store.registerEnum("dcc.swformat", ModuleId::DCC_HAL, 0, kSwitchFormatOpts);
    store.registerBool ("xn.sw_broadcast", true, ModuleId::LENZ_LAN); // 0x42 broadcast after a turnout command
    // XpressNet command station identification
    static const char* const kXnVersionOpts[] = {
        "v3.2 (LZV100)", "v3.6 (LZV100)", "v4.0 (TMC)", nullptr
    };
    static const char* const kXnKennungOpts[] = {
        "0x00 (LZ100)", "0x10 (Roco MultiMaus)", nullptr
    };
    store.registerEnum("xn.version", ModuleId::LENZ_LAN, 1, kXnVersionOpts); // default v3.6
    store.registerEnum("xn.kennung", ModuleId::LENZ_LAN, 0, kXnKennungOpts); // default LZ100
    store.registerBool  ("rs485.enable", true,  ModuleId::XPRESSNET_RS485);
    store.registerBool  ("rsbus.enable", true,  ModuleId::RS_BUS_HAL);
    store.registerBool  ("web.enable",   true,  ModuleId::WEBSERVER);
    store.registerUint16("web.port",     80,    ModuleId::WEBSERVER);
    store.load();
}

// ─────────────────────────────────────────────────────────────
//  Load network configuration
//
//  _loadNetworkConfig() — fills outMac with the MAC address to use
//  for the Ethernet controller. Starts from the hardcoded fallback
//  `mac[]` array (copied in unconditionally first), then attempts to
//  overwrite it with the value stored under the "net.mac" EEPROM key:
//  if that string is at least 17 characters long (the minimum length
//  of a valid "XX:XX:XX:XX:XX:XX" MAC string) and sscanf() succeeds in
//  parsing all six hex byte groups from it, those six parsed bytes
//  replace the fallback MAC. If the stored string is missing, too
//  short, or malformed, the hardcoded fallback MAC simply remains in
//  place — this function never leaves outMac uninitialised.
// ─────────────────────────────────────────────────────────────
void _loadNetworkConfig(byte* outMac) {
    memcpy(outMac, mac, 6);
    const char* macStr = eepromStore().getString("net.mac", "");
    if (strlen(macStr) >= 17) {
        unsigned int b[6];
        if (sscanf(macStr, "%x:%x:%x:%x:%x:%x",
                   &b[0],&b[1],&b[2],&b[3],&b[4],&b[5]) == 6)
            for (int i = 0; i < 6; i++) outMac[i] = (byte)b[i];
    }
}
