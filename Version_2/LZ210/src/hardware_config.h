 // ═══════════════════════════════════════════════════════════════
//  hardware_config.h  —  Centrale hardware-pinconfiguratie
//
//  Alle GPIO-pinnummers voor het TMC-LZ210 board op één plek.
//  Pas hier aan als je een ander board of andere pinning gebruikt.
//
//  Huidige hardware: RP2350 (Pico 2)
//
//  Overzicht:
//    GP0  GP1  — LocoNet (TX/RX)
//    GP2  GP3  — DCC signaal (+ geïnverteerd)
//    GP4        — DCC RailSync (LocoNet booster, via PIO)
//    GP5        — CDE booster E-signaal (kortsluitdetectie)
//    GP6  GP7  — OLED display (I2C SDA/SCL)
//    GP8  GP9  - TX/RX uart1 debug port
//    GP10 GP11 GP12 — XpressNet RS-485 (TX/RX/DE)
//    GP13       — Status LED
//    GP14 GP15 — RS-Bus (TX/RX, via PIO)
//    GP16 GP17 GP18 GP19 — W5500 Ethernet (SPI0: MISO/CS/SCK/MOSI)
//    GP20       — W5500 hardware reset
//    GP21       — LED Ethernet traffic
//    GP22       — LED DCC bus activity
//    GP27       — DCC ACK detectie
//    GP28       — LED RS-Bus activity
// ═══════════════════════════════════════════════════════════════
#pragma once

// ── LocoNet ───────────────────────────────────────────────────
#define HW_LN_TX_PIN        0   // GPIO00  — LocoNet TX (geïnverteerd voor open-collector)
#define HW_LN_RX_PIN        1   // GPIO01  — LocoNet RX
#define HW_LN_ENABLE			 15

// ── DCC uitgang ───────────────────────────────────────────────
#define HW_DCC_PIN          32   // GPIO32 — DCC signaal uitgang
#define HW_DCC2_PIN         33   // GPIO33 — DCC signaal geïnverteerd (H-bridge)
#define HW_DCC_RAILSYNC_PIN 34   // GPIO34 — DCC RailSync voor LocoNet boosters (PIO)
#define HW_DCC_FAULT			 35   // GPIO35 — Fault signal for DCC Signal
#define HW_DCC_ACTIVE		 36	// GPIO36 — Activate /sleep pi DCC H-Bridge
#define HW_SM_FAULT			 37   // GPIO37 — Fault signal for programming track DCC Signal fault
#define HW_SM_ACTIVE			 38   // GPIO38 — Active signal for programming track
#define HW_CDE_FAULT			 39   // GPIO39 — Fault signal for CDE Interface
#define HW_CDE_ACTIVE		 40   // GPIO40 — active Signal for CDE interface


//── DCC uitgang ───────────────────────────────────────────────
#define HW_DCC_SENSE	       44   // GPIO44 — DCC decoder ACK detectie 
#define HW_DCC_ACK_PIN	    44   // temporary needs to be removged
#define HW_SM_SENSE			 43   // GPIO43 — Loconet Sense detectie 
#define HW_CDE_SENSE			 42   // GPIO42 — CDE SENSE detectie 

// ── CDE booster kortsluitdetectie ────────────────────────────
#define HW_CDE_PIN          41  // GPIO41  — CDE E-signaal ingang (actief laag bij kortsluiting)

// ── OLED display (I2C) ────────────────────────────────────────
#define HW_OLED_SDA_PIN     2   // GPIO02  — I2C SDA
#define HW_OLED_SCL_PIN    3    // GPIO3 — I2C SCL

// -- Debug Uart ( Uart1)
#define HW_DBG_TX				8    // GPIO08 - TX Uart 1
#define HW_DBG_RX				9	  // GPIO09 - RX Uart 1
	
// ── XpressNet RS-485 ──────────────────────────────────────────
#define HW_XN_TX_PIN       12   // GPIO12 — RS-485 TX (PIO)
#define HW_XN_RX_PIN       13   // GPIO13 — RS-485 RX (PIO)
#define HW_XN_DE_PIN       14   // GPIO14 — RS-485 driver enable

// ── Status LED's ──────────────────────────────────────────────
// GP25 collides with LED_BUILTIN (the Olimex board's onboard LED,
// used for the status LED — power-on/short-circuit) and cannot be
// reassigned; this is a fixed hardware trait of the fabricated v2
// board, not something to work around per-pin. Resolved instead by
// reassigning the roles below: DCC bus activity moves onto the pin
// LN bus activity used to have, and LN bus activity is merged onto
// the same physical LED as XPN bus activity (HW_LED_LNBUS_PIN and
// HW_LED_XPNBUS_PIN are now intentionally the same GPIO) — so either
// LocoNet or XpressNet traffic lights that one shared LED. No change
// needed in led_status.h/.cpp: lnbus and xpnbus simply become two
// Led objects pointing at the same physical pin.
#define HW_LED_STATUS_PIN  00   // Bestaat niet meer — status LED uses LED_BUILTIN instead, see led_status.h
#define HW_LED_W5500_PIN   22   // GPIO22 — Ethernet traffic indicator
#define HW_LED_DCCBUS_PIN  23   // GPIO23 — DCC bus activity indicator (moved from GP25, which collides with LED_BUILTIN/status)
#define HW_LED_RSBUS_PIN   26   // GPIO26 — RS-Bus activity indicator
#define HW_LED_LNBUS_PIN   24	  // GPIO24 — LN-Bus activity indicator (merged onto the same LED as XPN-Bus, see above)
#define HW_LED_XPNBUS_PIN  24   // GPIO24 — XPN-Bus activity indicator (shared with LN-Bus, see above)

// ── RS-Bus (PIO) ──────────────────────────────────────────────
#define HW_RSBUS_TX_PIN    10   // GPIO10 — RS-Bus TX
#define HW_RSBUS_RX_PIN    11   // GPIO11 — RS-Bus RX
#define HW_RSBUS_SENSE     26   // GPIO26 - RS-BUS Sense pin

// ── W5500 Ethernet (SPI0) ────────────────────────────────────
#define HW_ETH_MISO_PIN    16   // GPIO16 — SPI0 MISO
#define HW_ETH_CS_PIN      17   // GPIO17 — W5500 chip select
#define HW_ETH_SCK_PIN     18   // GPIO18 — SPI0 SCK
#define HW_ETH_MOSI_PIN    19   // GPIO19 — SPI0 MOSI
#define HW_ETH_RST_PIN     20   // GPIO20 — W5500 hardware reset (Active Low)
#define HW_ETH_INT_PIN     21   // GPIO21 — W5500 Interrupt


// ── Power Sense ──────────────────────────────────────────────
#define HW_PWR_VIN_PIN	   45   // GPIO45 — VIN Power rail
#define HW_PWR_12V_PIN     46   // GPIO46 — 12V Power rail
#define HW_PWR_5V_PIN		47   // GPIO47 — 5V Power rail


// ── EXT Connector ──────────────────────────────────────────────
#define HW_EXT_I2C1_SDA     2   // I2C1 SDA pin of Extended Connector on cpu
#define HW_EXT_I2C1_SCL		 3   // I2C1 SDA pin of extended Connector on CPU

#define HW_EXT_GPIO_4	    4   // Free GPIO-Pin on extended Connector
#define HW_EXT_GPIO_5	    5   // Free GPIO-Pin on extended Connector
#define HW_EXT_GPIO_6		 6   // Free GPIO-Pin on Extended Connector
#define HW_EXT_GPIO_7       7   // FREE GPIO-Pin on extended Connector

// ── Extended Connector SPI1 ────────────────────────────────────
#define HW_EXT_MISO_PIN    28   // GPIO16 — SPI1 MISO
#define HW_EXT_CS_PIN      29   // GPIO17 — SPI1 CS
#define HW_EXT_SCK_PIN     30   // GPIO18 — SPI1 SCK
#define HW_EXT_MOSI_PIN    31   // GPIO19 — SPI1 MOSI
#define HW_EXT_RST_PIN     27   // GPIO20 — SPI1 RST hardware reset (Active Low)



