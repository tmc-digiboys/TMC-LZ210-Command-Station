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
#define HW_LN_TX_PIN        0   // GP0  — LocoNet TX (geïnverteerd voor open-collector)
#define HW_LN_RX_PIN        1   // GP1  — LocoNet RX

// ── DCC uitgang ───────────────────────────────────────────────
#define HW_DCC_PIN          2   // GP2  — DCC signaal uitgang
#define HW_DCC2_PIN         3   // GP3  — DCC signaal geïnverteerd (H-bridge)
#define HW_DCC_RAILSYNC_PIN 4   // GP4  — DCC RailSync voor LocoNet boosters (PIO)
#define HW_DCC_ACK_PIN     27   // GP27 — DCC decoder ACK detectie (momenteel uitgeschakeld)

// ── CDE booster kortsluitdetectie ────────────────────────────
#define HW_CDE_PIN          5   // GP5  — CDE E-signaal ingang (actief laag bij kortsluiting)

// ── OLED display (I2C) ────────────────────────────────────────
#define HW_OLED_SDA_PIN     6   // GP6  — I2C SDA
#define HW_OLED_SCL_PIN     7   // GP7  — I2C SCL

// -- Debug Uart ( Uart1)
#define HW_DBG_TX				8    // GP08 - TX Uart 1
#define HW_DBG_RX				9	  // GP09 - RX Uart 1
	
// ── XpressNet RS-485 ──────────────────────────────────────────
#define HW_XN_TX_PIN       10   // GP10 — RS-485 TX (PIO)
#define HW_XN_RX_PIN       11   // GP11 — RS-485 RX (PIO)
#define HW_XN_DE_PIN       12   // GP12 — RS-485 driver enable

// ── Status LED's ──────────────────────────────────────────────
#define HW_LED_STATUS_PIN  13   // GP13 — railspanning status (knippert bij kortsluiting)
#define HW_LED_W5500_PIN   21   // GP21 — Ethernet traffic indicator
#define HW_LED_DCCBUS_PIN  22   // GP22 — DCC bus activity indicator
#define HW_LED_RSBUS_PIN   28   // GP28 — RS-Bus activity indicator

// ── RS-Bus (PIO) ──────────────────────────────────────────────
#define HW_RSBUS_TX_PIN    14   // GP14 — RS-Bus TX
#define HW_RSBUS_RX_PIN    15   // GP15 — RS-Bus RX
#define HW_RSBUS_SENSE     26   // GP26 - RS-BUS Sense pin

// ── W5500 Ethernet (SPI0) ────────────────────────────────────
#define HW_ETH_MISO_PIN    16   // GP16 — SPI0 MISO
#define HW_ETH_CS_PIN      17   // GP17 — W5500 chip select
#define HW_ETH_SCK_PIN     18   // GP18 — SPI0 SCK
#define HW_ETH_MOSI_PIN    19   // GP19 — SPI0 MOSI
#define HW_ETH_RST_PIN     20   // GP20 — W5500 hardware reset (actief laag)

