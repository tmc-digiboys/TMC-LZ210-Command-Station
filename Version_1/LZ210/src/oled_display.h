#pragma once
// ═══════════════════════════════════════════════════════════════
//  oled_display.h  —  SH1106 128×64 OLED display driver (core0)
//
//  Driver for the GME12864-81 (1.3 inch SH1106) OLED display
//  connected via I2C. Provides text and graphics primitives with
//  a deferred double-buffer write to avoid I2C blocking in loop().
//
//  Text interface: 21 columns × 6 rows (6×8 pixel font)
//  Graphics: pixel, line, rectangle and bitmap drawing
//
//  SH1106 specifics:
//    Internal buffer is 132×64 but display shows 128×64 with a
//    2-pixel horizontal offset (OLED_COL_OFFSET). The top 16 pixel
//    rows of the SH1106 show corruption and are skipped (OLED_Y_OFFSET).
//
//  Hardware:
//    GP6 = SDA, GP7 = SCL (I2C1, 400kHz)
//    I2C address 0x3C (SA0=0) or 0x3D (SA0=1)
// ═══════════════════════════════════════════════════════════════

#include "module_arch.h"
#include <hardware/i2c.h>
#include <cstdarg>

// ── Hardware configuration ────────────────────────────────────
#include "hardware_config.h"

#define OLED_SDA_PIN     HW_OLED_SDA_PIN
#define OLED_SCL_PIN     HW_OLED_SCL_PIN
#define OLED_I2C         i2c1
#define OLED_I2C_ADDR    0x3C    // Default SH1106 address (0x3D when SA0=1)
#define OLED_I2C_FREQ    400000  // 400kHz fast mode

// ── Display dimensions ────────────────────────────────────────
#define OLED_WIDTH       128
#define OLED_HEIGHT      64
#define OLED_PAGES       8      // 64 pixels / 8 bits per page
#define OLED_COL_OFFSET  2      // SH1106 internal horizontal offset

// ── Font dimensions (6×8 pixel character cell) ───────────────
#define FONT_W           6
#define FONT_H           8
#define FONT_COLS        (OLED_WIDTH  / FONT_W)  // 21 columns
#define OLED_Y_OFFSET    16   // Top 16 rows corrupt on SH1106 — skipped
#define FONT_ROWS        ((OLED_HEIGHT - OLED_Y_OFFSET) / FONT_H)  // 6 rows

class OledDisplay : public HardwareModule
{
public:
    OledDisplay()
        : HardwareModule("OledDisplay", ModuleId::OLED_DISPLAY, ModuleCore::CORE0)
    {}

    // begin() — initialises I2C1 at 400kHz on the SDA/SCL pins,
    // sends the SH1106 initialisation command sequence, clears the
    // framebuffer, and pushes the cleared framebuffer to the display.
    // Idempotent: a second call returns immediately without doing
    // anything (see the _initialized guard).
    void begin()  override;

    // loop() — currently a no-op. The framebuffer is written to the
    // display synchronously wherever show() is called (e.g. at the
    // end of showStatus()/showPower()/showSplash()) rather than via a
    // deferred/dirty-flag mechanism, despite the class-level comment
    // and the _dirty member suggesting that design — see oled_display.cpp
    // for the actual current behaviour.
    void loop()   override;

    // ── Text interface ────────────────────────────────────────

    // Clears the framebuffer and moves cursor to top-left (col=0, row=0)
    void clear();

    // Moves the text cursor to the given column (0-20) and row (0-5)
    void setCursor(uint8_t col, uint8_t row);

    // Writes a null-terminated string at the current cursor position
    void print(const char* str);

    // Writes a string followed by a newline (moves to next row, col=0)
    void println(const char* str);

    // Writes a formatted string (printf-style) at the current cursor
    void printf(const char* fmt, ...);

    // ── Graphics primitives ───────────────────────────────────

    // Sets or clears one pixel at (x, y) where x=0..127, y=0..63
    void drawPixel (uint8_t x, uint8_t y, bool on = true);

    // Draws a horizontal line at y from x to x+w-1
    void drawHLine (uint8_t x, uint8_t y, uint8_t w);

    // Draws a vertical line at x from y to y+h-1
    void drawVLine (uint8_t x, uint8_t y, uint8_t h);

    // Draws a rectangle outline, or filled rectangle if fill=true
    void drawRect  (uint8_t x, uint8_t y, uint8_t w, uint8_t h, bool fill = false);

    // Draws a 1-bit-per-pixel bitmap (row-major, MSB first)
    void drawBitmap(uint8_t x, uint8_t y, uint8_t w, uint8_t h, const uint8_t* data);

    // ── Application-level display functions ───────────────────

    // Shows one module startup status line (called during begin() sequence).
    // name: module name (max 14 characters); ok: true=✓ false=✗
    void showStatus(const char* name, bool ok);

    // Shows a large power on/off indicator in the centre of the screen
    void showPower(bool on);

    // Shows the startup splash screen with firmware version
    void showSplash(const char* version);

    // ── Display control ───────────────────────────────────────

    // Transfers the framebuffer to the display over I2C (blocking)
    void show();

    // Sets display brightness (0=dimmest, 255=brightest)
    void setContrast(uint8_t val);

    // Inverts all pixels on the display
    void setInvert(bool invert);

private:
    // Framebuffer: 128×64 pixels = 1024 bytes (1 byte per 8 vertical pixels)
    uint8_t _buf[OLED_WIDTH * OLED_PAGES];

    // Current text cursor position
    uint8_t _col = 0;
    uint8_t _row = 0;

    // Row counter for showStatus() — increments on each call
    uint8_t _statusRow = 0;

    // Initialisation guard — begin() is idempotent
    bool _initialized = false;

    // Dirty flag — show() is called from loop() when this is true
    volatile bool _dirty = false;

    // _cmd() — sends a single command byte to the SH1106 over I2C,
    // preceded by the SSD1306/SH1106 control byte 0x00 (which marks
    // the following byte as a command rather than display data).
    void _cmd(uint8_t c);

    // _writePage() — transfers one 128-byte horizontal page (8 rows
    // of pixels) of the framebuffer to the display. Sets the SH1106's
    // page address and the (offset-adjusted) column address via
    // command bytes, then writes the 128 framebuffer bytes for that
    // page as display data (preceded by control byte 0x40).
    void _writePage(uint8_t page);

    // _cmds() — sends a sequence of command bytes to the SH1106 by
    // calling _cmd() once per byte in `data`.
    void _cmds(const uint8_t* data, size_t len);

    // _initDisplay() — sends the fixed SH1106 initialisation command
    // sequence (INIT_SEQ, see oled_display.cpp) that configures clock
    // divider, multiplex ratio, display offset, charge pump, segment
    // remap, COM scan direction/pin configuration, contrast, pre-
    // charge and VCOMH timing, and finally turns the display on.
    void _initDisplay();

    // _drawChar() — draws one character glyph from the FONT6x8 table
    // into the framebuffer at the given text column/row, by drawing
    // its 6×8 pixels one column of the glyph at a time via drawPixel().
    void _drawChar(uint8_t col, uint8_t row, char c);
};

extern OledDisplay gOled;
