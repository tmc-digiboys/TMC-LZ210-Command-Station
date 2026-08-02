// ═══════════════════════════════════════════════════════════════
//  oled_display.cpp  —  SH1106 OLED driver for the GME12864-81
// ═══════════════════════════════════════════════════════════════

#include "oled_display.h"
#include <cstring>
#include <cstdio>
#include <Arduino.h>

OledDisplay gOled;

// ── 6×8 ASCII font (0x20 space through 0x7E) ─────────────────
// Each column = 1 byte, 8 pixels tall
// bit 0 = TOP pixel, bit 7 = bottom pixel
static const uint8_t FONT6x8[][6] = {
    {0x00,0x00,0x00,0x00,0x00,0x00}, // 0x20 space
    {0x00,0x00,0x5F,0x00,0x00,0x00}, // !
    {0x00,0x07,0x00,0x07,0x00,0x00}, // "
    {0x14,0x7F,0x14,0x7F,0x14,0x00}, // #
    {0x24,0x2A,0x7F,0x2A,0x12,0x00}, // $
    {0x23,0x13,0x08,0x64,0x62,0x00}, // %
    {0x36,0x49,0x55,0x22,0x50,0x00}, // &
    {0x00,0x05,0x03,0x00,0x00,0x00}, // '
    {0x00,0x1C,0x22,0x41,0x00,0x00}, // (
    {0x00,0x41,0x22,0x1C,0x00,0x00}, // )
    {0x08,0x2A,0x1C,0x2A,0x08,0x00}, // *
    {0x08,0x08,0x3E,0x08,0x08,0x00}, // +
    {0x00,0x50,0x30,0x00,0x00,0x00}, // ,
    {0x08,0x08,0x08,0x08,0x08,0x00}, // -
    {0x00,0x60,0x60,0x00,0x00,0x00}, // .
    {0x20,0x10,0x08,0x04,0x02,0x00}, // /
    {0x3E,0x51,0x49,0x45,0x3E,0x00}, // 0
    {0x00,0x42,0x7F,0x40,0x00,0x00}, // 1
    {0x42,0x61,0x51,0x49,0x46,0x00}, // 2
    {0x21,0x41,0x45,0x4B,0x31,0x00}, // 3
    {0x18,0x14,0x12,0x7F,0x10,0x00}, // 4
    {0x27,0x45,0x45,0x45,0x39,0x00}, // 5
    {0x3C,0x4A,0x49,0x49,0x30,0x00}, // 6
    {0x01,0x71,0x09,0x05,0x03,0x00}, // 7
    {0x36,0x49,0x49,0x49,0x36,0x00}, // 8
    {0x06,0x49,0x49,0x29,0x1E,0x00}, // 9
    {0x00,0x36,0x36,0x00,0x00,0x00}, // :
    {0x00,0x56,0x36,0x00,0x00,0x00}, // ;
    {0x00,0x08,0x14,0x22,0x41,0x00}, // <
    {0x14,0x14,0x14,0x14,0x14,0x00}, // =
    {0x41,0x22,0x14,0x08,0x00,0x00}, // >
    {0x02,0x01,0x51,0x09,0x06,0x00}, // ?
    {0x32,0x49,0x79,0x41,0x3E,0x00}, // @
    {0x7E,0x11,0x11,0x11,0x7E,0x00}, // A
    {0x7F,0x49,0x49,0x49,0x36,0x00}, // B
    {0x3E,0x41,0x41,0x41,0x22,0x00}, // C
    {0x7F,0x41,0x41,0x22,0x1C,0x00}, // D
    {0x7F,0x49,0x49,0x49,0x41,0x00}, // E
    {0x7F,0x09,0x09,0x01,0x01,0x00}, // F
    {0x3E,0x41,0x41,0x51,0x32,0x00}, // G
    {0x7F,0x08,0x08,0x08,0x7F,0x00}, // H
    {0x00,0x41,0x7F,0x41,0x00,0x00}, // I
    {0x20,0x40,0x41,0x3F,0x01,0x00}, // J
    {0x7F,0x08,0x14,0x22,0x41,0x00}, // K
    {0x7F,0x40,0x40,0x40,0x40,0x00}, // L
    {0x7F,0x02,0x04,0x02,0x7F,0x00}, // M
    {0x7F,0x04,0x08,0x10,0x7F,0x00}, // N
    {0x3E,0x41,0x41,0x41,0x3E,0x00}, // O
    {0x7F,0x09,0x09,0x09,0x06,0x00}, // P
    {0x3E,0x41,0x51,0x21,0x5E,0x00}, // Q
    {0x7F,0x09,0x19,0x29,0x46,0x00}, // R
    {0x46,0x49,0x49,0x49,0x31,0x00}, // S
    {0x01,0x01,0x7F,0x01,0x01,0x00}, // T
    {0x3F,0x40,0x40,0x40,0x3F,0x00}, // U
    {0x1F,0x20,0x40,0x20,0x1F,0x00}, // V
    {0x3F,0x40,0x38,0x40,0x3F,0x00}, // W
    {0x63,0x14,0x08,0x14,0x63,0x00}, // X
    {0x03,0x04,0x78,0x04,0x03,0x00}, // Y
    {0x61,0x51,0x49,0x45,0x43,0x00}, // Z
    {0x00,0x00,0x7F,0x41,0x41,0x00}, // [
    {0x02,0x04,0x08,0x10,0x20,0x00}, // backslash
    {0x41,0x41,0x7F,0x00,0x00,0x00}, // ]
    {0x04,0x02,0x01,0x02,0x04,0x00}, // ^
    {0x40,0x40,0x40,0x40,0x40,0x00}, // _
    {0x00,0x01,0x02,0x04,0x00,0x00}, // `
    {0x20,0x54,0x54,0x54,0x78,0x00}, // a
    {0x7F,0x48,0x44,0x44,0x38,0x00}, // b
    {0x38,0x44,0x44,0x44,0x20,0x00}, // c
    {0x38,0x44,0x44,0x48,0x7F,0x00}, // d
    {0x38,0x54,0x54,0x54,0x18,0x00}, // e
    {0x08,0x7E,0x09,0x01,0x02,0x00}, // f
    {0x08,0x14,0x54,0x54,0x3C,0x00}, // g
    {0x7F,0x08,0x04,0x04,0x78,0x00}, // h
    {0x00,0x44,0x7D,0x40,0x00,0x00}, // i
    {0x20,0x40,0x44,0x3D,0x00,0x00}, // j
    {0x00,0x7F,0x10,0x28,0x44,0x00}, // k
    {0x00,0x41,0x7F,0x40,0x00,0x00}, // l
    {0x7C,0x04,0x18,0x04,0x78,0x00}, // m
    {0x7C,0x08,0x04,0x04,0x78,0x00}, // n
    {0x38,0x44,0x44,0x44,0x38,0x00}, // o
    {0x7C,0x14,0x14,0x14,0x08,0x00}, // p
    {0x08,0x14,0x14,0x18,0x7C,0x00}, // q
    {0x7C,0x08,0x04,0x04,0x08,0x00}, // r
    {0x48,0x54,0x54,0x54,0x20,0x00}, // s
    {0x04,0x3F,0x44,0x40,0x20,0x00}, // t
    {0x3C,0x40,0x40,0x20,0x7C,0x00}, // u
    {0x1C,0x20,0x40,0x20,0x1C,0x00}, // v
    {0x3C,0x40,0x30,0x40,0x3C,0x00}, // w
    {0x44,0x28,0x10,0x28,0x44,0x00}, // x
    {0x0C,0x50,0x50,0x50,0x3C,0x00}, // y
    {0x44,0x64,0x54,0x4C,0x44,0x00}, // z
    {0x00,0x08,0x36,0x41,0x00,0x00}, // {
    {0x00,0x00,0x7F,0x00,0x00,0x00}, // |
    {0x00,0x41,0x36,0x08,0x00,0x00}, // }
    {0x08,0x08,0x2A,0x1C,0x08,0x00}, // ~
};

// ── SH1106 initialisation sequence ─────────────────────────────
static const uint8_t INIT_SEQ[] = {
    0xAE,       // display off
    0xD5, 0x80, // clock divider/oscillator frequency
    0xA8, 0x3F, // multiplex ratio: 64 rows
    0xD3, 0x00, // display offset: 0
    0x40,       // display start line: 0
    0xAD, 0x8B, // charge pump on (SH1106)
    0xA1,       // segment remap
    0xC8,       // COM output scan direction reversed
    0xDA, 0x12, // COM pins hardware configuration
    0x81, 0xCF, // contrast control
    0xD9, 0xF1, // pre-charge period
    0xDB, 0x40, // VCOMH deselect level
    0xA4,       // normal display (not "entire display on")
    0xA6,       // not inverted
    0xAF,       // display on
};

// ── I2C helpers ───────────────────────────────────────────────

// _cmd() — sends a single command byte to the SH1106. The leading
// 0x00 control byte tells the display this transfer is a command,
// not display data (0x40 would mean data — see _writePage() below).
// Uses a 5ms I2C timeout so a stuck/disconnected display cannot
// permanently hang the caller.
void OledDisplay::_cmd(uint8_t c) {
    uint8_t buf[2] = { 0x00, c };
    i2c_write_timeout_us(OLED_I2C, OLED_I2C_ADDR, buf, 2, false, 5000);
}

// _cmds() — sends a sequence of command bytes by calling _cmd() once
// per byte in `data`. Used to send INIT_SEQ as a whole in one call.
void OledDisplay::_cmds(const uint8_t* data, size_t len) {
    for (size_t i = 0; i < len; i++) _cmd(data[i]);
}

// ── begin() ───────────────────────────────────────────────────

// begin() — one-time hardware initialisation. Guarded by
// _initialized so a repeated call is a harmless no-op. Initialises
// I2C1 at OLED_I2C_FREQ (400kHz), configures the SDA/SCL pins for
// I2C function with pull-ups enabled, waits 50ms for the display's
// own power-on reset to settle, then sends the SH1106 init sequence
// (_initDisplay()), clears the in-memory framebuffer (clear()), and
// pushes that cleared framebuffer out to the display (show()) so it
// starts in a known, blank state rather than showing whatever
// garbage was left in its GDDRAM at power-up.
void OledDisplay::begin() {
    if (_initialized) return;
    _initialized = true;

    i2c_init(OLED_I2C, OLED_I2C_FREQ);
    gpio_set_function(OLED_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(OLED_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(OLED_SDA_PIN);
    gpio_pull_up(OLED_SCL_PIN);
    sleep_ms(50);

    _initDisplay();
    clear();
    show();
}

// _initDisplay() — sends the fixed SH1106 initialisation command
// sequence (INIT_SEQ above) in one go via _cmds().
void OledDisplay::_initDisplay() {
    _cmds(INIT_SEQ, sizeof(INIT_SEQ));
}

// ── loop() ────────────────────────────────────────────────────

// loop() — intentionally empty. Despite the class-level header
// comment describing a "deferred double-buffer write to avoid I2C
// blocking in loop()" design (and the presence of the _dirty
// member), the actual display update currently happens synchronously
// and immediately, inside each application-level function
// (showStatus()/showPower()/showSplash(), each of which calls
// show() directly at the end) rather than being deferred to a
// dirty-flag check here.
void OledDisplay::loop() {}

// ── Framebuffer ───────────────────────────────────────────────

// clear() — zeroes the entire in-memory framebuffer (every pixel
// off) and resets the text cursor and showStatus() row counter back
// to their initial positions. Does NOT itself push anything to the
// physical display — call show() afterwards for that.
void OledDisplay::clear() {
    memset(_buf, 0, sizeof(_buf));
    _col = 0; _row = 0; _statusRow = 0;
}

// setCursor() — moves the text cursor to the given column/row,
// clamping both to the valid range (0..FONT_COLS-1, 0..FONT_ROWS-1)
// rather than wrapping or rejecting out-of-range values.
void OledDisplay::setCursor(uint8_t col, uint8_t row) {
    if (col >= FONT_COLS) col = FONT_COLS - 1;
    if (row >= FONT_ROWS) row = FONT_ROWS - 1;
    _col = col; _row = row;
}

// drawPixel() — sets or clears a single pixel at logical coordinate
// (x, y), where y is first shifted down by 16 (the fixed
// OLED_Y_OFFSET workaround for the SH1106's corrupted top 16 pixel
// rows on this specific display unit), before being ignored entirely
// if the resulting coordinate falls outside the physical 128×64
// panel. The framebuffer is organised as OLED_PAGES horizontal pages
// of 8 vertical pixels each (the SH1106's native GDDRAM layout): the
// byte index is x plus the page number (y/8) times the display
// width, and the specific bit within that byte is y%8. Setting the
// bit turns the pixel on; clearing it turns the pixel off.
void OledDisplay::drawPixel(uint8_t x, uint8_t y, bool on) {
    y += 16;  // hardware offset: the top 16 pixels are corrupted
    if (x >= OLED_WIDTH || y >= OLED_HEIGHT) return;
    uint16_t idx = (uint16_t)x + (uint16_t)(y / 8) * OLED_WIDTH;
    uint8_t  bit = y % 8;
    if (on) _buf[idx] |=  (1 << bit);
    else    _buf[idx] &= ~(1 << bit);
}

// drawHLine() — draws a horizontal line of length w starting at
// (x, y), by calling drawPixel() once per pixel, stopping early if
// it would run past the right edge of the display.
void OledDisplay::drawHLine(uint8_t x, uint8_t y, uint8_t w) {
    for (uint8_t i = 0; i < w && x + i < OLED_WIDTH; i++)
        drawPixel(x + i, y);
}

// drawVLine() — draws a vertical line of length h starting at
// (x, y), by calling drawPixel() once per pixel, stopping early if
// it would run past the bottom edge of the display.
void OledDisplay::drawVLine(uint8_t x, uint8_t y, uint8_t h) {
    for (uint8_t i = 0; i < h && y + i < OLED_HEIGHT; i++)
        drawPixel(x, y + i);
}

// drawRect() — draws a rectangle of size w×h with its top-left
// corner at (x, y). If fill is true, draws it as a series of filled
// horizontal lines (one per row, stopping early at the bottom edge
// of the display); otherwise draws only its four edges via two
// horizontal and two vertical lines.
void OledDisplay::drawRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, bool fill) {
    if (fill) {
        for (uint8_t row = y; row < y + h && row < OLED_HEIGHT; row++)
            drawHLine(x, row, w);
    } else {
        drawHLine(x,     y,     w);
        drawHLine(x,     y+h-1, w);
        drawVLine(x,     y,     h);
        drawVLine(x+w-1, y,     h);
    }
}

// drawBitmap() — draws a 1-bit-per-pixel bitmap of size w×h at
// (x, y). The source data is organised row-major in bytes of 8
// vertically-stacked pixels each (same page-based layout as the
// framebuffer itself), MSB... actually bit0=top-of-byte-row here
// (matching drawPixel()'s own bit%8 convention): for each pixel,
// looks up the byte covering that pixel's row-group and column
// (`(row/8)*w + col`), extracts the bit for this specific row within
// that group (`(byte >> (row%8)) & 1`), and calls drawPixel() with
// the result.
void OledDisplay::drawBitmap(uint8_t x, uint8_t y, uint8_t w, uint8_t h,
                              const uint8_t* data) {
    for (uint8_t row = 0; row < h; row++) {
        for (uint8_t col = 0; col < w; col++) {
            uint8_t byte = data[(row / 8) * w + col];
            drawPixel(x + col, y + row, (byte >> (row % 8)) & 1);
        }
    }
}

// ── Text ──────────────────────────────────────────────────────

// _drawChar() — draws a single character glyph from the FONT6x8
// table at the given text column/row. Non-printable characters
// (outside the 0x20-0x7E range covered by the font table) are
// substituted with a space. Looks up the glyph's 6 column bytes,
// converts the (col, row) text-cell position into pixel coordinates
// (px, py), and for every column/row combination within the 6×8 cell,
// draws the corresponding pixel via drawPixel() based on the
// relevant bit of that glyph column's byte.
void OledDisplay::_drawChar(uint8_t col, uint8_t row, char c) {
    if (c < 0x20 || c > 0x7E) c = ' ';
    const uint8_t* glyph = FONT6x8[c - 0x20];
    uint8_t px = col * FONT_W;
    uint8_t py = row * FONT_H;
    for (uint8_t fx = 0; fx < FONT_W; fx++) {
        uint8_t col_data = glyph[fx];
        for (uint8_t fy = 0; fy < FONT_H; fy++) {
            drawPixel(px + fx, py + fy, (col_data >> fy) & 1);
        }
    }
}

// print() — writes a null-terminated string starting at the current
// cursor position. A '\n' character moves to the start of the next
// row without drawing anything (clamped at the last row rather than
// scrolling or wrapping back to row 0). Any other character is drawn
// via _drawChar() at the current cursor cell; if the cursor is
// already past the last column when about to draw a character, it
// first wraps to the start of the next row (again clamped at the
// last row) before drawing. The cursor column advances by one after
// each drawn character.
void OledDisplay::print(const char* str) {
    while (*str) {
        if (*str == '\n') {
            _col = 0; if (++_row >= FONT_ROWS) _row = FONT_ROWS - 1;
        } else {
            if (_col >= FONT_COLS) { _col = 0; if (++_row >= FONT_ROWS) _row = FONT_ROWS - 1; }
            _drawChar(_col++, _row, *str);
        }
        str++;
    }
}

// println() — writes the given string via print(), then explicitly
// moves the cursor to the start of the next row (equivalent to
// print() encountering a trailing '\n', but without requiring the
// caller to add one).
void OledDisplay::println(const char* str) {
    print(str);
    _col = 0; if (++_row >= FONT_ROWS) _row = FONT_ROWS - 1;
}

// printf() — formats `fmt`/varargs into a fixed 128-byte stack
// buffer using vsnprintf() (safely truncating if the formatted
// result would not fit), then writes the result via print().
void OledDisplay::printf(const char* fmt, ...) {
    char buf[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    print(buf);
}

// ── show() — transfer the framebuffer to the display ───────────

// _writePage() — transfers one 128-byte horizontal page (8 rows of
// pixels) of the in-memory framebuffer to the physical display.
// Selects the target page via the SH1106 page-address command
// (0xB0 | page), then sets the column start address split across its
// low- and high-nibble command bytes, offset by OLED_COL_OFFSET (the
// SH1106's internal 132-column buffer vs. the physical 128-column
// panel). Builds a single I2C transfer consisting of the 0x40 "this
// is display data" control byte followed by the 128 framebuffer
// bytes for this page, using a longer (10ms) timeout than _cmd()
// since this transfer is considerably larger.
void OledDisplay::_writePage(uint8_t page) {
    _cmd(0xB0 | page);
    _cmd(0x00 | (OLED_COL_OFFSET & 0x0F));
    _cmd(0x10 | ((OLED_COL_OFFSET >> 4) & 0x0F));
    uint8_t txbuf[OLED_WIDTH + 1];
    txbuf[0] = 0x40;
    memcpy(txbuf + 1, _buf + page * OLED_WIDTH, OLED_WIDTH);
    i2c_write_timeout_us(OLED_I2C, OLED_I2C_ADDR,
                         txbuf, OLED_WIDTH + 1, false, 10000);
}

// show() — writes every page of the framebuffer out to the physical
// display via _writePage(). Page 0 is deliberately written TWICE in
// a row before moving on to the remaining pages — a known quirk of
// this particular SH1106 display unit where the first page write
// after selecting it is sometimes not correctly latched; writing it
// twice reliably works around that.
void OledDisplay::show() {
    // Page 0 written twice — SH1106 quirk
    _writePage(0);
    _writePage(0);
    for (uint8_t page = 1; page < OLED_PAGES; page++) {
        _writePage(page);
    }
}

// ── Application-level screens ───────────────────────────────────

// showSplash() — draws the startup splash screen: clears the
// framebuffer, draws a border rectangle around the whole display,
// prints the fixed title text, a horizontal separator line, the
// supplied firmware version string, and a fixed subtitle line, then
// pushes the result to the display.
void OledDisplay::showSplash(const char* version) {
    clear();
    drawRect(0, 0, OLED_WIDTH, OLED_HEIGHT, false);
    setCursor(2, 1); print("TMC XpressNetLAN");
    drawHLine(2, 17, 124);
    setCursor(7, 3); print(version);
    setCursor(2, 6); print("Core 0+1  RP2350");
    show();
}

// showStatus() — appends one line to the module-startup status
// screen, showing `name` (truncated/padded to 14 characters) followed
// by "ok" or "!!" depending on the `ok` flag. On the very first call
// (detected via _statusRow==0), clears the screen and draws a fixed
// two-column header ("Module" / "St") plus a separator line first,
// before starting to print actual status lines from row 1 onward. If
// the screen is already full (_statusRow has reached FONT_ROWS),
// silently does nothing rather than overflowing or scrolling.
// Otherwise formats the line into a fixed-width buffer, prints it,
// advances the row counter, and immediately pushes the updated
// framebuffer to the display so status appears incrementally as each
// module starts, rather than all at once at the end.
void OledDisplay::showStatus(const char* name, bool ok) {
    if (_statusRow == 0) {
        clear();
        setCursor(0, 0);
        print("Module         St");
        drawHLine(0, 9, OLED_WIDTH);
        _statusRow = 1;
    }
    if (_statusRow >= FONT_ROWS) return;
    setCursor(0, _statusRow);
    char line[22];
    snprintf(line, sizeof(line), "%-14.14s %s", name, ok ? "ok" : "!!");
    print(line);
    _statusRow++;
    show();
}

// showPower() — draws a large, centred power on/off indicator
// screen: clears the framebuffer, draws a border rectangle, prints a
// large status line ("track power on/off" — see the on-screen Dutch
// text), two horizontal separator lines, and a bracketed on/off
// indicator plus a DCC active/stopped line, then pushes the result to
// the display. Called whenever track power changes state.
void OledDisplay::showPower(bool on) {
    clear();
    drawRect(0, 0, OLED_WIDTH, OLED_HEIGHT, false);
    setCursor(3, 2); print(on ? "  SPANNING AAN" : "  SPANNING UIT");
    drawHLine(10, 28, 108);
    drawHLine(10, 30, 108);
    setCursor(5, 5); print(on ? "   [  AAN  ]" : "   [  UIT  ]");
    setCursor(5, 6); print(on ? "  DCC actief" : " DCC gestopt");
    show();
}

// setContrast() — sends the SH1106 contrast-control command (0x81)
// followed by the requested contrast value (0=dimmest, 255=brightest).
void OledDisplay::setContrast(uint8_t val) {
    _cmd(0x81); _cmd(val);
}

// setInvert() — sends the SH1106 command to display normally (0xA6)
// or with all pixels inverted (0xA7), depending on `invert`.
void OledDisplay::setInvert(bool invert) {
    _cmd(invert ? 0xA7 : 0xA6);
}
