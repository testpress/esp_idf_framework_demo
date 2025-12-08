#ifndef DISPLAY_DRIVER_H
#define DISPLAY_DRIVER_H

#include "esp_err.h"
#include <stdint.h>

// Display configuration (480x320)
#define HOR_RES 480
#define VER_RES 320

// Display pins (same as camera project)
#define PIN_TFT_MOSI 23
#define PIN_TFT_SCLK 18
#define PIN_TFT_CS 5
#define PIN_TFT_DC 13
#define PIN_TFT_RST 4

// Colors (RGB565)
#define COLOR_BLACK 0x0000
#define COLOR_WHITE 0xFFFF
#define COLOR_RED 0xF800
#define COLOR_GREEN 0x07E0
#define COLOR_BLUE 0x001F
#define COLOR_YELLOW 0xFFE0

// Convert RGB565 to RGB666 (3 bytes)
// R5(k) -> R6: (k & 0xF800) >> 8
// G6(k) -> G6: (k & 0x07E0) >> 3
// B5(k) -> B6: (k & 0x001F) << 1
// Note: To expand 5 to 6 bits accurately usually (val << 1) | (val >> 4).
// Simplification for solid colors often fine, but let's try to be close to
// camera. Camera uses LUT. We will implement simple shift.

void lcd_init(void);
void lcd_fill(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
void lcd_draw_char(uint16_t x, uint16_t y, char c, uint16_t color,
                   uint16_t bg_color, uint8_t size);
void lcd_draw_string(uint16_t x, uint16_t y, const char *str, uint16_t color,
                     uint16_t bg_color, uint8_t size);
void lcd_clear(uint16_t color);

#endif
