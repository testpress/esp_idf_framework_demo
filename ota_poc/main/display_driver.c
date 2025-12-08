#include "display_driver.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "font6x8.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const char *TAG = "display_driver";

static spi_device_handle_t spi;

// ILI9488 Commands
#define ILI9488_SWRESET 0x01
#define ILI9488_SLPOUT 0x11
#define ILI9488_DISPOFF 0x28
#define ILI9488_DISPON 0x29
#define ILI9488_CASET 0x2A
#define ILI9488_PASET 0x2B
#define ILI9488_RAMWR 0x2C
#define ILI9488_MADCTL 0x36
#define ILI9488_COLMOD 0x3A

static void lcd_cmd(uint8_t cmd) {
  gpio_set_level(PIN_TFT_DC, 0);
  spi_transaction_t t = {
      .length = 8,
      .tx_buffer = &cmd,
  };
  spi_device_polling_transmit(spi, &t);
}

static void lcd_data(const uint8_t *data, int len) {
  if (len == 0)
    return;
  gpio_set_level(PIN_TFT_DC, 1);
  spi_transaction_t t = {
      .length = len * 8,
      .tx_buffer = data,
  };
  spi_device_polling_transmit(spi, &t);
}

static void lcd_data_byte(uint8_t data) { lcd_data(&data, 1); }

void lcd_init(void) {
  ESP_LOGI(TAG, "Initializing Display...");

  // GPIO Init
  gpio_config_t io_conf = {
      .pin_bit_mask =
          (1ULL << PIN_TFT_DC) | (1ULL << PIN_TFT_RST) | (1ULL << PIN_TFT_CS),
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  gpio_config(&io_conf);

  gpio_set_level(PIN_TFT_CS, 0);

  // SPI Init
  spi_bus_config_t buscfg = {
      .mosi_io_num = PIN_TFT_MOSI,
      .miso_io_num = -1,
      .sclk_io_num = PIN_TFT_SCLK,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1,
      .max_transfer_sz = 480 * 320 * 3 + 8, // RGB666 needs 3 bytes/pixel
  };
  spi_device_interface_config_t devcfg = {
      .clock_speed_hz = 20 * 1000 * 1000,
      .mode = 0,
      .spics_io_num = PIN_TFT_CS,
      .queue_size = 7,
  };

  ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));
  ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &spi));

  // Reset Sequence
  gpio_set_level(PIN_TFT_RST, 0);
  vTaskDelay(pdMS_TO_TICKS(100));
  gpio_set_level(PIN_TFT_RST, 1);
  vTaskDelay(pdMS_TO_TICKS(100));

  // Initialization Commands
  lcd_cmd(ILI9488_SWRESET);
  vTaskDelay(pdMS_TO_TICKS(100));

  lcd_cmd(ILI9488_SLPOUT);
  vTaskDelay(pdMS_TO_TICKS(100));

  lcd_cmd(ILI9488_MADCTL);
  lcd_data_byte(0xE8); // Landscape

  lcd_cmd(ILI9488_COLMOD);
  lcd_data_byte(0x66); // 18-bit RGB666 (REQUIRED for most ILI9488 SPI modules)

  lcd_cmd(ILI9488_DISPON);
  vTaskDelay(pdMS_TO_TICKS(100));

  lcd_clear(COLOR_BLACK);
}

void lcd_set_window(uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
  uint8_t data[4];

  lcd_cmd(ILI9488_CASET);
  data[0] = (x >> 8) & 0xFF;
  data[1] = x & 0xFF;
  data[2] = ((x + w - 1) >> 8) & 0xFF;
  data[3] = (x + w - 1) & 0xFF;
  lcd_data(data, 4);

  lcd_cmd(ILI9488_PASET);
  data[0] = (y >> 8) & 0xFF;
  data[1] = y & 0xFF;
  data[2] = ((y + h - 1) >> 8) & 0xFF;
  data[3] = (y + h - 1) & 0xFF;
  lcd_data(data, 4);

  lcd_cmd(ILI9488_RAMWR);
}

void lcd_fill(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
  if (x >= HOR_RES || y >= VER_RES)
    return;
  if (x + w > HOR_RES)
    w = HOR_RES - x;
  if (y + h > VER_RES)
    h = VER_RES - y;

  lcd_set_window(x, y, w, h);

  // Convert RGB565 color to RGB666 (3 bytes)
  // RGB565: RRRRRGGG GGGBBBBB
  // RGB666: RRRRRr.. GGGGGG.. BBBBBb..
  uint8_t r = (color >> 8) & 0xF8;
  uint8_t g = (color >> 3) & 0xFC;
  uint8_t b = (color << 3) & 0xF8;

  uint32_t line_len = w * 3;
  uint8_t *buffer = heap_caps_malloc(line_len, MALLOC_CAP_DMA);
  if (!buffer) {
    ESP_LOGE(TAG, "Malloc failed");
    return;
  }

  for (int i = 0; i < w; i++) {
    buffer[i * 3 + 0] = r;
    buffer[i * 3 + 1] = g;
    buffer[i * 3 + 2] = b;
  }

  for (int i = 0; i < h; i++) {
    lcd_data(buffer, line_len);
  }

  free(buffer);
}

void lcd_clear(uint16_t color) { lcd_fill(0, 0, HOR_RES, VER_RES, color); }

void lcd_draw_pixel(uint16_t x, uint16_t y, uint16_t color) {
  if (x >= HOR_RES || y >= VER_RES)
    return;
  lcd_set_window(x, y, 1, 1);

  // Convert to RGB666
  uint8_t data[3];
  data[0] = (color >> 8) & 0xF8;
  data[1] = (color >> 3) & 0xFC;
  data[2] = (color << 3) & 0xF8;

  lcd_data(data, 3);
}

// Simple 6x8 font drawing
void lcd_draw_char(uint16_t x, uint16_t y, char c, uint16_t color,
                   uint16_t bg_color, uint8_t size) {
  if (c < 32 || c > 126)
    return;

  const uint8_t *char_data = &font6x8[(c - 32) * 6];

  for (int i = 0; i < 6; i++) { // 6 columns
    uint8_t line = char_data[i];
    for (int j = 0; j < 8; j++) { // 8 rows
      if (line & 0x01) {
        if (size == 1) {
          lcd_draw_pixel(x + i, y + j, color);
        } else {
          lcd_fill(x + i * size, y + j * size, size, size, color);
        }
      } else if (bg_color !=
                 color) { // Check if we want transparent bg (pass same color)
        if (size == 1) {
          lcd_draw_pixel(x + i, y + j, bg_color);
        } else {
          lcd_fill(x + i * size, y + j * size, size, size, bg_color);
        }
      }
      line >>= 1;
    }
  }
}

void lcd_draw_string(uint16_t x, uint16_t y, const char *str, uint16_t color,
                     uint16_t bg_color, uint8_t size) {
  while (*str) {
    if (x + 6 * size >= HOR_RES) {
      x = 0;
      y += 8 * size;
    }
    lcd_draw_char(x, y, *str, color, bg_color, size);
    x += 6 * size;
    str++;
  }
}
