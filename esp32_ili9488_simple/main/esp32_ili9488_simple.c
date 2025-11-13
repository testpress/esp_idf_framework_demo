#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_system.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/uart.h"
#include "esp_heap_caps.h"

#include "font6x8.h"

static const char *TAG = "ILI9488";

#define PIN_TFT_MOSI   23
#define PIN_TFT_SCLK   18
#define PIN_TFT_CS     5
#define PIN_TFT_DC     13
#define PIN_TFT_RST    4

spi_device_handle_t tft_spi = NULL;

// ===================== LOW LEVEL SPI HELPERS =====================

void ili9488_send_cmd(uint8_t cmd)
{
    gpio_set_level(PIN_TFT_DC, 0);
    spi_transaction_t t = {0};
    t.length = 8;
    t.tx_buffer = &cmd;
    spi_device_polling_transmit(tft_spi, &t);
}

void ili9488_send_data(uint8_t data)
{
    gpio_set_level(PIN_TFT_DC, 1);
    spi_transaction_t t = {0};
    t.length = 8;
    t.tx_buffer = &data;
    spi_device_polling_transmit(tft_spi, &t);
}

// ===================== ILI9488 INIT =====================

void ili9488_init(void)
{
    ESP_LOGI(TAG, "Resetting display...");
    gpio_set_level(PIN_TFT_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(PIN_TFT_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(120));

    ESP_LOGI(TAG, "Sending ILI9488 init...");

    ili9488_send_cmd(0x11);     // Sleep Out
    vTaskDelay(pdMS_TO_TICKS(120));

    ili9488_send_cmd(0x36);     // Memory Access Control
    ili9488_send_data(0xE8);   // Landscape rotation (same as TFT_eSPI rotation=3)

    ili9488_send_cmd(0x3A);     // Pixel Format
    ili9488_send_data(0x66);    // 18-bit/pixel mode (RGB666)

    ili9488_send_cmd(0x29);     // Display On
    vTaskDelay(pdMS_TO_TICKS(10));
}

// ===================== FAST FILL (DMA) =====================

void fill_color(uint16_t color565)
{
    // 565 → 666 expand
    uint8_t r5 = (color565 >> 11) & 0x1F;
    uint8_t g6 = (color565 >> 5) & 0x3F;
    uint8_t b5 =  color565       & 0x1F;

    uint8_t R = r5 << 3;
    uint8_t G = g6 << 2;
    uint8_t B = b5 << 3;

    const int total_pixels  = 480 * 320;
    const int chunk_pixels  = 1200;
    const int chunk_bytes   = chunk_pixels * 3;

    uint8_t *buf = heap_caps_malloc(chunk_bytes, MALLOC_CAP_DMA);
    if (!buf) {
        ESP_LOGE(TAG, "DMA buffer alloc failed");
        return;
    }

    for (int i = 0; i < chunk_pixels; i++) {
        buf[3*i + 0] = R;
        buf[3*i + 1] = G;
        buf[3*i + 2] = B;
    }

    // Column (X): 0 → 479
    ili9488_send_cmd(0x2A);
    ili9488_send_data(0);
    ili9488_send_data(0);
    ili9488_send_data((480-1) >> 8);
    ili9488_send_data((480-1) & 0xFF);

    // Row (Y): 0 → 319
    ili9488_send_cmd(0x2B);
    ili9488_send_data(0);
    ili9488_send_data(0);
    ili9488_send_data((320-1) >> 8);
    ili9488_send_data((320-1) & 0xFF);

    // Memory write
    ili9488_send_cmd(0x2C);
    gpio_set_level(PIN_TFT_DC, 1);

    spi_transaction_t t = {0};
    t.tx_buffer = buf;

    int sent = 0;
    while (sent < total_pixels)
    {
        int to_send = (total_pixels - sent > chunk_pixels)
                        ? chunk_pixels
                        : (total_pixels - sent);

        t.length = to_send * 24; // 24 bits = 3 bytes/pixel

        spi_device_polling_transmit(tft_spi, &t);
        sent += to_send;

        vTaskDelay(pdMS_TO_TICKS(1)); // feed watchdog
    }

    heap_caps_free(buf);
}

// ===================== DRAW PRIMITIVES =====================

void drawPixel(int x, int y, uint16_t color565)
{
    if (x < 0 || x >= 480 || y < 0 || y >= 320) return;

    uint8_t r5 = (color565 >> 11) & 0x1F;
    uint8_t g6 = (color565 >> 5) & 0x3F;
    uint8_t b5 =  color565       & 0x1F;

    uint8_t R = r5 << 3;
    uint8_t G = g6 << 2;
    uint8_t B = b5 << 3;

    // Column (0..479)
    ili9488_send_cmd(0x2A);
    ili9488_send_data(x >> 8);
    ili9488_send_data(x & 0xFF);
    ili9488_send_data(x >> 8);
    ili9488_send_data(x & 0xFF);

    // Row (0..319)
    ili9488_send_cmd(0x2B);
    ili9488_send_data(y >> 8);
    ili9488_send_data(y & 0xFF);
    ili9488_send_data(y >> 8);
    ili9488_send_data(y & 0xFF);

    // Memory write
    ili9488_send_cmd(0x2C);
    gpio_set_level(PIN_TFT_DC, 1);

    uint8_t px[3] = {R, G, B};

    spi_transaction_t t = {0};
    t.length = 24;
    t.tx_buffer = px;

    spi_device_polling_transmit(tft_spi, &t);
}

void drawHLine(int x, int y, int w, uint16_t color)
{
    for (int i = 0; i < w; i++)
        drawPixel(x + i, y, color);
}

void drawVLine(int x, int y, int h, uint16_t color)
{
    for (int i = 0; i < h; i++)
        drawPixel(x, y + i, color);
}

void drawRect(int x, int y, int w, int h, uint16_t color)
{
    drawHLine(x, y, w, color);
    drawHLine(x, y + h - 1, w, color);
    drawVLine(x, y, h, color);
    drawVLine(x + w - 1, y, h, color);
}

void fillRect(int x, int y, int w, int h, uint16_t color)
{
    for (int r = 0; r < h; r++)
        drawHLine(x, y+r, w, color);
}

// ===================== TEXT DRAWING =====================

void drawChar(int x, int y, char c, uint16_t color, uint8_t scale)
{
    if (c < 32 || c > 126) return;

    const uint8_t *glyph = &font6x8[(c - 32) * 6];

    for (int col = 0; col < 6; col++)
    {
        uint8_t line = glyph[col];

        for (int row = 0; row < 8; row++)
        {
            if (line & 1)
            {
                // Draw a scale x scale block
                for (int dx = 0; dx < scale; dx++)
                {
                    for (int dy = 0; dy < scale; dy++)
                    {
                        drawPixel(x + col*scale + dx,
                                  y + row*scale + dy,
                                  color);
                    }
                }
            }

            line >>= 1;
        }
    }
}


void drawText(int x, int y, const char *s, uint16_t color, uint8_t scale)
{
    while (*s)
    {
        drawChar(x, y, *s, color, scale);
        x += 6 * scale;   // move ahead by scaled width
        s++;
    }
}

// ===================== INTERACTIVE SERIAL MENU =====================

void serial_task(void *pv)
{
    const char *menu =
        "\n--- ILI9488 Interactive Menu ---\n"
        "1: Fill RED\n"
        "2: Fill GREEN\n"
        "3: Fill BLUE\n"
        "4: Draw shapes & text\n"
        "5: Clear (BLACK)\n"
        "Enter choice: ";

    uint8_t c;

    while (1)
    {
        printf("%s", menu);

        // Blocking UART read — waits for a real key
        int len = uart_read_bytes(UART_NUM_0, &c, 1, portMAX_DELAY);

        if (len == 1)
        {
            printf("You pressed: %c\n", c);

            switch (c)
            {
                case '1': fill_color(0xF800); break;
                case '2': fill_color(0x07E0); break;
                case '3': fill_color(0x001F); break;

                case '4':
                    // Background
                    fill_color(0x0000);   // Black

                    // Title (centered)
                    drawText(140, 10, "ESP32 + ILI9488", 0xFFFF, 3);   // White, big

                    // Divider line
                    drawHLine(40, 55, 400, 0xFFFF); // white

                    // Info box
                    drawRect(20, 80, 440, 180, 0xFFFF); // white outline

                    drawText(30, 100, "DEVICE:",  0xFFFF, 2);
                    drawText(30, 140, "STATUS:",  0xFFFF, 2);
                    drawText(30, 180, "SCREEN:",  0xFFFF, 2);

                    drawText(170, 100, "ESP32-WROVER", 0x07FF, 2);  // cyan
                    drawText(170, 140, "OK",            0x07E0, 2);  // green
                    drawText(170, 180, "480x320",       0xFFE0, 2);  // yellow

                    // Footer
                    drawText(110, 270, "ILI9488 Test Screen", 0xFFFF, 2);

                    break;

                case '5': fill_color(0x0000); break;

                default:
                    printf("Unknown command\n");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ===================== APP MAIN =====================

void app_main(void)
{
    ESP_LOGI(TAG, "Configuring pins...");
    gpio_config_t io = {0};
    io.mode = GPIO_MODE_OUTPUT;
    io.pin_bit_mask = (1ULL<<PIN_TFT_DC) | (1ULL<<PIN_TFT_RST);
    gpio_config(&io);

    ESP_LOGI(TAG, "Init UART...");
    uart_config_t uart_cfg = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_0, &uart_cfg);
    uart_driver_install(UART_NUM_0, 2048, 0, 0, NULL, 0);

    ESP_LOGI(TAG, "Init SPI...");
    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_TFT_MOSI,
        .miso_io_num = -1,
        .sclk_io_num = PIN_TFT_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 320*480*3
    };
    spi_bus_initialize(HSPI_HOST, &buscfg, SPI_DMA_CH_AUTO);

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 20 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = PIN_TFT_CS,
        .queue_size = 1
    };
    spi_bus_add_device(HSPI_HOST, &devcfg, &tft_spi);

    ili9488_init();

    // initial RGB test
    fill_color(0xF800);
    vTaskDelay(pdMS_TO_TICKS(700));

    fill_color(0x07E0);
    vTaskDelay(pdMS_TO_TICKS(700));

    fill_color(0x001F);
    vTaskDelay(pdMS_TO_TICKS(700));

    ESP_LOGI(TAG, "Starting interactive menu...");
    xTaskCreate(serial_task, "serial_task", 4096, NULL, 5, NULL);
}
