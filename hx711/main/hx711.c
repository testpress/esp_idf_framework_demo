#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "esp_vfs_dev.h"
#include "esp_system.h"

#define HX_DOUT 34
#define HX_SCK  14

#define AVG_SAMPLES 10
#define TARE_SAMPLES 30

static const char *TAG = "HX711_SCALE";

static int32_t offset = 0;      // tare offset
static double scale = 1.0;      // counts per gram
static int direction = 1;       // +1 or -1 depending on wiring (auto detected)

// ------------------- HX711 Functions -------------------
static void hx711_init(void) {
    gpio_reset_pin(HX_DOUT);
    gpio_set_direction(HX_DOUT, GPIO_MODE_INPUT);
    gpio_reset_pin(HX_SCK);
    gpio_set_direction(HX_SCK, GPIO_MODE_OUTPUT);
    gpio_set_level(HX_SCK, 0);
}

static bool hx711_wait_ready(int timeout_ms) {
    int64_t start = esp_timer_get_time();
    while (gpio_get_level(HX_DOUT) == 1) {
        vTaskDelay(1 / portTICK_PERIOD_MS);
        if ((esp_timer_get_time() - start) > timeout_ms * 1000) return false;
    }
    return true;
}

static int32_t hx711_read_raw(void) {
    if (!hx711_wait_ready(500)) return 0;

    uint32_t value = 0;
    for (int i = 0; i < 24; i++) {
        gpio_set_level(HX_SCK, 1);
        esp_rom_delay_us(1);
        value = (value << 1) | gpio_get_level(HX_DOUT);
        gpio_set_level(HX_SCK, 0);
        esp_rom_delay_us(1);
    }

    gpio_set_level(HX_SCK, 1); esp_rom_delay_us(1);
    gpio_set_level(HX_SCK, 0); esp_rom_delay_us(1);

    if (value & 0x800000) value |= 0xFF000000;  // sign extend
    return (int32_t)value;
}

static int32_t hx711_read_average(int samples) {
    int64_t sum = 0;
    for (int i = 0; i < samples; i++) {
        sum += hx711_read_raw();
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    return (int32_t)(sum / samples);
}

// ------------------- Helper Functions -------------------
static void do_tare(void) {
    ESP_LOGI(TAG, "Taring... Remove all weight.");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    offset = hx711_read_average(TARE_SAMPLES);
    ESP_LOGI(TAG, "Tare complete. Offset=%ld", (long)offset);
}

static void do_calibrate(void) {
    char buf[32];
    ESP_LOGI(TAG, "Place a known weight on the scale (in grams) and enter its value:");

    fgets(buf, sizeof(buf), stdin);
    double known_weight = atof(buf);
    if (known_weight <= 0.0) {
        ESP_LOGW(TAG, "Invalid weight entered.");
        return;
    }

    int32_t raw = hx711_read_average(20);
    int32_t diff = raw - offset;

    // Detect direction
    direction = (diff > 0) ? 1 : -1;
    scale = (double)abs(diff) / known_weight;

    ESP_LOGI(TAG, "Calibration done. scale=%.2f counts/g, direction=%s",
             scale, (direction == 1 ? "positive" : "negative"));
}

static double get_weight(void) {
    int32_t raw = hx711_read_average(AVG_SAMPLES);
    double grams = direction * ((double)(raw - offset) / scale);
    return grams;
}

static void record_weight(void) {
    double grams = get_weight();
    ESP_LOGI(TAG, "Recorded Weight: %.2f g", grams);
}

// ------------------- Input Handling -------------------
static int get_char_nonblock(void) {
    int ch = EOF;
    struct timeval tv = {0L, 0L};
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(fileno(stdin), &fds);
    if (select(fileno(stdin)+1, &fds, NULL, NULL, &tv) > 0) {
        ch = getchar();
    }
    return ch;
}

// ------------------- Main Application -------------------
void app_main(void) {
    hx711_init();
    setvbuf(stdin, NULL, _IONBF, 0);

#if CONFIG_VFS_SUPPORT_TERMIOS
    esp_vfs_dev_uart_use_driver(CONFIG_ESP_CONSOLE_UART_NUM);
#else
    ESP_LOGW(TAG, "UART console disabled: interactive commands unavailable.");
#endif

    ESP_LOGI(TAG, "HX711 Weight Scale Ready (DOUT=%d, SCK=%d)", HX_DOUT, HX_SCK);

    do_tare();

    while (1) {
        double grams = get_weight();
        printf("Current weight: %.2f g\r\n", grams);

        int ch = get_char_nonblock();
        if (ch != EOF) {
            switch (ch) {
                case 't': do_tare(); break;
                case 'c': do_calibrate(); break;
                case 'r': record_weight(); break;
                case 'q': ESP_LOGI(TAG, "Exiting..."); return;
                case '\n': case '\r': break; // ignore Enter
                default:
                    ESP_LOGI(TAG, "[t]=tare, [c]=calibrate, [r]=record, [q]=quit");
                    break;
            }
        }

        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}
