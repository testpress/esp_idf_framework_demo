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

static const char *TAG = "HX711_SCALE";

// --- Calibration state (RAM only) ---
static long offset = 0;           // raw zero point (set by tare)
static double scale = -0.001307;   // g per count (update via calibration)

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
        vTaskDelay(10 / portTICK_PERIOD_MS);  // longer delay to prevent watchdog
        if ((esp_timer_get_time() - start) > timeout_ms * 1000) {
            ESP_LOGW(TAG, "HX711 not ready after %d ms", timeout_ms);
            return false;
        }
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

static long readRawAverage(uint16_t samples) {
    long sum = 0;
    for (uint16_t i = 0; i < samples; ++i) {
        sum += hx711_read_raw();
    }
    return sum / (long)samples;
}

// ------------------- Helper Functions -------------------
static double gramsFromRaw(long raw) {
    return (raw - offset) * scale;
}

static void doTare(uint16_t samples) {
    offset = readRawAverage(samples);
    ESP_LOGI(TAG, "[TARE] OFFSET set to %ld (avg of %u samples)", offset, samples);
}

static void showStatus() {
    ESP_LOGI(TAG, "STATUS: OFFSET=%ld, SCALE=%.8f g/count", offset, scale);
}

// ------------------- Main Application -------------------
void app_main(void) {
    ESP_LOGI(TAG, "\nHX711 Scale – RAM-only calibration");
    ESP_LOGI(TAG, "HX711 Weight Scale Ready (DOUT=%d, SCK=%d)", HX_DOUT, HX_SCK);

    hx711_init();

    // Initial tare with balanced samples for stability and speed
    doTare(20);
    showStatus();

    while (1) {
        long raw = readRawAverage(10);  // balanced averaging for stability and speed
        double grams = gramsFromRaw(raw);

        ESP_LOGI(TAG, "RAW: %ld | Weight: %.3f g", raw, grams);

        vTaskDelay(1000 / portTICK_PERIOD_MS);  // 1 Hz streaming
    }
}
