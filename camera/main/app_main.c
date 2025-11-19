#include "ArducamCamera.h"
#include "ArducamLink.h"
#include "delay.h"
#include "driver/gpio.h"
#include "esp_event.h"
#include "esp_http_client.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "uart.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


// LVGL includes
#include "lvgl.h"
#include "driver/spi_master.h"

#define TAG "camera_server"

// ---- LVGL Display configuration ----
// Force recompilation after font fixes
#define HOR_RES 480
#define VER_RES 320

// ---- Display pins ----
#define PIN_TFT_MOSI   23
#define PIN_TFT_SCLK   18
#define PIN_TFT_CS      5
#define PIN_TFT_DC     13
#define PIN_TFT_RST     4

// ---- HX711 Load Cell pins ----
#define HX_DOUT 34
#define HX_SCK  14

// ---- WiFi config (change to your AP) ----
#define WIFI_SSID "Testpress_4G"
#define WIFI_PASS "Tp@12345"
#define MAXIMUM_RETRY 5

// ---- HTTP upload config (change URL/token) ----
static const char *UPLOAD_URL = "http://192.168.0.4:8000/api/v1/meal_logs/";
static const char *AUTH_HEADER_VALUE = "Bearer iKnCz7E2Mtfl_V0bDcasJWDMzVN39L_BCySvj1hLDSc";

// ---- Camera globals ----
ArducamCamera myCAM;
const int CS_PIN = 33;
static httpd_handle_t server = NULL;

// ---- LVGL Display globals ----
spi_device_handle_t tft_spi = NULL;
static lv_display_t *disp = NULL;
static lv_color_t *lv_buf1 = NULL;
static lv_color_t *lv_buf2 = NULL;
static uint8_t *rgb666_buf = NULL;

// ---- UI Status globals ----
static char ui_status_message[128] = "Initializing...";
static bool ui_status_update = false;

// ---- UI Status functions ----
static void update_ui_status(const char *message)
{
    strncpy(ui_status_message, message, sizeof(ui_status_message) - 1);
    ui_status_message[sizeof(ui_status_message) - 1] = '\0';
    ui_status_update = true;
}

// ---- HX711 globals ----
static long hx711_offset = 0;           // raw zero point (set by tare)
static double hx711_scale = -0.001307;   // g per count (update via calibration)

// WiFi event group
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static int s_retry_num = 0;

// ---- LVGL Display functions ----

// RGB565 to RGB666 conversion lookup tables (from LVGL example)
static const uint8_t rgb565_r_lut[32] = {
    0x00, 0x08, 0x10, 0x18, 0x20, 0x28, 0x30, 0x38,
    0x40, 0x48, 0x50, 0x58, 0x60, 0x68, 0x70, 0x78,
    0x80, 0x88, 0x90, 0x98, 0xA0, 0xA8, 0xB0, 0xB8,
    0xC0, 0xC8, 0xD0, 0xD8, 0xE0, 0xE8, 0xF0, 0xF8
};

static const uint8_t rgb565_g_lut[64] = {
    0x00, 0x04, 0x08, 0x0C, 0x10, 0x14, 0x18, 0x1C,
    0x20, 0x24, 0x28, 0x2C, 0x30, 0x34, 0x38, 0x3C,
    0x40, 0x44, 0x48, 0x4C, 0x50, 0x54, 0x58, 0x5C,
    0x60, 0x64, 0x68, 0x6C, 0x70, 0x74, 0x78, 0x7C,
    0x80, 0x84, 0x88, 0x8C, 0x90, 0x94, 0x98, 0x9C,
    0xA0, 0xA4, 0xA8, 0xAC, 0xB0, 0xB4, 0xB8, 0xBC,
    0xC0, 0xC4, 0xC8, 0xCC, 0xD0, 0xD4, 0xD8, 0xDC,
    0xE0, 0xE4, 0xE8, 0xEC, 0xF0, 0xF4, 0xF8, 0xFC
};

static const uint8_t rgb565_b_lut[32] = {
    0x00, 0x08, 0x10, 0x18, 0x20, 0x28, 0x30, 0x38,
    0x40, 0x48, 0x50, 0x58, 0x60, 0x68, 0x70, 0x78,
    0x80, 0x88, 0x90, 0x98, 0xA0, 0xA8, 0xB0, 0xB8,
    0xC0, 0xC8, 0xD0, 0xD8, 0xE0, 0xE8, 0xF0, 0xF8
};

// Low-level SPI helpers
static void send_cmd(uint8_t cmd)
{
    gpio_set_level(PIN_TFT_DC, 0);
    spi_transaction_t t = {0};
    t.length = 8;
    t.flags = SPI_TRANS_USE_TXDATA;
    t.tx_data[0] = cmd;
    t.rxlength = 0;
    esp_err_t err = spi_device_polling_transmit(tft_spi, &t);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "send_cmd spi tx err %s", esp_err_to_name(err));
    }
}

static void send_data_chunked(const uint8_t *data, size_t len)
{
    if (!data || len == 0) return;
    gpio_set_level(PIN_TFT_DC, 1);

    spi_transaction_t t = {0};
    size_t sent = 0;

    while (sent < len) {
        size_t to_send = len - sent;
        if (to_send > 16384) {  // MAX_SPI_TRANS_BYTES equivalent
            to_send = 16384;
        }

        t = (spi_transaction_t){0};
        t.length = to_send * 8;
        t.tx_buffer = data + sent;
        t.rxlength = 0;

        esp_err_t err = spi_device_polling_transmit(tft_spi, &t);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "send_data_chunked spi tx err %s", esp_err_to_name(err));
            break;
        }
        sent += to_send;
    }
}

// ILI9488 initialization
static void ili9488_init(void)
{
    ESP_LOGI(TAG, "Resetting display...");
    gpio_set_level(PIN_TFT_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(PIN_TFT_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(120));

    ESP_LOGI(TAG, "Sending ILI9488 init...");

    send_cmd(0x11); // Sleep Out
    vTaskDelay(pdMS_TO_TICKS(120));

    send_cmd(0x36); // MADCTL
    uint8_t madctl = 0xE8; // landscape
    send_data_chunked(&madctl, 1);

    send_cmd(0x3A); // Pixel format
    uint8_t pix = 0x66; // RGB666
    send_data_chunked(&pix, 1);

    send_cmd(0x29); // Display ON
    vTaskDelay(pdMS_TO_TICKS(20));
}

// Fast RGB565 to RGB666 conversion
static inline void rgb565_to_rgb666_optimized(const uint16_t *src, uint8_t *dst, int count)
{
    int i = 0;
    const int unroll_count = count & ~15;

    for (; i < unroll_count; i += 16) {
        uint16_t c0 = src[i], c1 = src[i+1], c2 = src[i+2], c3 = src[i+3];
        uint16_t c4 = src[i+4], c5 = src[i+5], c6 = src[i+6], c7 = src[i+7];
        uint16_t c8 = src[i+8], c9 = src[i+9], c10 = src[i+10], c11 = src[i+11];
        uint16_t c12 = src[i+12], c13 = src[i+13], c14 = src[i+14], c15 = src[i+15];

        uint8_t *d = dst + 3*i;

        d[0] = rgb565_r_lut[(c0 >> 11) & 0x1F]; d[1] = rgb565_g_lut[(c0 >> 5) & 0x3F]; d[2] = rgb565_b_lut[c0 & 0x1F];
        d[3] = rgb565_r_lut[(c1 >> 11) & 0x1F]; d[4] = rgb565_g_lut[(c1 >> 5) & 0x3F]; d[5] = rgb565_b_lut[c1 & 0x1F];
        d[6] = rgb565_r_lut[(c2 >> 11) & 0x1F]; d[7] = rgb565_g_lut[(c2 >> 5) & 0x3F]; d[8] = rgb565_b_lut[c2 & 0x1F];
        d[9] = rgb565_r_lut[(c3 >> 11) & 0x1F]; d[10] = rgb565_g_lut[(c3 >> 5) & 0x3F]; d[11] = rgb565_b_lut[c3 & 0x1F];
        d[12] = rgb565_r_lut[(c4 >> 11) & 0x1F]; d[13] = rgb565_g_lut[(c4 >> 5) & 0x3F]; d[14] = rgb565_b_lut[c4 & 0x1F];
        d[15] = rgb565_r_lut[(c5 >> 11) & 0x1F]; d[16] = rgb565_g_lut[(c5 >> 5) & 0x3F]; d[17] = rgb565_b_lut[c5 & 0x1F];
        d[18] = rgb565_r_lut[(c6 >> 11) & 0x1F]; d[19] = rgb565_g_lut[(c6 >> 5) & 0x3F]; d[20] = rgb565_b_lut[c6 & 0x1F];
        d[21] = rgb565_r_lut[(c7 >> 11) & 0x1F]; d[22] = rgb565_g_lut[(c7 >> 5) & 0x3F]; d[23] = rgb565_b_lut[c7 & 0x1F];
        d[24] = rgb565_r_lut[(c8 >> 11) & 0x1F]; d[25] = rgb565_g_lut[(c8 >> 5) & 0x3F]; d[26] = rgb565_b_lut[c8 & 0x1F];
        d[27] = rgb565_r_lut[(c9 >> 11) & 0x1F]; d[28] = rgb565_g_lut[(c9 >> 5) & 0x3F]; d[29] = rgb565_b_lut[c9 & 0x1F];
        d[30] = rgb565_r_lut[(c10 >> 11) & 0x1F]; d[31] = rgb565_g_lut[(c10 >> 5) & 0x3F]; d[32] = rgb565_b_lut[c10 & 0x1F];
        d[33] = rgb565_r_lut[(c11 >> 11) & 0x1F]; d[34] = rgb565_g_lut[(c11 >> 5) & 0x3F]; d[35] = rgb565_b_lut[c11 & 0x1F];
        d[36] = rgb565_r_lut[(c12 >> 11) & 0x1F]; d[37] = rgb565_g_lut[(c12 >> 5) & 0x3F]; d[38] = rgb565_b_lut[c12 & 0x1F];
        d[39] = rgb565_r_lut[(c13 >> 11) & 0x1F]; d[40] = rgb565_g_lut[(c13 >> 5) & 0x3F]; d[41] = rgb565_b_lut[c13 & 0x1F];
        d[42] = rgb565_r_lut[(c14 >> 11) & 0x1F]; d[43] = rgb565_g_lut[(c14 >> 5) & 0x3F]; d[44] = rgb565_b_lut[c14 & 0x1F];
        d[45] = rgb565_r_lut[(c15 >> 11) & 0x1F]; d[46] = rgb565_g_lut[(c15 >> 5) & 0x3F]; d[47] = rgb565_b_lut[c15 & 0x1F];
    }

    for (; i < count; i++) {
        uint16_t c = src[i];
        uint8_t *d = dst + 3*i;
        d[0] = rgb565_r_lut[(c >> 11) & 0x1F];
        d[1] = rgb565_g_lut[(c >> 5) & 0x3F];
        d[2] = rgb565_b_lut[c & 0x1F];
    }
}

// LVGL flush callback
static void ili9488_flush(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
    uint16_t *src = (uint16_t *)px_map;
    int32_t w = lv_area_get_width(area);
    int32_t h = lv_area_get_height(area);
    int total = w * h;

    // Convert RGB565 to RGB666
    rgb565_to_rgb666_optimized(src, rgb666_buf, total);

    // Set column address (X coordinates)
    uint8_t col[4] = {
        (uint8_t)((area->x1 >> 8) & 0xFF), (uint8_t)(area->x1 & 0xFF),
        (uint8_t)((area->x2 >> 8) & 0xFF), (uint8_t)(area->x2 & 0xFF)
    };
    send_cmd(0x2A);
    send_data_chunked(col, 4);

    // Set row address (Y coordinates)
    uint8_t row[4] = {
        (uint8_t)((area->y1 >> 8) & 0xFF), (uint8_t)(area->y1 & 0xFF),
        (uint8_t)((area->y2 >> 8) & 0xFF), (uint8_t)(area->y2 & 0xFF)
    };
    send_cmd(0x2B);
    send_data_chunked(row, 4);

    // Send RGB666 pixel data (3 bytes per pixel)
    send_cmd(0x2C);
    send_data_chunked(rgb666_buf, total * 3);

    lv_display_flush_ready(disp);
}


// ---- HX711 Load Cell functions ----
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

static long hx711_read_raw_average(uint16_t samples) {
    long sum = 0;
    for (uint16_t i = 0; i < samples; ++i) {
        sum += hx711_read_raw();
    }
    return sum / (long)samples;
}

static double hx711_grams_from_raw(long raw) {
    return (raw - hx711_offset) * hx711_scale;
}

static void hx711_do_tare(uint16_t samples) {
    hx711_offset = hx711_read_raw_average(samples);
    ESP_LOGI(TAG, "[HX711 TARE] OFFSET set to %ld (avg of %u samples)", hx711_offset, samples);
}

static double hx711_get_weight_grams(void) {
    long raw = hx711_read_raw_average(10);  // balanced averaging for stability and speed
    return hx711_grams_from_raw(raw);
}

/* ---------- WiFi ---------- */
static void event_handler(void* arg, esp_event_base_t event_base,
                          int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "Retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG, "Connect to the AP failed");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Connected to AP SSID:%s", WIFI_SSID);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGE(TAG, "Failed to connect to SSID:%s", WIFI_SSID);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
}

/* ---------- Helper: POST multipart/form-data ---------- */

// Structure to hold response data
typedef struct {
    char *buffer;
    int len;
    int max_len;
} response_buffer_t;

// HTTP event handler to capture response
static esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
    response_buffer_t *resp = (response_buffer_t *)evt->user_data;
    
    switch(evt->event_id) {
        case HTTP_EVENT_ON_DATA:
            // Append received data to our buffer
            if (resp && resp->buffer && evt->data_len > 0) {
                int copy_len = evt->data_len;
                if (resp->len + copy_len >= resp->max_len) {
                    copy_len = resp->max_len - resp->len - 1;
                }
                if (copy_len > 0) {
                    memcpy(resp->buffer + resp->len, evt->data, copy_len);
                    resp->len += copy_len;
                    resp->buffer[resp->len] = 0;
                }
            }
            break;
        default:
            break;
    }
    return ESP_OK;
}


static esp_err_t send_image_to_server(const uint8_t *jpeg, size_t jpeg_len, double weight_grams)
{
    const char *url = UPLOAD_URL;
    const char *auth_header_value = AUTH_HEADER_VALUE;

    const char *timestamp_value = "2024-01-15T12:30:00Z";
    char weight_value[16];
    snprintf(weight_value, sizeof(weight_value), "%.2f", weight_grams);
    const char *user_id_value = "11";
    const char *file_field_name = "image_file";
    const char *file_name = "capture.jpg";
    const char *file_mime = "image/jpeg";
    const char *boundary = "----ESP32FormBoundary7MA4YWxkTrZu0gW";

    char header_part[512];
    int n;
    size_t total_size = 0;

    // Calculate text parts size
    n = snprintf(header_part, sizeof(header_part),
                 "--%s\r\n"
                 "Content-Disposition: form-data; name=\"timestamp\"\r\n\r\n"
                 "%s\r\n",
                 boundary, timestamp_value);
    if (n < 0) return ESP_ERR_INVALID_ARG;
    total_size += (size_t)n;

    n = snprintf(header_part, sizeof(header_part),
                 "--%s\r\n"
                 "Content-Disposition: form-data; name=\"weight_g\"\r\n\r\n"
                 "%s\r\n",
                 boundary, weight_value);
    if (n < 0) return ESP_ERR_INVALID_ARG;
    total_size += (size_t)n;

    n = snprintf(header_part, sizeof(header_part),
                 "--%s\r\n"
                 "Content-Disposition: form-data; name=\"user_id\"\r\n\r\n"
                 "%s\r\n",
                 boundary, user_id_value);
    if (n < 0) return ESP_ERR_INVALID_ARG;
    total_size += (size_t)n;

    // File header
    char file_header[512];
    n = snprintf(file_header, sizeof(file_header),
                 "--%s\r\n"
                 "Content-Disposition: form-data; name=\"%s\"; filename=\"%s\"\r\n"
                 "Content-Type: %s\r\n\r\n",
                 boundary, file_field_name, file_name, file_mime);
    if (n < 0) return ESP_ERR_INVALID_ARG;
    size_t file_header_len = (size_t)n;
    total_size += file_header_len;

    total_size += jpeg_len;

    // Ending boundary
    char ending[64];
    n = snprintf(ending, sizeof(ending), "\r\n--%s--\r\n", boundary);
    if (n < 0) return ESP_ERR_INVALID_ARG;
    size_t ending_len = (size_t)n;
    total_size += ending_len;

    // Allocate body in PSRAM
    uint8_t *body = heap_caps_malloc(total_size + 1, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!body) {
        ESP_LOGE(TAG, "Failed to allocate PSRAM for multipart body size=%u", (unsigned)total_size);
        return ESP_ERR_NO_MEM;
    }

    // Fill body
    size_t offset = 0;
    int m;

    m = snprintf((char*)body + offset, 256,
                 "--%s\r\n"
                 "Content-Disposition: form-data; name=\"timestamp\"\r\n\r\n"
                 "%s\r\n", boundary, timestamp_value);
    if (m < 0) { heap_caps_free(body); return ESP_FAIL; }
    offset += (size_t)m;

    m = snprintf((char*)body + offset, 256,
                 "--%s\r\n"
                 "Content-Disposition: form-data; name=\"weight_g\"\r\n\r\n"
                 "%s\r\n", boundary, weight_value);
    if (m < 0) { heap_caps_free(body); return ESP_FAIL; }
    offset += (size_t)m;

    m = snprintf((char*)body + offset, 256,
                 "--%s\r\n"
                 "Content-Disposition: form-data; name=\"user_id\"\r\n\r\n"
                 "%s\r\n", boundary, user_id_value);
    if (m < 0) { heap_caps_free(body); return ESP_FAIL; }
    offset += (size_t)m;

    memcpy(body + offset, file_header, file_header_len);
    offset += file_header_len;

    memcpy(body + offset, jpeg, jpeg_len);
    offset += jpeg_len;

    memcpy(body + offset, ending, ending_len);
    offset += ending_len;

    if (offset != total_size) {
        ESP_LOGW(TAG, "constructed multipart size mismatch (offset=%u total=%u)", 
                 (unsigned)offset, (unsigned)total_size);
    }

    // Allocate response buffer
    response_buffer_t response;
    response.max_len = 4096;
    response.buffer = heap_caps_malloc(response.max_len, MALLOC_CAP_8BIT);
    response.len = 0;
    
    if (!response.buffer) {
        ESP_LOGE(TAG, "Failed to allocate response buffer");
        heap_caps_free(body);
        return ESP_ERR_NO_MEM;
    }
    response.buffer[0] = 0;

    // Prepare HTTP client with event handler
    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_POST,
        .timeout_ms = 15000,
        .event_handler = _http_event_handler,
        .user_data = &response,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) {
        ESP_LOGE(TAG, "esp_http_client_init failed");
        heap_caps_free(response.buffer);
        heap_caps_free(body);
        return ESP_FAIL;
    }

    // Set headers
    char content_type_hdr[128];
    snprintf(content_type_hdr, sizeof(content_type_hdr), 
             "multipart/form-data; boundary=%s", boundary);
    esp_http_client_set_header(client, "Content-Type", content_type_hdr);
    esp_http_client_set_header(client, "Authorization", auth_header_value);
    esp_http_client_set_header(client, "Accept", "application/json");

    // Set POST body
    esp_err_t err = esp_http_client_set_post_field(client, (const char*)body, total_size);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_http_client_set_post_field failed: %d", err);
        esp_http_client_cleanup(client);
        heap_caps_free(response.buffer);
        heap_caps_free(body);
        return err;
    }

    // Perform POST (response will be captured by event handler)
    ESP_LOGI(TAG, "Attempting HTTP POST to: %s", url);
    err = esp_http_client_perform(client);

    if (err == ESP_OK) {
        int status = esp_http_client_get_status_code(client);
        int content_length = esp_http_client_get_content_length(client);
    
        ESP_LOGI(TAG, "Upload finished. HTTP status = %d, content_length = %d",
                 status, content_length);
    
        ESP_LOGI(TAG, "FastAPI Response (%d bytes): %s", response.len, response.buffer);

        if (status == 200 || status == 201) {
            update_ui_status("Upload successful!");
            // Parse and display entry data would go here in a more complete implementation
        } else {
            // ---- ERROR CASE ----
            char status_msg[64];
            snprintf(status_msg, sizeof(status_msg), "Upload error - HTTP %d", status);
            update_ui_status(status_msg);
        }

    } else {
        // ---- NETWORK ERROR ----
        ESP_LOGE(TAG, "HTTP POST failed: %d", err);

        // Log more details about the error
        switch(err) {
            case ESP_ERR_HTTP_CONNECT:
                ESP_LOGE(TAG, "Connection failed - check server IP/port and if server is running");
                break;
            case ESP_ERR_HTTP_WRITE_DATA:
                ESP_LOGE(TAG, "Failed to write data to server");
                break;
            case ESP_ERR_HTTP_FETCH_HEADER:
                ESP_LOGE(TAG, "Failed to fetch HTTP headers");
                break;
            case ESP_ERR_HTTP_INVALID_TRANSPORT:
                ESP_LOGE(TAG, "Invalid transport");
                break;
            case ESP_ERR_HTTP_CONNECTING:
                ESP_LOGE(TAG, "Still connecting...");
                break;
            default:
                ESP_LOGE(TAG, "Unknown HTTP error: %d", err);
                break;
        }

        char error_msg[64];
        snprintf(error_msg, sizeof(error_msg), "Network error: %d", err);
        update_ui_status(error_msg);
    }
    

    esp_http_client_cleanup(client);
    heap_caps_free(response.buffer);
    heap_caps_free(body);
    return err;
}

/* ---------- Simple web server handlers ---------- */

static esp_err_t root_handler(httpd_req_t *req)
{
    const char* html = "<!DOCTYPE html>"
                       "<html><head><title>ESP32 Camera</title></head>"
                       "<body><h1>ESP32 Camera Server</h1>"
                       "<p><a href='/capture'>Capture Image</a></p>"
                       "</body></html>";
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, html, strlen(html));
}

static esp_err_t favicon_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "image/x-icon");
    return httpd_resp_send(req, "", 0);
}

static esp_err_t capture_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "Capture request received");

    // Read current weight from load cell
    double current_weight = hx711_get_weight_grams();
    ESP_LOGI(TAG, "Current weight: %.2f grams", current_weight);

    CamStatus status = takePicture(&myCAM, CAM_IMAGE_MODE_VGA, CAM_IMAGE_PIX_FMT_JPG);
    if (status != CAM_ERR_SUCCESS) {
        ESP_LOGE(TAG, "Failed to take picture, status: %d", status);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    const TickType_t wait_tick = pdMS_TO_TICKS(50);
    const int max_wait_cycles = 40;
    int wait_count = 0;
    uint32_t image_length = 0;

    ESP_LOGI(TAG, "Waiting for image to be ready...");
    while (wait_count < max_wait_cycles) {
        captureThread(&myCAM);
        vTaskDelay(wait_tick);
        image_length = imageAvailable(&myCAM);
        if (image_length > 0) break;
        wait_count++;
    }

    if (image_length == 0) {
        ESP_LOGE(TAG, "Timeout waiting for image (no data available)");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    if (image_length > 2000000) {
        ESP_LOGE(TAG, "Image too large: %u", (unsigned)image_length);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Image length: %u bytes", (unsigned)image_length);

    uint8_t *image_data = heap_caps_malloc(image_length, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!image_data) {
        ESP_LOGE(TAG, "Failed to allocate PSRAM for image!");
        httpd_resp_send_500(req);
        return ESP_ERR_NO_MEM;
    }

    const uint32_t chunk_size = 200;
    uint8_t *tmp_buf = heap_caps_malloc(chunk_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!tmp_buf) {
        ESP_LOGE(TAG, "Failed to allocate tmp buffer!");
        heap_caps_free(image_data);
        httpd_resp_send_500(req);
        return ESP_ERR_NO_MEM;
    }

    uint32_t bytes_read = 0;
    while (bytes_read < image_length) {
        uint32_t remaining = image_length - bytes_read;
        uint32_t to_read = (remaining > chunk_size) ? chunk_size : remaining;

        uint8_t got = 0;
        int retry = 0;
        const int max_read_retries = 5;
        while (retry < max_read_retries) {
            captureThread(&myCAM);
            got = readBuff(&myCAM, tmp_buf, (uint8_t)to_read);
            if (got > 0) break;
            vTaskDelay(pdMS_TO_TICKS(10 + retry*5));
            retry++;
        }

        if (got == 0) {
            ESP_LOGE(TAG, "Failed to read chunk after retries (read %u/%u bytes)",
                     (unsigned)bytes_read, (unsigned)image_length);
            heap_caps_free(tmp_buf);
            heap_caps_free(image_data);
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }

        memcpy(image_data + bytes_read, tmp_buf, got);
        bytes_read += got;
    }

    heap_caps_free(tmp_buf);

    if (bytes_read != image_length) {
        ESP_LOGE(TAG, "Incomplete read: %u/%u", (unsigned)bytes_read, (unsigned)image_length);
        heap_caps_free(image_data);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    esp_err_t post_err = send_image_to_server(image_data, image_length, current_weight);
    if (post_err != ESP_OK) {
        ESP_LOGW(TAG, "send_image_to_server failed: %d", post_err);
    } else {
        ESP_LOGI(TAG, "Image uploaded to backend successfully");
    }

    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=\"capture.jpg\"");
    char cl_hdr[32];
    snprintf(cl_hdr, sizeof(cl_hdr), "%u", (unsigned)image_length);
    httpd_resp_set_hdr(req, "Content-Length", cl_hdr);

    esp_err_t err = httpd_resp_send(req, (const char*)image_data, image_length);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "httpd_resp_send failed: %d", err);
    }

    heap_caps_free(image_data);
    return err;
}

/* ---------- Start/Stop webserver ---------- */

httpd_handle_t start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;

    ESP_LOGI(TAG, "Starting server on port: %d", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &(httpd_uri_t){
            .uri       = "/",
            .method    = HTTP_GET,
            .handler   = root_handler,
            .user_ctx  = NULL
        });
        httpd_register_uri_handler(server, &(httpd_uri_t){
            .uri       = "/capture",
            .method    = HTTP_GET,
            .handler   = capture_handler,
            .user_ctx  = NULL
        });
        httpd_register_uri_handler(server, &(httpd_uri_t){
            .uri       = "/favicon.ico",
            .method    = HTTP_GET,
            .handler   = favicon_handler,
            .user_ctx  = NULL
        });
        return server;
    }
    ESP_LOGE(TAG, "Error starting server!");
    return NULL;
}

void stop_preview(void)
{
    ESP_LOGI(TAG, "stop_preview() called");
}

/* -------------------------------------------------------------------
 * LVGL tick
 * -------------------------------------------------------------------*/
static void tick_inc(void *arg)
{
    lv_tick_inc(1);
}

/* ---------- LVGL Task ---------- */

static lv_obj_t *status_label = NULL;
static lv_obj_t *weight_label = NULL;
static lv_obj_t *capture_btn = NULL;

static void capture_btn_handler(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_CLICKED) {
        ESP_LOGI(TAG, "Capture button pressed");
        // Capture will be handled by web server endpoint
    }
}

static void create_camera_ui(void)
{
    lv_obj_t *scr = lv_screen_active();

    /* Professional dark theme background */
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x0A0E27), LV_PART_MAIN);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

    /* Title */
    lv_obj_t *title = lv_label_create(scr);
    lv_label_set_text(title, "ESP32 Camera Server");
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
    // Use default font
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 20);

    /* Status card */
    lv_obj_t *status_card = lv_obj_create(scr);
    lv_obj_set_size(status_card, 440, 80);
    lv_obj_align(status_card, LV_ALIGN_TOP_MID, 0, 60);
    lv_obj_set_style_bg_color(status_card, lv_color_hex(0x1A1F3A), LV_PART_MAIN);
    lv_obj_set_style_border_width(status_card, 2, LV_PART_MAIN);
    lv_obj_set_style_border_color(status_card, lv_color_hex(0x667EEA), LV_PART_MAIN);
    lv_obj_set_style_radius(status_card, 10, LV_PART_MAIN);
    lv_obj_clear_flag(status_card, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *status_title = lv_label_create(status_card);
    lv_label_set_text(status_title, "Status");
    lv_obj_set_style_text_color(status_title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(status_title, LV_ALIGN_TOP_LEFT, 10, 5);

    status_label = lv_label_create(status_card);
    lv_label_set_text(status_label, "Initializing...");
    lv_obj_set_style_text_color(status_label, lv_color_hex(0x9CA3AF), 0);
    lv_obj_align(status_label, LV_ALIGN_TOP_LEFT, 10, 30);

    /* Weight card */
    lv_obj_t *weight_card = lv_obj_create(scr);
    lv_obj_set_size(weight_card, 440, 80);
    lv_obj_align_to(weight_card, status_card, LV_ALIGN_OUT_BOTTOM_MID, 0, 20);
    lv_obj_set_style_bg_color(weight_card, lv_color_hex(0x1A1F3A), LV_PART_MAIN);
    lv_obj_set_style_border_width(weight_card, 2, LV_PART_MAIN);
    lv_obj_set_style_border_color(weight_card, lv_color_hex(0x4ECDC4), LV_PART_MAIN);
    lv_obj_set_style_radius(weight_card, 10, LV_PART_MAIN);
    lv_obj_clear_flag(weight_card, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *weight_title = lv_label_create(weight_card);
    lv_label_set_text(weight_title, "Current Weight");
    lv_obj_set_style_text_color(weight_title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(weight_title, LV_ALIGN_TOP_LEFT, 10, 5);

    weight_label = lv_label_create(weight_card);
    lv_label_set_text(weight_label, "-- g");
    lv_obj_set_style_text_color(weight_label, lv_color_hex(0x4ECDC4), 0);
    // Use default font
    lv_obj_align(weight_label, LV_ALIGN_TOP_LEFT, 10, 30);

    /* Capture button */
    capture_btn = lv_button_create(scr);
    lv_obj_set_size(capture_btn, 200, 60);
    lv_obj_align(capture_btn, LV_ALIGN_BOTTOM_MID, 0, -20);
    lv_obj_set_style_bg_color(capture_btn, lv_color_hex(0x2196F3), LV_PART_MAIN);
    lv_obj_set_style_radius(capture_btn, 10, LV_PART_MAIN);
    lv_obj_add_event_cb(capture_btn, capture_btn_handler, LV_EVENT_CLICKED, NULL);

    lv_obj_t *btn_label = lv_label_create(capture_btn);
    lv_label_set_text(btn_label, "CAPTURE");
    lv_obj_set_style_text_color(btn_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_center(btn_label);

    ESP_LOGI(TAG, "Camera UI created");
}

static void lvgl_task(void *arg)
{
    /* Create UI after LVGL task starts */
    vTaskDelay(pdMS_TO_TICKS(100));
    create_camera_ui();

    uint32_t update_counter = 0;

    while (1) {
        /* Process LVGL timers */
        lv_timer_handler();

        /* Update weight display every 1 second */
        if (update_counter % 100 == 0 && weight_label) {
            double current_weight = hx711_get_weight_grams();
            char weight_str[32];
            snprintf(weight_str, sizeof(weight_str), "%.1f g", current_weight);
            lv_label_set_text(weight_label, weight_str);
        }

        /* Update status every 5 seconds or when status changes */
        if ((update_counter % 500 == 0 || ui_status_update) && status_label) {
            lv_label_set_text(status_label, ui_status_message);
            ui_status_update = false;
        }

        update_counter++;
        vTaskDelay(pdMS_TO_TICKS(10)); // 10ms delay
    }
}

/* ---------- app_main ---------- */

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    uartBegin(115200);
    ESP_LOGI(TAG, "ESP32 Camera Server starting...");

    // Initialize display
    ESP_LOGI(TAG, "Initializing display...");
    gpio_config_t io = {0};
    io.mode = GPIO_MODE_OUTPUT;
    io.pin_bit_mask = (1ULL<<PIN_TFT_DC) | (1ULL<<PIN_TFT_RST);
    gpio_config(&io);

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

    /* Initialize LVGL */
    ESP_LOGI(TAG, "Initializing LVGL...");
    lv_init();

    /* LVGL tick */
    const esp_timer_create_args_t tick_args = {
        .callback = &tick_inc,
        .name = "lv_tick"
    };
    esp_timer_handle_t tmr;
    esp_timer_create(&tick_args, &tmr);
    esp_timer_start_periodic(tmr, 1000);

    /* Allocate RGB666 conversion buffer from PSRAM */
    size_t rgb666_buf_size = HOR_RES * VER_RES * 3;
    rgb666_buf = heap_caps_malloc(rgb666_buf_size, MALLOC_CAP_SPIRAM);
    if (!rgb666_buf) {
        rgb666_buf = heap_caps_malloc(rgb666_buf_size, MALLOC_CAP_DEFAULT);
        if (!rgb666_buf) {
            ESP_LOGE(TAG, "Failed to allocate rgb666_buf");
            return;
        }
        ESP_LOGW(TAG, "rgb666_buf allocated from regular heap");
    }

    /* Allocate LVGL full-screen buffers from PSRAM */
    size_t lvgl_buf_size_bytes = HOR_RES * VER_RES * sizeof(lv_color_t);
    lv_buf1 = heap_caps_malloc(lvgl_buf_size_bytes, MALLOC_CAP_SPIRAM);
    lv_buf2 = heap_caps_malloc(lvgl_buf_size_bytes, MALLOC_CAP_SPIRAM);

    if (!lv_buf1 || !lv_buf2) {
        ESP_LOGW(TAG, "PSRAM allocation failed, trying regular heap");
        if (lv_buf1) free(lv_buf1);
        if (lv_buf2) free(lv_buf2);
        lv_buf1 = heap_caps_malloc(lvgl_buf_size_bytes, MALLOC_CAP_DEFAULT);
        lv_buf2 = heap_caps_malloc(lvgl_buf_size_bytes, MALLOC_CAP_DEFAULT);
        if (!lv_buf1 || !lv_buf2) {
            ESP_LOGE(TAG, "Failed to allocate LVGL buffers!");
            return;
        }
        ESP_LOGW(TAG, "Using regular heap for LVGL buffers");
    }

    /* Initialize LVGL draw buffers */
    static lv_draw_buf_t draw_buf1, draw_buf2;
    lv_draw_buf_init(&draw_buf1, HOR_RES, VER_RES, LV_COLOR_FORMAT_RGB565, LV_STRIDE_AUTO, lv_buf1, lvgl_buf_size_bytes);
    lv_draw_buf_init(&draw_buf2, HOR_RES, VER_RES, LV_COLOR_FORMAT_RGB565, LV_STRIDE_AUTO, lv_buf2, lvgl_buf_size_bytes);

    /* Create LVGL display */
    disp = lv_display_create(HOR_RES, VER_RES);
    if (!disp) {
        ESP_LOGE(TAG, "lv_display_create failed");
        return;
    }

    lv_display_set_flush_cb(disp, ili9488_flush);
    lv_display_set_draw_buffers(disp, &draw_buf1, &draw_buf2);
    lv_display_set_render_mode(disp, LV_DISPLAY_RENDER_MODE_PARTIAL);

    ESP_LOGI(TAG, "Initializing HX711 load cell...");
    hx711_init();
    // Initial tare with balanced samples for stability and speed
    hx711_do_tare(20);

    /* Create LVGL handler task */
    const int LVGL_TASK_STACK = 8192;
    BaseType_t r = xTaskCreatePinnedToCore(lvgl_task, "lvgl", LVGL_TASK_STACK, NULL, 5, NULL, 1);
    if (r != pdPASS) {
        ESP_LOGE(TAG, "Failed to start lvgl task");
        return;
    }

    update_ui_status("Initializing WiFi...");
    ESP_LOGI(TAG, "Initializing WiFi...");
    wifi_init_sta();

    update_ui_status("Initializing camera...");
    ESP_LOGI(TAG, "Initializing camera...");
    myCAM = createArducamCamera(CS_PIN);
    begin(&myCAM);

    update_ui_status("Starting web server...");
    ESP_LOGI(TAG, "Starting web server...");
    server = start_webserver();
    if (server) {
        ESP_LOGI(TAG, "Camera server started successfully!");
        update_ui_status("Ready - Visit /capture endpoint");
    } else {
        ESP_LOGE(TAG, "Failed to start web server!");
        update_ui_status("Web server failed!");
    }

    // Keep camera alive
    while (1) {
        captureThread(&myCAM);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
