#include "ArducamCamera.h"
#include "ArducamLink.h"
#include "delay.h"
#include "diagnostics.h"
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
#include "driver/i2c.h"
#include "priorities.h"
#include "spi_mutex.h"
#include "uart.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


// LVGL includes
#include "lvgl.h"
#include "driver/spi_master.h"

#define TAG "camera_server"

// ---- LVGL Display configuration ----
// Force recompilation after UI fixes
#define HOR_RES 480
#define VER_RES 320

// ---- Display pins ----
#define PIN_TFT_MOSI   23
#define PIN_TFT_SCLK   18
#define PIN_TFT_CS      5
#define PIN_TFT_DC     13
#define PIN_TFT_RST     4

// ---- Touch pins (FT6206 - I2C) ----
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_SDA_IO           21
#define I2C_MASTER_SCL_IO           22
#define I2C_MASTER_FREQ_HZ          400000
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0
#define I2C_MASTER_TIMEOUT_MS       1000

#define PIN_TOUCH_IRQ  27
#define PIN_TOUCH_RST  32
#define FT6206_I2C_ADDR    0x38
#define TOUCH_THRESHOLD    40

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

// ---- Touch globals ----
static volatile bool touch_interrupt_flag = false;

// ---- UI Status globals ----
static char ui_status_message[128] = "Initializing...";
static bool ui_status_update = false;

// ---- UI Navigation globals ----
static lv_obj_t *tabview = NULL;
static lv_obj_t *tab_capture = NULL;
static lv_obj_t *tab_results = NULL;
static lv_obj_t *loading_screen = NULL;
static lv_obj_t *spinner = NULL;

// ---- Meal data globals ----
static char meal_label[256] = "";
static float meal_weight = 0.0f;
static float meal_calories = 0.0f;
static float meal_protein = 0.0f;
static float meal_carbs = 0.0f;
static float meal_fat = 0.0f;
static char meal_status[32] = "";

// ---- Results page UI elements ----
static lv_obj_t *result_meal_name = NULL;
static lv_obj_t *result_weight = NULL;
static lv_obj_t *result_calories = NULL;
static lv_obj_t *result_protein = NULL;
static lv_obj_t *result_carbs = NULL;
static lv_obj_t *result_fat = NULL;

// ---- UI Status functions ----
static void update_ui_status(const char *message)
{
    strncpy(ui_status_message, message, sizeof(ui_status_message) - 1);
    ui_status_message[sizeof(ui_status_message) - 1] = '\0';
    ui_status_update = true;
}

// ---- Function prototypes ----
static void update_results_display(void);
static void show_loading_screen(void);
static void hide_loading_screen(void);

// ---- JSON Parsing functions ----
static const char* find_json_value(const char* json_str, const char* key) {
    char search_key[64];
    snprintf(search_key, sizeof(search_key), "\"%s\":", key);
    const char* pos = strstr(json_str, search_key);
    if (!pos) return NULL;

    pos += strlen(search_key);
    // Skip whitespace
    while (*pos == ' ' || *pos == '\t' || *pos == '\n' || *pos == '\r') pos++;

    return pos;
}

static void parse_meal_response(const char* json_response) {
    // Parse status
    const char* status_pos = find_json_value(json_response, "status");
    if (status_pos && *status_pos == '"') {
        status_pos++;
        const char* end = strchr(status_pos, '"');
        if (end) {
            size_t len = end - status_pos;
            if (len < sizeof(meal_status)) {
                memcpy(meal_status, status_pos, len);
                meal_status[len] = '\0';
            }
        }
    }

    // Parse entry object
    const char* entry_pos = find_json_value(json_response, "entry");
    if (entry_pos && *entry_pos == '{') {
        // Parse label
        const char* label_pos = find_json_value(entry_pos, "label");
        if (label_pos && *label_pos == '"') {
            label_pos++;
            const char* end = strchr(label_pos, '"');
            if (end) {
                size_t len = end - label_pos;
                if (len < sizeof(meal_label)) {
                    memcpy(meal_label, label_pos, len);
                    meal_label[len] = '\0';
                }
            }
        }

        // Parse numeric values
        const char* weight_pos = find_json_value(entry_pos, "weight_g");
        if (weight_pos) meal_weight = atof(weight_pos);

        const char* kcal_pos = find_json_value(entry_pos, "estimated_kcal");
        if (kcal_pos) meal_calories = atof(kcal_pos);

        const char* protein_pos = find_json_value(entry_pos, "protein_g");
        if (protein_pos) meal_protein = atof(protein_pos);

        const char* carbs_pos = find_json_value(entry_pos, "carbs_g");
        if (carbs_pos) meal_carbs = atof(carbs_pos);

        const char* fat_pos = find_json_value(entry_pos, "fat_g");
        if (fat_pos) meal_fat = atof(fat_pos);
    }
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
    // Acquire SPI mutex for thread-safe access
    spi_mutex_acquire(HSPI_HOST);

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

    // Release SPI mutex
    spi_mutex_release(HSPI_HOST);
}

static void send_data_chunked(const uint8_t *data, size_t len)
{
    if (!data || len == 0) return;

    // Acquire SPI mutex for thread-safe access (whole operation)
    spi_mutex_acquire(HSPI_HOST);

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

    // Release SPI mutex after all chunks sent
    spi_mutex_release(HSPI_HOST);
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

// ---- Touch functions ----

/* -------------------------------------------------------------------
 * I2C Master Initialization
 * -------------------------------------------------------------------*/
static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(I2C_MASTER_NUM, &conf);
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

/* -------------------------------------------------------------------
 * Touch Controller (FT6206) - I2C Functions
 * -------------------------------------------------------------------*/
#define FT6206_REG_NUMTOUCHES   0x02  // Number of touch points
#define FT6206_REG_THRESHHOLD   0x80  // Touch threshold register
#define FT6206_REG_P1_XH        0x03  // Point 1 X position high byte
#define FT6206_REG_P1_XL        0x04  // Point 1 X position low byte
#define FT6206_REG_P1_YH        0x05  // Point 1 Y position high byte
#define FT6206_REG_P1_YL        0x06  // Point 1 Y position low byte

static void touch_reset(void)
{
    // Configure RST pin as output
    gpio_config_t rst_config = {
        .pin_bit_mask = (1ULL << PIN_TOUCH_RST),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&rst_config);

    // Reset sequence: LOW → 10ms delay → HIGH → 50ms delay
    gpio_set_level(PIN_TOUCH_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(PIN_TOUCH_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(50));
}

static void IRAM_ATTR touch_isr_handler(void* arg)
{
    touch_interrupt_flag = true;
}

static void touch_interrupt_init(void)
{
    // Configure IRQ pin as input with pull-up
    gpio_config_t irq_config = {
        .pin_bit_mask = (1ULL << PIN_TOUCH_IRQ),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,  // FALLING edge
    };
    gpio_config(&irq_config);

    // Install ISR handler
    gpio_install_isr_service(0);
    gpio_isr_handler_add(PIN_TOUCH_IRQ, touch_isr_handler, NULL);
}

static esp_err_t ft6206_init(uint8_t threshold)
{
    uint8_t data[2];

    // Set touch threshold
    data[0] = FT6206_REG_THRESHHOLD;
    data[1] = threshold;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (FT6206_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, data, 2, true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    return ret;
}

static esp_err_t ft6206_read_touch(uint16_t* x, uint16_t* y, uint8_t* num_touches)
{
    uint8_t data[16];

    // Read touch data starting from register 0x00
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (FT6206_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x00, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (FT6206_I2C_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 16, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        return ret;
    }

    // Parse touch data
    *num_touches = data[FT6206_REG_NUMTOUCHES] & 0x0F;

    if (*num_touches > 0) {
        // Point 1 coordinates
        uint16_t raw_x = ((data[FT6206_REG_P1_XH] & 0x0F) << 8) | data[FT6206_REG_P1_XL];
        uint16_t raw_y = ((data[FT6206_REG_P1_YH] & 0x0F) << 8) | data[FT6206_REG_P1_YL];

        // Calibrated touch mapping for 480x320 display
        // These values may need calibration for your specific touchscreen
        #define TOUCH_X_MIN 10
        #define TOUCH_X_MAX 320
        #define TOUCH_Y_MIN 0
        #define TOUCH_Y_MAX 470

        // Map raw coordinates to screen coordinates
        int32_t mapped_x = ((int32_t)(TOUCH_Y_MAX - raw_y) * (HOR_RES - 1)) / TOUCH_Y_MAX;
        int32_t mapped_y = ((int32_t)(raw_x - TOUCH_X_MIN) * (VER_RES - 1)) / (TOUCH_X_MAX - TOUCH_X_MIN);

        // Clamp to screen bounds
        if (mapped_x < 0) mapped_x = 0;
        if (mapped_x >= HOR_RES) mapped_x = HOR_RES - 1;
        if (mapped_y < 0) mapped_y = 0;
        if (mapped_y >= VER_RES) mapped_y = VER_RES - 1;

        *x = (uint16_t)mapped_x;
        *y = (uint16_t)mapped_y;

        ESP_LOGI(TAG, "Touch: raw(%d,%d) → screen(%d,%d)", raw_x, raw_y, *x, *y);
    }

    return ESP_OK;
}

/* -------------------------------------------------------------------
 * LVGL Touch Input Callback
 * -------------------------------------------------------------------*/
static void touch_read_cb(lv_indev_t *indev, lv_indev_data_t *data)
{
    static int16_t last_x = 0;
    static int16_t last_y = 0;

    // Check for touch interrupt flag
    if (touch_interrupt_flag) {
        touch_interrupt_flag = false;

        // Read touch data
        uint16_t x, y;
        uint8_t num_touches;

        if (ft6206_read_touch(&x, &y, &num_touches) == ESP_OK && num_touches > 0) {
            // Clamp to screen bounds
            if (x >= HOR_RES) x = HOR_RES - 1;
            if (y >= VER_RES) y = VER_RES - 1;

            last_x = x;
            last_y = y;

            data->point.x = x;
            data->point.y = y;
            data->state = LV_INDEV_STATE_PRESSED;
        } else {
            data->point.x = last_x;
            data->point.y = last_y;
            data->state = LV_INDEV_STATE_RELEASED;
        }
    } else {
        data->point.x = last_x;
        data->point.y = last_y;
        data->state = LV_INDEV_STATE_RELEASED;
    }
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
    if (!hx711_wait_ready(1000)) return 0;

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
    response.buffer = heap_caps_malloc(response.max_len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
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
            // Parse the JSON response and store meal data
            parse_meal_response((const char*)response.buffer);

            // Update results display with parsed data
            update_results_display();

            // Switch to results tab FIRST (before hiding loading screen)
            if (tabview) {
                lv_tabview_set_active(tabview, 1, LV_ANIM_OFF); // Instant switch, no animation
            }

            // Hide loading screen to reveal results
            hide_loading_screen();

            update_ui_status("Meal analyzed successfully!");
        } else {
            // ---- ERROR CASE ----
            // Hide loading screen on error
            hide_loading_screen();

            char status_msg[64];
            snprintf(status_msg, sizeof(status_msg), "Upload error - HTTP %d", status);
            update_ui_status(status_msg);
        }

    } else {
        // ---- NETWORK ERROR ----
        // Hide loading screen on network error
        hide_loading_screen();

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
    config.task_priority = TASK_PRIORITY_NETWORK;  // Use standardized priority

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

static void trigger_capture(void)
{
    ESP_LOGI(TAG, "Capture triggered");

    // Read current weight from load cell
    double current_weight = hx711_get_weight_grams();
    ESP_LOGI(TAG, "Current weight: %.2f grams", current_weight);

    CamStatus status = takePicture(&myCAM, CAM_IMAGE_MODE_VGA, CAM_IMAGE_PIX_FMT_JPG);
    if (status != CAM_ERR_SUCCESS) {
        ESP_LOGE(TAG, "Failed to take picture, status: %d", status);
        update_ui_status("Camera capture failed");
        return;
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
        update_ui_status("Camera timeout");
        return;
    }

    if (image_length > 2000000) {
        ESP_LOGE(TAG, "Image too large: %u", (unsigned)image_length);
        update_ui_status("Image too large");
        return;
    }

    ESP_LOGI(TAG, "Image length: %u bytes", (unsigned)image_length);

    uint8_t *image_data = heap_caps_malloc(image_length, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!image_data) {
        ESP_LOGE(TAG, "Failed to allocate PSRAM for image!");
        update_ui_status("Memory allocation failed");
        return;
    }

    const uint32_t chunk_size = 200;
    uint8_t *tmp_buf = heap_caps_malloc(chunk_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!tmp_buf) {
        ESP_LOGE(TAG, "Failed to allocate tmp buffer!");
        heap_caps_free(image_data);
        update_ui_status("Memory allocation failed");
        return;
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
            update_ui_status("Camera read failed");
            return;
        }

        memcpy(image_data + bytes_read, tmp_buf, got);
        bytes_read += got;
    }

    heap_caps_free(tmp_buf);

    if (bytes_read != image_length) {
        ESP_LOGE(TAG, "Incomplete read: %u/%u", (unsigned)bytes_read, (unsigned)image_length);
        heap_caps_free(image_data);
        update_ui_status("Incomplete image read");
        return;
    }

    esp_err_t post_err = send_image_to_server(image_data, image_length, current_weight);
    if (post_err != ESP_OK) {
        ESP_LOGW(TAG, "send_image_to_server failed: %d", post_err);
    } else {
        ESP_LOGI(TAG, "Image uploaded to backend successfully");
        update_ui_status("Upload successful!");
    }

    heap_caps_free(image_data);
}

static void capture_btn_handler(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_CLICKED) {
        ESP_LOGI(TAG, "Capture button pressed - triggering capture");
        // Show loading screen
        show_loading_screen();
        // Force display update to show loading screen immediately
        lv_refr_now(NULL);
        // Small delay to ensure loading screen is visible
        vTaskDelay(pdMS_TO_TICKS(50));
        // Trigger the same capture functionality as the web endpoint
        trigger_capture();
    }
}

static void back_to_capture_handler(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_CLICKED) {
        ESP_LOGI(TAG, "Back to capture button pressed");
        // Switch back to capture tab
        if (tabview) {
            lv_tabview_set_active(tabview, 0, LV_ANIM_ON);
        }
    }
}

static void show_loading_screen(void)
{
    if (!loading_screen) {
        lv_obj_t *scr = lv_screen_active();

        // Create simple loading overlay
        loading_screen = lv_obj_create(scr);
        lv_obj_set_size(loading_screen, HOR_RES, VER_RES);
        lv_obj_align(loading_screen, LV_ALIGN_TOP_LEFT, 0, 0);
        lv_obj_set_style_bg_color(loading_screen, lv_color_hex(0x0A0E27), LV_PART_MAIN);
        lv_obj_set_style_bg_opa(loading_screen, LV_OPA_80, LV_PART_MAIN);
        lv_obj_clear_flag(loading_screen, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_clear_flag(loading_screen, LV_OBJ_FLAG_CLICKABLE); // Block interactions

        // Simple loading text
        lv_obj_t *loading_text = lv_label_create(loading_screen);
        lv_label_set_text(loading_text, "Analyzing...");
        lv_obj_set_style_text_color(loading_text, lv_color_hex(0xFFFFFF), 0);
        lv_obj_align(loading_text, LV_ALIGN_CENTER, 0, -20);

        // Simple spinner
        spinner = lv_spinner_create(loading_screen);
        lv_obj_set_size(spinner, 40, 40);
        lv_obj_align(spinner, LV_ALIGN_CENTER, 0, 20);
        lv_obj_set_style_arc_color(spinner, lv_color_hex(0x2196F3), LV_PART_INDICATOR);
    }

    lv_obj_clear_flag(loading_screen, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(loading_screen);
}

static void hide_loading_screen(void)
{
    if (loading_screen) {
        lv_obj_add_flag(loading_screen, LV_OBJ_FLAG_HIDDEN);
    }
}

static void update_results_display(void)
{
    if (result_meal_name) {
        lv_label_set_text(result_meal_name, meal_label[0] ? meal_label : "Unknown Food");
    }

    if (result_weight) {
        char weight_str[32];
        snprintf(weight_str, sizeof(weight_str), "%.1f g", meal_weight);
        lv_label_set_text(result_weight, weight_str);
    }

    if (result_calories) {
        char cal_str[32];
        snprintf(cal_str, sizeof(cal_str), "%.0f kcal", meal_calories);
        lv_label_set_text(result_calories, cal_str);
    }

    if (result_protein) {
        char protein_str[32];
        snprintf(protein_str, sizeof(protein_str), "%.1f g", meal_protein);
        lv_label_set_text(result_protein, protein_str);
    }

    if (result_carbs) {
        char carbs_str[32];
        snprintf(carbs_str, sizeof(carbs_str), "%.1f g", meal_carbs);
        lv_label_set_text(result_carbs, carbs_str);
    }

    if (result_fat) {
        char fat_str[32];
        snprintf(fat_str, sizeof(fat_str), "%.1f g", meal_fat);
        lv_label_set_text(result_fat, fat_str);
    }
}

static void create_camera_ui(void)
{
    lv_obj_t *scr = lv_screen_active();

    /* Professional dark theme background */
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x0A0E27), LV_PART_MAIN);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

    /* Create tabview - full screen height */
    tabview = lv_tabview_create(scr);
    lv_obj_set_size(tabview, HOR_RES, VER_RES);
    lv_obj_align(tabview, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_set_style_bg_color(tabview, lv_color_hex(0x0A0E27), LV_PART_MAIN);
    lv_obj_set_style_border_width(tabview, 0, LV_PART_MAIN);
    lv_tabview_set_tab_bar_size(tabview, 0); // Hide default tab bar

    /* Tab 1: Capture */
    tab_capture = lv_tabview_add_tab(tabview, "");

    /* Title for capture tab */
    lv_obj_t *capture_title = lv_label_create(tab_capture);
    lv_label_set_text(capture_title, "ESP32 Smart Scale");
    lv_obj_set_style_text_color(capture_title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(capture_title, LV_ALIGN_TOP_MID, 0, 15);

    /* Status card - positioned nicely */
    lv_obj_t *status_card = lv_obj_create(tab_capture);
    lv_obj_set_size(status_card, 420, 75);
    lv_obj_align(status_card, LV_ALIGN_TOP_MID, 0, 50);
    lv_obj_set_style_bg_color(status_card, lv_color_hex(0x1A1F3A), LV_PART_MAIN);
    lv_obj_set_style_border_width(status_card, 2, LV_PART_MAIN);
    lv_obj_set_style_border_color(status_card, lv_color_hex(0x667EEA), LV_PART_MAIN);
    lv_obj_set_style_radius(status_card, 10, LV_PART_MAIN);
    lv_obj_clear_flag(status_card, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *status_title = lv_label_create(status_card);
    lv_label_set_text(status_title, "Status");
    lv_obj_set_style_text_color(status_title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(status_title, LV_ALIGN_TOP_LEFT, 15, 8);

    status_label = lv_label_create(status_card);
    lv_label_set_text(status_label, "Initializing...");
    lv_obj_set_style_text_color(status_label, lv_color_hex(0x9CA3AF), 0);
    lv_obj_align(status_label, LV_ALIGN_TOP_LEFT, 15, 30);

    /* Weight card - positioned below status */
    lv_obj_t *weight_card = lv_obj_create(tab_capture);
    lv_obj_set_size(weight_card, 420, 75);
    lv_obj_align_to(weight_card, status_card, LV_ALIGN_OUT_BOTTOM_MID, 0, 15);
    lv_obj_set_style_bg_color(weight_card, lv_color_hex(0x1A1F3A), LV_PART_MAIN);
    lv_obj_set_style_border_width(weight_card, 2, LV_PART_MAIN);
    lv_obj_set_style_border_color(weight_card, lv_color_hex(0x4ECDC4), LV_PART_MAIN);
    lv_obj_set_style_radius(weight_card, 10, LV_PART_MAIN);
    lv_obj_clear_flag(weight_card, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *weight_title = lv_label_create(weight_card);
    lv_label_set_text(weight_title, "Current Weight");
    lv_obj_set_style_text_color(weight_title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(weight_title, LV_ALIGN_TOP_LEFT, 15, 8);

    weight_label = lv_label_create(weight_card);
    lv_label_set_text(weight_label, "-- g");
    lv_obj_set_style_text_color(weight_label, lv_color_hex(0x4ECDC4), 0);
    lv_obj_align(weight_label, LV_ALIGN_TOP_LEFT, 15, 30);

    /* Capture button - centered at bottom */
    capture_btn = lv_button_create(tab_capture);
    lv_obj_set_size(capture_btn, 200, 60);
    lv_obj_align(capture_btn, LV_ALIGN_BOTTOM_MID, 0, -30);
    lv_obj_set_style_bg_color(capture_btn, lv_color_hex(0x2196F3), LV_PART_MAIN);
    lv_obj_set_style_radius(capture_btn, 10, LV_PART_MAIN);
    lv_obj_add_event_cb(capture_btn, capture_btn_handler, LV_EVENT_CLICKED, NULL);

    lv_obj_t *btn_label = lv_label_create(capture_btn);
    lv_label_set_text(btn_label, "CAPTURE MEAL");
    lv_obj_set_style_text_color(btn_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_center(btn_label);

    /* Tab 2: Results */
    tab_results = lv_tabview_add_tab(tabview, "");

    /* Results page title */
    lv_obj_t *results_title = lv_label_create(tab_results);
    lv_label_set_text(results_title, "Meal Analysis Results");
    lv_obj_set_style_text_color(results_title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(results_title, LV_ALIGN_TOP_MID, 0, 15);

    /* Main results card - single card containing all info */
    lv_obj_t *results_card = lv_obj_create(tab_results);
    lv_obj_set_size(results_card, 440, 200);
    lv_obj_align(results_card, LV_ALIGN_TOP_MID, 0, 45);
    lv_obj_set_style_bg_color(results_card, lv_color_hex(0x1A1F3A), LV_PART_MAIN);
    lv_obj_set_style_border_width(results_card, 2, LV_PART_MAIN);
    lv_obj_set_style_border_color(results_card, lv_color_hex(0x667EEA), LV_PART_MAIN);
    lv_obj_set_style_radius(results_card, 10, LV_PART_MAIN);
    lv_obj_clear_flag(results_card, LV_OBJ_FLAG_SCROLLABLE);

    /* Food name */
    lv_obj_t *food_label = lv_label_create(results_card);
    lv_label_set_text(food_label, "Food:");
    lv_obj_set_style_text_color(food_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(food_label, LV_ALIGN_TOP_LEFT, 15, 10);

    result_meal_name = lv_label_create(results_card);
    lv_label_set_text(result_meal_name, "No data yet");
    lv_obj_set_style_text_color(result_meal_name, lv_color_hex(0xFF6B6B), 0);
    lv_obj_align(result_meal_name, LV_ALIGN_TOP_LEFT, 80, 10);

    /* Nutrition info in simple list format */
    const char* nutrient_names[] = {"Weight:", "Calories:", "Protein:", "Carbs:", "Fat:"};
    lv_color_t nutrient_colors[] = {
        lv_color_hex(0x4ECDC4), lv_color_hex(0xFFE66D),
        lv_color_hex(0xFF6B6B), lv_color_hex(0xA78BFA), lv_color_hex(0xF093FB)
    };

    lv_obj_t **result_labels[] = {&result_weight, &result_calories, &result_protein, &result_carbs, &result_fat};

    for (int i = 0; i < 5; i++) {
        /* Nutrient label */
        lv_obj_t *nutrient_label = lv_label_create(results_card);
        lv_label_set_text(nutrient_label, nutrient_names[i]);
        lv_obj_set_style_text_color(nutrient_label, lv_color_hex(0xFFFFFF), 0);
        lv_obj_align(nutrient_label, LV_ALIGN_TOP_LEFT, 15, 35 + i * 25);

        /* Nutrient value */
        *result_labels[i] = lv_label_create(results_card);
        lv_label_set_text(*result_labels[i], "--");
        lv_obj_set_style_text_color(*result_labels[i], nutrient_colors[i], 0);
        lv_obj_align(*result_labels[i], LV_ALIGN_TOP_LEFT, 100, 35 + i * 25);
    }

    /* Back to capture button - positioned below the results card */
    lv_obj_t *back_btn = lv_button_create(tab_results);
    lv_obj_set_size(back_btn, 200, 50);
    lv_obj_align_to(back_btn, results_card, LV_ALIGN_OUT_BOTTOM_MID, 0, 20);
    lv_obj_set_style_bg_color(back_btn, lv_color_hex(0x667EEA), LV_PART_MAIN);
    lv_obj_set_style_radius(back_btn, 10, LV_PART_MAIN);
    lv_obj_add_event_cb(back_btn, back_to_capture_handler, LV_EVENT_CLICKED, NULL);

    lv_obj_t *back_label = lv_label_create(back_btn);
    lv_label_set_text(back_label, "CAPTURE ANOTHER");
    lv_obj_set_style_text_color(back_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_center(back_label);

    ESP_LOGI(TAG, "Camera UI with proper alignment created");
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

/**
 * Diagnostic monitoring task - runs with TASK_PRIORITY_LOW (5)
 * Continuously monitors system health without interfering with critical tasks
 */
static void diagnostic_monitor_task(void *arg)
{
    ESP_LOGI(TAG, "Diagnostic monitoring task started (priority %d)", TASK_PRIORITY_LOW);

    while (1) {
        /* Light monitoring - check heap usage periodically */
        static uint32_t heap_check_counter = 0;

        if (heap_check_counter % 600 == 0) {  // Every 60 seconds (6 * 10s)
            uint32_t usage_percent = diagnostics_get_heap_usage_percentage();
            if (usage_percent > 75) {  // Only warn if usage is high
                ESP_LOGW(TAG, "High heap usage detected: %u%%", usage_percent);
            }
        }

        heap_check_counter++;
        vTaskDelay(pdMS_TO_TICKS(10000));  // Check every 10 seconds
    }
}

/**
 * Camera control task - runs with TASK_PRIORITY_HIGH (18)
 * Handles camera operations that require high priority and responsiveness
 */
static void camera_control_task(void *arg)
{
    ESP_LOGI(TAG, "Camera control task started (priority %d)", TASK_PRIORITY_HIGH);

    while (1) {
        /* Camera health monitoring and control */
        static uint32_t camera_check_counter = 0;

        if (camera_check_counter % 100 == 0) {  // Every 10 seconds
            /* Check camera connectivity and health */
            // Note: Camera operations are handled by the main loop and HTTP handlers
            // This task is reserved for future camera control features
            ESP_LOGD(TAG, "Camera control task active");
        }

        camera_check_counter++;
        vTaskDelay(pdMS_TO_TICKS(100));  // Check every 100ms
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

    // [HEAP SNAPSHOT] After NVS init
    ESP_LOGI(TAG, "[INIT] NVS Flash initialized");
    diagnostics_print_heap_stats();

    uartBegin(115200);
    ESP_LOGI(TAG, "ESP32 Camera Server starting...");

    // Initialize diagnostics module
    diagnostics_init();
    ESP_LOGI(TAG, "[INIT] Diagnostics module initialized");

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
        .clock_speed_hz = 60 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = PIN_TFT_CS,
        .queue_size = 2,                          // DMA-safe: small queue for efficiency
        .flags = SPI_DEVICE_NO_DUMMY              // DMA-safe: no dummy bits
    };
    spi_bus_add_device(HSPI_HOST, &devcfg, &tft_spi);

    // Initialize SPI mutex for TFT display
    ESP_ERROR_CHECK(spi_mutex_init(HSPI_HOST));

    ili9488_init();

    // [HEAP SNAPSHOT] After display init
    ESP_LOGI(TAG, "[INIT] Display (ILI9488) initialized");
    diagnostics_print_heap_stats();

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
    rgb666_buf = heap_caps_malloc(rgb666_buf_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!rgb666_buf) {
        rgb666_buf = heap_caps_malloc(rgb666_buf_size, MALLOC_CAP_DEFAULT | MALLOC_CAP_8BIT);
        if (!rgb666_buf) {
            ESP_LOGE(TAG, "Failed to allocate rgb666_buf");
            return;
        }
        ESP_LOGW(TAG, "rgb666_buf allocated from regular heap");
    }

    /* Allocate LVGL full-screen buffers from PSRAM */
    size_t lvgl_buf_size_bytes = HOR_RES * VER_RES * sizeof(lv_color_t);
    lv_buf1 = heap_caps_malloc(lvgl_buf_size_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    lv_buf2 = heap_caps_malloc(lvgl_buf_size_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

    if (!lv_buf1 || !lv_buf2) {
        ESP_LOGW(TAG, "PSRAM allocation failed, trying regular heap");
        if (lv_buf1) free(lv_buf1);
        if (lv_buf2) free(lv_buf2);
        lv_buf1 = heap_caps_malloc(lvgl_buf_size_bytes, MALLOC_CAP_DEFAULT | MALLOC_CAP_8BIT);
        lv_buf2 = heap_caps_malloc(lvgl_buf_size_bytes, MALLOC_CAP_DEFAULT | MALLOC_CAP_8BIT);
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

    // [HEAP SNAPSHOT] After LVGL init + buffers
    ESP_LOGI(TAG, "[INIT] LVGL initialized with display buffers");
    diagnostics_print_heap_stats();

    /* Initialize touch controller */
    ESP_LOGI(TAG, "Initializing I2C for touch controller...");
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C master initialized for touch controller");

    touch_reset();
    touch_interrupt_init();
    ESP_ERROR_CHECK(ft6206_init(TOUCH_THRESHOLD));
    ESP_LOGI(TAG, "FT6206 touch controller initialized");

    /* Register touch input device with LVGL */
    lv_indev_t *indev = lv_indev_create();
    if (!indev) {
        ESP_LOGE(TAG, "lv_indev_create failed");
        return;
    }
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(indev, touch_read_cb);
    ESP_LOGI(TAG, "Touch input device registered");

    // [HEAP SNAPSHOT] After touch controller init
    ESP_LOGI(TAG, "[INIT] Touch controller (FT6206) initialized");
    diagnostics_print_heap_stats();

    ESP_LOGI(TAG, "Initializing HX711 load cell...");
    hx711_init();
    // Initial tare with balanced samples for stability and speed
    hx711_do_tare(20);

    // [HEAP SNAPSHOT] After HX711 init
    ESP_LOGI(TAG, "[INIT] HX711 load cell initialized");
    diagnostics_print_heap_stats();

    /* Create LVGL handler task */
    const int LVGL_TASK_STACK = 8192;
    BaseType_t r = xTaskCreatePinnedToCore(lvgl_task, "lvgl", LVGL_TASK_STACK, NULL, TASK_PRIORITY_UI, NULL, 1);
    if (r != pdPASS) {
        ESP_LOGE(TAG, "Failed to start lvgl task");
        return;
    }

    // [HEAP SNAPSHOT] After LVGL task creation
    ESP_LOGI(TAG, "[INIT] LVGL UI task created");
    diagnostics_print_heap_stats();

    /* Create diagnostic monitoring task with standardized priority */
    const int DIAGNOSTIC_TASK_STACK = 2048;
    BaseType_t diag_r = xTaskCreate(diagnostic_monitor_task, "diag_mon",
                                   DIAGNOSTIC_TASK_STACK, NULL,
                                   TASK_PRIORITY_LOW, NULL);
    if (diag_r != pdPASS) {
        ESP_LOGE(TAG, "Failed to start diagnostic monitoring task");
    } else {
        ESP_LOGI(TAG, "[INIT] Diagnostic monitoring task created (priority %d)",
                 TASK_PRIORITY_LOW);
    }

    /* Create camera control task with high priority for responsive capture */
    const int CAMERA_TASK_STACK = 4096;
    BaseType_t cam_r = xTaskCreate(camera_control_task, "cam_ctrl",
                                  CAMERA_TASK_STACK, NULL,
                                  TASK_PRIORITY_HIGH, NULL);
    if (cam_r != pdPASS) {
        ESP_LOGE(TAG, "Failed to start camera control task");
    } else {
        ESP_LOGI(TAG, "[INIT] Camera control task created (priority %d)",
                 TASK_PRIORITY_HIGH);
    }

    update_ui_status("Initializing WiFi...");
    ESP_LOGI(TAG, "Initializing WiFi...");
    wifi_init_sta();

    // [HEAP SNAPSHOT] After WiFi init
    ESP_LOGI(TAG, "[INIT] WiFi STA initialized");
    diagnostics_print_heap_stats();

    update_ui_status("Initializing camera...");
    ESP_LOGI(TAG, "Initializing camera...");
    myCAM = createArducamCamera(CS_PIN);
    begin(&myCAM);

    // [HEAP SNAPSHOT] After camera init
    ESP_LOGI(TAG, "[INIT] Camera (Arducam) initialized");
    diagnostics_print_heap_stats();

    update_ui_status("Starting web server...");
    ESP_LOGI(TAG, "Starting web server...");
    server = start_webserver();
    if (server) {
        ESP_LOGI(TAG, "Camera server started successfully!");
        update_ui_status("Ready - Visit /capture endpoint");

        // [HEAP SNAPSHOT] After full system init
        ESP_LOGI(TAG, "[INIT] Web server started - System fully initialized!");
        diagnostics_print_heap_stats();
    } else {
        ESP_LOGE(TAG, "Failed to start web server!");
        update_ui_status("Web server failed!");

        // [HEAP SNAPSHOT] After failed web server init
        ESP_LOGI(TAG, "[INIT] Web server failed - System partially initialized");
        diagnostics_print_heap_stats();
    }

    // Keep camera alive and run periodic diagnostics
    uint32_t diagnostic_counter = 0;
    const uint32_t DIAGNOSTICS_INTERVAL_MS = 30000; // 30 seconds
    const uint32_t DIAGNOSTICS_TICKS = DIAGNOSTICS_INTERVAL_MS / portTICK_PERIOD_MS;

    while (1) {
        captureThread(&myCAM);

        // Run diagnostics every 30 seconds
        if (diagnostic_counter >= DIAGNOSTICS_TICKS) {
            ESP_LOGI(TAG, "Running periodic diagnostics...");
            diagnostics_print_heap_stats();
            diagnostics_print_task_stats();
            diagnostic_counter = 0;
        }

        diagnostic_counter++;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
