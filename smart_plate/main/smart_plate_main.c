// Smart Plate - Meal Logging System
// Combines: ILI9488 Display + FT6206 Touch + HX711 Load Cell + Arducam Camera

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/i2c.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_rom_sys.h"
#include <string.h>
#include <stdio.h>
#include "lvgl.h"

// Camera and network includes
#include "ArducamCamera.h"
#include "ArducamLink.h"
#include "delay.h"
#include "esp_event.h"
#include "esp_http_client.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/event_groups.h"
#include "nvs_flash.h"

static const char *TAG = "SmartPlate";

/* Display resolution */
#define HOR_RES 480
#define VER_RES 320

/* Display pins */
#define PIN_TFT_MOSI   23
#define PIN_TFT_SCLK   18
#define PIN_TFT_CS      5
#define PIN_TFT_DC     13
#define PIN_TFT_RST     4
#define PIN_TFT_MISO   -1

/* Touch pins (FT6206 - I2C) */
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_SDA_IO           21
#define I2C_MASTER_SCL_IO           22
#define I2C_MASTER_FREQ_HZ          400000
#define PIN_TOUCH_IRQ  27
#define PIN_TOUCH_RST  32
#define FT6206_I2C_ADDR    0x38
#define TOUCH_THRESHOLD    40

/* HX711 pins */
#define HX711_DOUT 34
#define HX711_SCK  14

/* Camera pins */
#define CAMERA_CS  33

/* WiFi configuration */
#define WIFI_SSID "Testpress_4G"
#define WIFI_PASS "Tp@12345"
#define MAXIMUM_RETRY 5

/* API upload configuration */
static const char *UPLOAD_URL = "http://192.168.0.4:8000/api/v1/meal_logs/";
static const char *AUTH_HEADER_VALUE = "Bearer iKnCz7E2Mtfl_V0bDcasJWDMzVN39L_BCySvj1hLDSc";

/* Transfer sizing */
#define MAX_SPI_TRANS_BYTES  (8 * 1024)
#define LINES                25  // Reduced to fit in fragmented memory (need ~108KB total)
#define BUF_PIXELS           (HOR_RES * LINES)
#define RGB666_BUF_SIZE      (BUF_PIXELS * 3)

/* SPI Pin Assignments (to avoid conflicts):
 * Display (ILI9488) - HSPI_HOST: GPIO 23 (MOSI), GPIO 18 (SCLK), GPIO 5 (CS)
 * Camera (Arducam) - VSPI_HOST: GPIO 25 (MOSI), GPIO 26 (SCLK), GPIO 19 (MISO), GPIO 33 (CS)
 */

// Global variables
static spi_device_handle_t tft_spi;
static lv_color_t *lv_buf1 = NULL;
static lv_color_t *lv_buf2 = NULL;
static uint8_t *rgb666_buf = NULL;
static volatile bool touch_interrupt_flag = false;

// WiFi and camera globals
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
static int s_retry_num = 0;
ArducamCamera myCAM;

// HX711 state
static long hx711_offset = 0;
static double hx711_scale = -0.001307;

// Meal log data
typedef struct {
    uint8_t *image_data;
    size_t image_size;
    float weight;
    char food_name[32];
    int calories;
    float protein;
    float carbs;
    float fat;
} meal_log_t;

static meal_log_t current_meal;

// UI objects
static lv_obj_t *screen_home = NULL;
static lv_obj_t *screen_weighing = NULL;
static lv_obj_t *screen_result = NULL;
static lv_obj_t *weight_label = NULL;
static lv_obj_t *result_image_canvas = NULL;
static lv_obj_t *result_weight_label = NULL;
static lv_obj_t *result_food_label = NULL;
static lv_obj_t *result_nutrition_label = NULL;

// Canvas buffer for captured image display (100x75 pixels - reduced for memory)
#define CANVAS_WIDTH  100
#define CANVAS_HEIGHT 75
static lv_color_t *canvas_buffer = NULL;

/* -------------------------------------------------------------------
 * HX711 Functions
 * -------------------------------------------------------------------*/
static void hx711_init(void) {
    gpio_reset_pin(HX711_DOUT);
    gpio_set_direction(HX711_DOUT, GPIO_MODE_INPUT);
    gpio_reset_pin(HX711_SCK);
    gpio_set_direction(HX711_SCK, GPIO_MODE_OUTPUT);
    gpio_set_level(HX711_SCK, 0);
}

static bool hx711_wait_ready(int timeout_ms) {
    int64_t start = esp_timer_get_time();
    while (gpio_get_level(HX711_DOUT) == 1) {
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
        gpio_set_level(HX711_SCK, 1);
        esp_rom_delay_us(1);
        value = (value << 1) | gpio_get_level(HX711_DOUT);
        gpio_set_level(HX711_SCK, 0);
        esp_rom_delay_us(1);
    }

    gpio_set_level(HX711_SCK, 1);
    esp_rom_delay_us(1);
    gpio_set_level(HX711_SCK, 0);
    esp_rom_delay_us(1);

    if (value & 0x800000) value |= 0xFF000000;
    return (int32_t)value;
}

static long hx711_read_average(uint16_t samples) {
    long sum = 0;
    for (uint16_t i = 0; i < samples; ++i) {
        sum += hx711_read_raw();
    }
    return sum / (long)samples;
}

static void hx711_tare(uint16_t samples) {
    hx711_offset = hx711_read_average(samples);
    ESP_LOGI(TAG, "[TARE] OFFSET set to %ld (avg of %u samples)", hx711_offset, samples);
}

static float hx711_get_weight(void) {
    long raw = hx711_read_average(5);
    return (float)((raw - hx711_offset) * hx711_scale);
}

/* -------------------------------------------------------------------
 * WiFi Functions
 * -------------------------------------------------------------------*/
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

    // Register events
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

    // Configure and start WiFi
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

/* -------------------------------------------------------------------
 * Low-level SPI helpers (with chunking)
 * -------------------------------------------------------------------*/
static void send_cmd(uint8_t cmd) {
    gpio_set_level(PIN_TFT_DC, 0);
    spi_transaction_t t = {0};
    t.length = 8;
    t.tx_buffer = &cmd;
    spi_device_polling_transmit(tft_spi, &t);
}

static void send_data_chunked(const uint8_t *data, size_t len) {
    if (!data || len == 0) return;
    gpio_set_level(PIN_TFT_DC, 1);
    size_t sent = 0;
    while (sent < len) {
        size_t to_send = len - sent;
        if (to_send > MAX_SPI_TRANS_BYTES) to_send = MAX_SPI_TRANS_BYTES;
        spi_transaction_t t = {0};
        t.length = to_send * 8;
        t.tx_buffer = data + sent;
        spi_device_polling_transmit(tft_spi, &t);
        sent += to_send;
    }
}

/* -------------------------------------------------------------------
 * ILI9488 init
 * -------------------------------------------------------------------*/
static void ili9488_init(void) {
    ESP_LOGI(TAG, "LCD Reset");
    gpio_set_level(PIN_TFT_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(PIN_TFT_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(120));

    send_cmd(0x11); // Sleep Out
    vTaskDelay(pdMS_TO_TICKS(120));

    send_cmd(0x36); // MADCTL
    uint8_t madctl = 0xE8;
    send_data_chunked(&madctl, 1);

    send_cmd(0x3A); // Pixel format
    uint8_t pix = 0x66; // RGB666
    send_data_chunked(&pix, 1);

    send_cmd(0x29); // Display ON
    vTaskDelay(pdMS_TO_TICKS(10));
}

/* -------------------------------------------------------------------
 * LVGL flush
 * -------------------------------------------------------------------*/
static void ili9488_flush(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map) {
    int w = area->x2 - area->x1 + 1;
    int h = area->y2 - area->y1 + 1;
    int total = w * h;
    
    if (total <= 0) {
        lv_display_flush_ready(disp);
        return;
    }

    uint16_t *src = (uint16_t *)px_map;
    for (int i = 0; i < total; ++i) {
        uint16_t c = src[i];
        uint8_t r5 = (c >> 11) & 0x1F;
        uint8_t g6 = (c >> 5)  & 0x3F;
        uint8_t b5 = (c)       & 0x1F;
        rgb666_buf[3*i + 0] = (uint8_t)(r5 << 3);
        rgb666_buf[3*i + 1] = (uint8_t)(g6 << 2);
        rgb666_buf[3*i + 2] = (uint8_t)(b5 << 3);
    }

    uint8_t col[4] = {
        (uint8_t)((area->x1 >> 8) & 0xFF), (uint8_t)(area->x1 & 0xFF),
        (uint8_t)((area->x2 >> 8) & 0xFF), (uint8_t)(area->x2 & 0xFF)
    };
    send_cmd(0x2A);
    send_data_chunked(col, 4);

    uint8_t row[4] = {
        (uint8_t)((area->y1 >> 8) & 0xFF), (uint8_t)(area->y1 & 0xFF),
        (uint8_t)((area->y2 >> 8) & 0xFF), (uint8_t)(area->y2 & 0xFF)
    };
    send_cmd(0x2B);
    send_data_chunked(row, 4);

    send_cmd(0x2C);
    send_data_chunked(rgb666_buf, total * 3);

    lv_display_flush_ready(disp);
}

/* -------------------------------------------------------------------
 * I2C Master Initialization
 * -------------------------------------------------------------------*/
static esp_err_t i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(I2C_MASTER_NUM, &conf);
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

/* -------------------------------------------------------------------
 * Touch Controller (FT6206)
 * -------------------------------------------------------------------*/
#define FT6206_REG_NUMTOUCHES   0x02
#define FT6206_REG_THRESHHOLD   0x80
#define FT6206_REG_P1_XH        0x03
#define FT6206_REG_P1_XL        0x04
#define FT6206_REG_P1_YH        0x05
#define FT6206_REG_P1_YL        0x06

static void touch_reset(void) {
    gpio_config_t rst_config = {
        .pin_bit_mask = (1ULL << PIN_TOUCH_RST),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&rst_config);

    gpio_set_level(PIN_TOUCH_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(PIN_TOUCH_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(50));
}

static void IRAM_ATTR touch_isr_handler(void* arg) {
    touch_interrupt_flag = true;
}

static void touch_interrupt_init(void) {
    gpio_config_t irq_config = {
        .pin_bit_mask = (1ULL << PIN_TOUCH_IRQ),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&irq_config);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(PIN_TOUCH_IRQ, touch_isr_handler, NULL);
}

static esp_err_t ft6206_init(uint8_t threshold) {
    uint8_t data[2];
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

static esp_err_t ft6206_read_touch(uint16_t* x, uint16_t* y, uint8_t* num_touches) {
    uint8_t data[16];

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

    *num_touches = data[FT6206_REG_NUMTOUCHES] & 0x0F;

    if (*num_touches > 0) {
        uint16_t raw_x = ((data[FT6206_REG_P1_XH] & 0x0F) << 8) | data[FT6206_REG_P1_XL];
        uint16_t raw_y = ((data[FT6206_REG_P1_YH] & 0x0F) << 8) | data[FT6206_REG_P1_YL];

        #define TOUCH_X_MIN 10
        #define TOUCH_X_MAX 322
        #define TOUCH_Y_MIN 0
        #define TOUCH_Y_MAX 470
        
        int32_t mapped_x = ((int32_t)(TOUCH_Y_MAX - raw_y) * (HOR_RES - 1)) / TOUCH_Y_MAX;
        int32_t mapped_y = ((int32_t)(raw_x - TOUCH_X_MIN) * (VER_RES - 1)) / (TOUCH_X_MAX - TOUCH_X_MIN);
        
        if (mapped_x < 0) mapped_x = 0;
        if (mapped_x >= HOR_RES) mapped_x = HOR_RES - 1;
        if (mapped_y < 0) mapped_y = 0;
        if (mapped_y >= VER_RES) mapped_y = VER_RES - 1;
        
        *x = (uint16_t)mapped_x;
        *y = (uint16_t)mapped_y;
    }

    return ESP_OK;
}

/* -------------------------------------------------------------------
 * LVGL Touch Input Callback
 * -------------------------------------------------------------------*/
static void touch_read_cb(lv_indev_t *indev, lv_indev_data_t *data) {
    static int16_t last_x = 0;
    static int16_t last_y = 0;

    if (touch_interrupt_flag) {
        touch_interrupt_flag = false;

        uint16_t x, y;
        uint8_t num_touches;

        if (ft6206_read_touch(&x, &y, &num_touches) == ESP_OK && num_touches > 0) {
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

/* -------------------------------------------------------------------
 * LVGL tick
 * -------------------------------------------------------------------*/
static void tick_inc(void *arg) {
    lv_tick_inc(1);
}

/* -------------------------------------------------------------------
 * Camera Simulation (replace with actual ArduCAM code)
 * -------------------------------------------------------------------*/
static void draw_simulated_food_image(float weight) {
    if (!result_image_canvas || !canvas_buffer) return;
    
    // Determine food color based on weight
    lv_color_t food_color;
    lv_color_t bg_color = lv_color_hex(0x2A2A3A);
    
    if (weight < 50) {
        food_color = lv_color_hex(0xFF6B6B); // Red for cherry
    } else if (weight < 150) {
        food_color = lv_color_hex(0x4ECDC4); // Cyan for apple
    } else if (weight < 300) {
        food_color = lv_color_hex(0xFFE66D); // Yellow for banana
    } else {
        food_color = lv_color_hex(0xFFA07A); // Orange for orange
    }
    
    // Draw directly to canvas buffer
    int center_x = CANVAS_WIDTH / 2;
    int center_y = CANVAS_HEIGHT / 2;
    int radius = 35;
    
    // Fill background and draw circular food item
    for (int y = 0; y < CANVAS_HEIGHT; y++) {
        for (int x = 0; x < CANVAS_WIDTH; x++) {
            int dx = x - center_x;
            int dy = y - center_y;
            int dist_sq = dx * dx + dy * dy;
            int radius_sq = radius * radius;
            
            // Draw circular food item
            if (dist_sq <= radius_sq) {
                // Add highlight effect in upper-left quadrant
                if (dx < -8 && dy < -8 && dist_sq <= (radius / 3) * (radius / 3)) {
                    // Bright highlight
                    lv_color_t highlight = lv_color_white();
                    canvas_buffer[y * CANVAS_WIDTH + x] = lv_color_mix(highlight, food_color, 180);
                } else if (dx < 0 && dy < 0) {
                    // Lighter shade on top-left
                    lv_color_t highlight = lv_color_white();
                    canvas_buffer[y * CANVAS_WIDTH + x] = lv_color_mix(highlight, food_color, 50);
                } else if (dx > 0 && dy > 0) {
                    // Darker shade on bottom-right
                    lv_color_t shadow = lv_color_black();
                    canvas_buffer[y * CANVAS_WIDTH + x] = lv_color_mix(shadow, food_color, 30);
                } else {
                    // Main color
                    canvas_buffer[y * CANVAS_WIDTH + x] = food_color;
                }
            } else {
                // Background
                canvas_buffer[y * CANVAS_WIDTH + x] = bg_color;
            }
        }
    }
    
    // Force LVGL to redraw the canvas
    lv_obj_invalidate(result_image_canvas);
    
    ESP_LOGI(TAG, "Food image rendered on canvas (weight: %.1fg)", weight);
}

// Simplified image capture that gets image size only (no storage)
static bool capture_image_simple(size_t *image_size) {
    ESP_LOGI(TAG, "Starting camera capture");

    // Clear any previous image length
    myCAM.image_length = 0;

    // Trigger capture (QQVGA JPEG for much smaller file size to save memory)
    CamStatus status = takePicture(&myCAM, CAM_IMAGE_MODE_QQVGA, CAM_IMAGE_PIX_FMT_JPG);
    if (status != CAM_ERR_SUCCESS) {
        ESP_LOGE(TAG, "Failed to take picture, status: %d", status);
        return false;
    }

    // Check if image is available (length should be set by takePicture)
    uint32_t length = imageAvailable(&myCAM);
    if (length == 0) {
        ESP_LOGE(TAG, "No image data available after capture");
        return false;
    }

    if (length > 100000) {  // Allow up to 100KB for QQVGA images
        ESP_LOGE(TAG, "Image too large: %u", (unsigned)length);
        return false;
    }

    ESP_LOGI(TAG, "Image captured successfully: %u bytes", (unsigned)length);
    *image_size = length;

    return true;
}

// Legacy function for compatibility (now simplified)
static bool capture_image(uint8_t **image_data, size_t *image_size) {
    ESP_LOGI(TAG, "Starting camera capture");

    // Clear any previous image length
    myCAM.image_length = 0;

    // Trigger capture (QQVGA JPEG for much smaller file size to save memory)
    CamStatus status = takePicture(&myCAM, CAM_IMAGE_MODE_QQVGA, CAM_IMAGE_PIX_FMT_JPG);
    if (status != CAM_ERR_SUCCESS) {
        ESP_LOGE(TAG, "Failed to take picture, status: %d", status);
        return false;
    }

    // Check if image is available (length should be set by takePicture)
    uint32_t length = imageAvailable(&myCAM);
    if (length == 0) {
        ESP_LOGE(TAG, "No image data available after capture");
        return false;
    }

    if (length > 100000) {  // Allow up to 100KB for QQVGA images
        ESP_LOGE(TAG, "Image too large: %u", (unsigned)length);
        return false;
    }

    ESP_LOGI(TAG, "Image length: %u bytes", (unsigned)length);
    *image_size = length;

    // Allocate PSRAM buffer for image
    *image_data = heap_caps_malloc(length, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!*image_data) {
        ESP_LOGE(TAG, "Failed to allocate PSRAM for image!");
        return false;
    }

    // Read image in small chunks (<=255) because many readBuff implementations use uint8_t length
    const uint32_t chunk_size = 200; // MUST be <= 255
    uint8_t *tmp_buf = heap_caps_malloc(chunk_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!tmp_buf) {
        ESP_LOGE(TAG, "Failed to allocate tmp buffer!");
        heap_caps_free(*image_data);
        *image_data = NULL;
        return false;
    }

    uint32_t bytes_read = 0;
    while (bytes_read < length) {
        uint32_t remaining = length - bytes_read;
        uint32_t to_read = (remaining > chunk_size) ? chunk_size : remaining;

        // readBuff typically returns uint8_t (0..255)
        uint8_t got = 0;
        int retry = 0;
        const int max_read_retries = 5;

        while (retry < max_read_retries) {
            // Ensure camera FSM runs in between attempts
            captureThread(&myCAM);
            got = readBuff(&myCAM, tmp_buf, (uint8_t)to_read);
            if (got > 0) break;
            // if got == 0, wait a bit and retry
            vTaskDelay(pdMS_TO_TICKS(10 + retry*5));
            retry++;
        }

        ESP_LOGD(TAG, "Chunk read attempt: offset=%u to_read=%u got=%u retries=%d",
                 (unsigned)bytes_read, (unsigned)to_read, (unsigned)got, retry);

        if (got == 0) {
            ESP_LOGE(TAG, "Failed to read chunk after retries (read %u/%u bytes)",
                     (unsigned)bytes_read, (unsigned)length);
            heap_caps_free(tmp_buf);
            heap_caps_free(*image_data);
            *image_data = NULL;
            return false;
        }

        memcpy(*image_data + bytes_read, tmp_buf, got);
        bytes_read += got;
    }

    heap_caps_free(tmp_buf);

    if (bytes_read != length) {
        ESP_LOGE(TAG, "Incomplete read: %u/%u", (unsigned)bytes_read, (unsigned)length);
        heap_caps_free(*image_data);
        *image_data = NULL;
        return false;
    }

    ESP_LOGI(TAG, "Image captured successfully: %u bytes", (unsigned)length);

    // For simplicity, we'll send image directly to API without local storage
    // Free the allocated buffer since we won't use it
    if (*image_data) {
        heap_caps_free(*image_data);
        *image_data = NULL;
    }

    return true;
}

/* -------------------------------------------------------------------
 * AI Food Recognition Simulation
 * -------------------------------------------------------------------*/
static void recognize_food(float weight, meal_log_t *meal) {
    // Simulate AI recognition based on weight
    if (weight < 50) {
        strcpy(meal->food_name, "Cherry");
        meal->calories = 5;
        meal->protein = 0.1f;
        meal->carbs = 1.2f;
        meal->fat = 0.0f;
    } else if (weight < 150) {
        strcpy(meal->food_name, "Apple");
        meal->calories = 95;
        meal->protein = 0.5f;
        meal->carbs = 25.0f;
        meal->fat = 0.3f;
    } else if (weight < 300) {
        strcpy(meal->food_name, "Banana");
        meal->calories = 105;
        meal->protein = 1.3f;
        meal->carbs = 27.0f;
        meal->fat = 0.4f;
    } else {
        strcpy(meal->food_name, "Orange");
        meal->calories = 62;
        meal->protein = 1.2f;
        meal->carbs = 15.4f;
        meal->fat = 0.2f;
    }

    ESP_LOGI(TAG, "Food recognized: %s (%.1fg)", meal->food_name, weight);
}

// Forward declaration for multipart upload function
static esp_err_t send_image_to_server(const uint8_t *jpeg, size_t jpeg_len, float weight);

// Upload function that reads image into memory first, then sends (reliable approach)
static esp_err_t send_image_to_server_buffered(size_t image_size, float weight) {
    ESP_LOGI(TAG, "Starting buffered image upload: %zu bytes", image_size);

    // Wait for camera to be ready for reading (like in working camera project)
    const TickType_t wait_tick = pdMS_TO_TICKS(50);
    const int max_wait_cycles = 40; // 40 * 50ms = 2s total wait
    int wait_count = 0;
    uint32_t available_length = 0;

    ESP_LOGI(TAG, "Waiting for camera to be ready...");
    while (wait_count < max_wait_cycles) {
        captureThread(&myCAM); // drive camera state machine
        vTaskDelay(wait_tick);
        available_length = imageAvailable(&myCAM);
        if (available_length > 0) break;
        wait_count++;
    }

    if (available_length == 0) {
        ESP_LOGE(TAG, "Camera not ready for reading after timeout");
        return ESP_FAIL;
    }

    if (available_length != image_size) {
        ESP_LOGW(TAG, "Available length (%u) != expected size (%zu), using available length",
                 (unsigned)available_length, image_size);
        image_size = available_length;
    }

    // Allocate buffer for complete image (try PSRAM first, then regular heap)
    uint8_t *image_data = heap_caps_malloc(image_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!image_data) {
        ESP_LOGW(TAG, "PSRAM allocation failed for image buffer (%zu bytes), trying regular heap", image_size);
        image_data = heap_caps_malloc(image_size, MALLOC_CAP_8BIT);
        if (!image_data) {
            ESP_LOGE(TAG, "Failed to allocate regular heap for image buffer (%zu bytes)", image_size);
            return ESP_ERR_NO_MEM;
        }
        ESP_LOGI(TAG, "Allocated image buffer in regular heap: %zu bytes", image_size);
    } else {
        ESP_LOGI(TAG, "Allocated image buffer in PSRAM: %zu bytes", image_size);
    }

    // Read image in chunks (like in camera project)
    const uint32_t chunk_size = 1024; // 1KB chunks
    uint8_t *tmp_buf = heap_caps_malloc(chunk_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!tmp_buf) {
        ESP_LOGW(TAG, "PSRAM allocation failed for temp buffer, trying regular heap");
        tmp_buf = heap_caps_malloc(chunk_size, MALLOC_CAP_8BIT);
        if (!tmp_buf) {
            ESP_LOGE(TAG, "Failed to allocate temporary buffer");
            heap_caps_free(image_data);
            return ESP_ERR_NO_MEM;
        }
    }

    uint32_t bytes_read = 0;
    while (bytes_read < image_size) {
        uint32_t remaining = image_size - bytes_read;
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

        ESP_LOGD(TAG, "Read chunk: offset=%u to_read=%u got=%u retries=%d",
                 (unsigned)bytes_read, (unsigned)to_read, (unsigned)got, retry);

        if (got == 0) {
            ESP_LOGE(TAG, "Failed to read chunk after retries (read %u/%u bytes)",
                     (unsigned)bytes_read, (unsigned)image_size);
            heap_caps_free(tmp_buf);
            heap_caps_free(image_data);
            return ESP_FAIL;
        }

        memcpy(image_data + bytes_read, tmp_buf, got);
        bytes_read += got;
    }

    heap_caps_free(tmp_buf);

    if (bytes_read != image_size) {
        ESP_LOGE(TAG, "Incomplete read: %u/%u bytes", (unsigned)bytes_read, (unsigned)image_size);
        heap_caps_free(image_data);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Image read into buffer successfully: %u bytes", (unsigned)image_size);

    // Now send the buffered image using the proven multipart approach
    esp_err_t result = send_image_to_server(image_data, image_size, weight);

    // Clean up
    heap_caps_free(image_data);

    return result;
}

/* -------------------------------------------------------------------
 * HTTP Upload Functions
 * -------------------------------------------------------------------*/
static esp_err_t send_image_to_server(const uint8_t *jpeg, size_t jpeg_len, float weight)
{
    const char *url = UPLOAD_URL;
    const char *auth_header_value = AUTH_HEADER_VALUE;
    const char *timestamp_value = "2024-01-15T12:30:00Z";
    char weight_value[32];
    char user_id_value[32] = "11";
    const char *file_field_name = "image_file";
    const char *file_name = "capture.jpg";
    const char *file_mime = "image/jpeg";
    const char *boundary = "----ESP32FormBoundary7MA4YWxkTrZu0gW";

    // Convert weight to string
    snprintf(weight_value, sizeof(weight_value), "%.1f", weight);

    // Prepare lengths for parts
    char header_part[512];
    int n;

    // We'll construct the body in PSRAM to avoid internal heap fragmentation
    // Compute exact size:
    size_t total_size = 0;

    // text parts
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

    // file header
    char file_header[512];
    n = snprintf(file_header, sizeof(file_header),
                 "--%s\r\n"
                 "Content-Disposition: form-data; name=\"%s\"; filename=\"%s\"\r\n"
                 "Content-Type: %s\r\n\r\n",
                 boundary, file_field_name, file_name, file_mime);
    if (n < 0) return ESP_ERR_INVALID_ARG;
    size_t file_header_len = (size_t)n;
    total_size += file_header_len;

    // file binary length
    total_size += jpeg_len;

    // ending boundary
    char ending[64];
    n = snprintf(ending, sizeof(ending), "\r\n--%s--\r\n", boundary);
    if (n < 0) return ESP_ERR_INVALID_ARG;
    size_t ending_len = (size_t)n;
    total_size += ending_len;

    // allocate body in PSRAM
    uint8_t *body = heap_caps_malloc(total_size + 1, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!body) {
        ESP_LOGE(TAG, "Failed to allocate PSRAM for multipart body size=%u", (unsigned)total_size);
        return ESP_ERR_NO_MEM;
    }

    // Fill body (carefully)
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

    // file header
    memcpy(body + offset, file_header, file_header_len);
    offset += file_header_len;

    // file data
    memcpy(body + offset, jpeg, jpeg_len);
    offset += jpeg_len;

    // ending
    memcpy(body + offset, ending, ending_len);
    offset += ending_len;

    if (offset != total_size) {
        ESP_LOGW(TAG, "constructed multipart size mismatch (offset=%u total=%u)", (unsigned)offset, (unsigned)total_size);
    }

    // Prepare HTTP client
    esp_http_client_config_t config = {
        .url = url,
        .timeout_ms = 15000,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) {
        ESP_LOGE(TAG, "esp_http_client_init failed");
        heap_caps_free(body);
        return ESP_FAIL;
    }

    // Headers
    char content_type_hdr[128];
    snprintf(content_type_hdr, sizeof(content_type_hdr), "multipart/form-data; boundary=%s", boundary);
    esp_http_client_set_header(client, "Content-Type", content_type_hdr);
    esp_http_client_set_header(client, "Authorization", auth_header_value);
    esp_http_client_set_header(client, "Accept", "application/json");

    // Set POST body
    esp_err_t err = esp_http_client_set_post_field(client, (const char*)body, total_size);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_http_client_set_post_field failed: %d", err);
        esp_http_client_cleanup(client);
        heap_caps_free(body);
        return err;
    }

    // Perform POST
    err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        int status = esp_http_client_get_status_code(client);
        int content_length = esp_http_client_get_content_length(client);
        ESP_LOGI(TAG, "Upload finished. HTTP status = %d, content_length = %d", status, content_length);
    } else {
        ESP_LOGE(TAG, "HTTP POST failed: %d", err);
    }

    esp_http_client_cleanup(client);
    heap_caps_free(body);

    return err;
}

/* -------------------------------------------------------------------
 * UI Event Handlers
 * -------------------------------------------------------------------*/
static void btn_record_meal_handler(lv_event_t *e) {
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        ESP_LOGI(TAG, "Record Meal button clicked");
        
        // Tare the scale
        hx711_tare(10);
        
        // Switch to weighing screen
        lv_screen_load(screen_weighing);
    }
}

static void btn_capture_handler(lv_event_t *e) {
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        ESP_LOGI(TAG, "Capture button clicked");

        // Get current weight
        current_meal.weight = hx711_get_weight();

        if (current_meal.weight < 1.0f) {
            ESP_LOGW(TAG, "No item detected on scale");
            return;
        }

        // Capture image (simplified - no local storage)
        if (!capture_image_simple(&current_meal.image_size)) {
            ESP_LOGE(TAG, "Failed to capture image");
            return;
        }

        // Upload image to API with weight data (buffered)
        esp_err_t upload_err = send_image_to_server_buffered(current_meal.image_size, current_meal.weight);
        if (upload_err != ESP_OK) {
            ESP_LOGW(TAG, "Failed to upload image to API: %d", upload_err);
            // Continue to show results even if upload failed
        } else {
            ESP_LOGI(TAG, "Image uploaded successfully to API");
        }

        // Recognize food (simulation for now)
        recognize_food(current_meal.weight, &current_meal);

        // Update result screen labels
        lv_label_set_text(result_food_label, current_meal.food_name);

        char weight_str[64];
        snprintf(weight_str, sizeof(weight_str), "Weight: %.1f g", current_meal.weight);
        lv_label_set_text(result_weight_label, weight_str);

        char nutrition_str[256];
        snprintf(nutrition_str, sizeof(nutrition_str),
                "Calories: %d kcal\n\n"
                "Protein: %.1f g\n"
                "Carbs: %.1f g\n"
                "Fat: %.1f g",
                current_meal.calories,
                current_meal.protein,
                current_meal.carbs,
                current_meal.fat);
        lv_label_set_text(result_nutrition_label, nutrition_str);

        // Switch to result screen
        lv_screen_load(screen_result);

        // Draw simulated food image on canvas (after screen is loaded)
        vTaskDelay(pdMS_TO_TICKS(50));
        draw_simulated_food_image(current_meal.weight);
    }
}

static void btn_back_home_handler(lv_event_t *e) {
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        ESP_LOGI(TAG, "Back to home");
        
        // Free image data if allocated
        if (current_meal.image_data) {
            heap_caps_free(current_meal.image_data);
            current_meal.image_data = NULL;
        }
        
        // Switch to home screen
        lv_screen_load(screen_home);
    }
}

static void btn_cancel_handler(lv_event_t *e) {
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        ESP_LOGI(TAG, "Cancel weighing");
        lv_screen_load(screen_home);
    }
}

/* -------------------------------------------------------------------
 * UI Creation
 * -------------------------------------------------------------------*/
static void create_home_screen(void) {
    screen_home = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(screen_home, lv_color_hex(0x0A0E27), LV_PART_MAIN);
    
    // Title
    lv_obj_t *title = lv_label_create(screen_home);
    lv_label_set_text(title, "Smart Plate");
    lv_obj_set_style_text_font(title, &lv_font_montserrat_32, 0);
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 40);
    
    // Subtitle
    lv_obj_t *subtitle = lv_label_create(screen_home);
    lv_label_set_text(subtitle, "AI-Powered Meal Tracking");
    lv_obj_set_style_text_color(subtitle, lv_color_hex(0x9CA3AF), 0);
    lv_obj_align(subtitle, LV_ALIGN_TOP_MID, 0, 85);
    
    // Record Meal Button
    lv_obj_t *btn_record = lv_button_create(screen_home);
    lv_obj_set_size(btn_record, 300, 80);
    lv_obj_align(btn_record, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_color(btn_record, lv_color_hex(0x667EEA), LV_PART_MAIN);
    lv_obj_set_style_radius(btn_record, 15, LV_PART_MAIN);
    lv_obj_add_event_cb(btn_record, btn_record_meal_handler, LV_EVENT_CLICKED, NULL);
    
    lv_obj_t *btn_label = lv_label_create(btn_record);
    lv_label_set_text(btn_label, LV_SYMBOL_PLUS " Record Meal");
    lv_obj_set_style_text_font(btn_label, &lv_font_montserrat_24, 0);
    lv_obj_center(btn_label);
    
    // Instructions
    lv_obj_t *instructions = lv_label_create(screen_home);
    lv_label_set_text(instructions, 
        "1. Place food on the plate\n"
        "2. Camera will capture image\n"
        "3. View nutrition information");
    lv_obj_set_style_text_color(instructions, lv_color_hex(0x9CA3AF), 0);
    lv_obj_set_style_text_align(instructions, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(instructions, LV_ALIGN_BOTTOM_MID, 0, -30);
}

static void create_weighing_screen(void) {
    screen_weighing = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(screen_weighing, lv_color_hex(0x0A0E27), LV_PART_MAIN);
    
    // Title
    lv_obj_t *title = lv_label_create(screen_weighing);
    lv_label_set_text(title, "Place Food on Plate");
    lv_obj_set_style_text_font(title, &lv_font_montserrat_28, 0);
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 30);
    
    // Weight display card
    lv_obj_t *weight_card = lv_obj_create(screen_weighing);
    lv_obj_set_size(weight_card, 400, 120);
    lv_obj_align(weight_card, LV_ALIGN_CENTER, 0, -30);
    lv_obj_set_style_bg_color(weight_card, lv_color_hex(0x1A1F3A), LV_PART_MAIN);
    lv_obj_set_style_border_width(weight_card, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(weight_card, 15, LV_PART_MAIN);
    
    weight_label = lv_label_create(weight_card);
    lv_label_set_text(weight_label, "0.0 g");
    lv_obj_set_style_text_font(weight_label, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_color(weight_label, lv_color_hex(0x4ECDC4), 0);
    lv_obj_center(weight_label);
    
    // Capture button
    lv_obj_t *btn_capture = lv_button_create(screen_weighing);
    lv_obj_set_size(btn_capture, 200, 60);
    lv_obj_align(btn_capture, LV_ALIGN_BOTTOM_MID, -60, -50);
    lv_obj_set_style_bg_color(btn_capture, lv_color_hex(0x00D4FF), LV_PART_MAIN);
    lv_obj_set_style_radius(btn_capture, 12, LV_PART_MAIN);
    lv_obj_add_event_cb(btn_capture, btn_capture_handler, LV_EVENT_CLICKED, NULL);
    
    lv_obj_t *capture_label = lv_label_create(btn_capture);
    lv_label_set_text(capture_label, LV_SYMBOL_OK " Capture");
    lv_obj_set_style_text_font(capture_label, &lv_font_montserrat_20, 0);
    lv_obj_center(capture_label);
    
    // Cancel button
    lv_obj_t *btn_cancel = lv_button_create(screen_weighing);
    lv_obj_set_size(btn_cancel, 100, 60);
    lv_obj_align(btn_cancel, LV_ALIGN_BOTTOM_MID, 90, -50);
    lv_obj_set_style_bg_color(btn_cancel, lv_color_hex(0xFF6B6B), LV_PART_MAIN);
    lv_obj_set_style_radius(btn_cancel, 12, LV_PART_MAIN);
    lv_obj_add_event_cb(btn_cancel, btn_cancel_handler, LV_EVENT_CLICKED, NULL);
    
    lv_obj_t *cancel_label = lv_label_create(btn_cancel);
    lv_label_set_text(cancel_label, LV_SYMBOL_CLOSE);
    lv_obj_set_style_text_font(cancel_label, &lv_font_montserrat_20, 0);
    lv_obj_center(cancel_label);
}

static void create_result_screen(void) {
    screen_result = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(screen_result, lv_color_hex(0x0A0E27), LV_PART_MAIN);
    
    // Title
    lv_obj_t *title = lv_label_create(screen_result);
    lv_label_set_text(title, "Meal Logged!");
    lv_obj_set_style_text_font(title, &lv_font_montserrat_24, 0);
    lv_obj_set_style_text_color(title, lv_color_hex(0x00FF88), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 15);
    
    // LEFT SIDE - Status indicator instead of image
    lv_obj_t *status_card = lv_obj_create(screen_result);
    lv_obj_set_size(status_card, 120, 80);
    lv_obj_align(status_card, LV_ALIGN_TOP_LEFT, 20, 50);
    lv_obj_set_style_bg_color(status_card, lv_color_hex(0x1A1F3A), LV_PART_MAIN);
    lv_obj_set_style_border_width(status_card, 2, LV_PART_MAIN);
    lv_obj_set_style_border_color(status_card, lv_color_hex(0x667EEA), LV_PART_MAIN);
    lv_obj_set_style_radius(status_card, 8, LV_PART_MAIN);

    lv_obj_t *status_icon = lv_label_create(status_card);
    lv_label_set_text(status_icon, LV_SYMBOL_OK);
    lv_obj_set_style_text_font(status_icon, &lv_font_montserrat_32, 0);
    lv_obj_set_style_text_color(status_icon, lv_color_hex(0x00FF88), 0);
    lv_obj_center(status_icon);

    lv_obj_t *status_label = lv_label_create(screen_result);
    lv_label_set_text(status_label, "Image\nCaptured");
    lv_obj_set_style_text_font(status_label, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(status_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(status_label, LV_ALIGN_TOP_LEFT, 20, 140);
    
    // LEFT SIDE - Food name (below image)
    result_food_label = lv_label_create(screen_result);
    lv_label_set_text(result_food_label, "Apple");
    lv_obj_set_style_text_font(result_food_label, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(result_food_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(result_food_label, LV_ALIGN_TOP_LEFT, 20, 155);
    
    // LEFT SIDE - Weight (below food name)
    result_weight_label = lv_label_create(screen_result);
    lv_label_set_text(result_weight_label, "Weight: 150.0 g");
    lv_obj_set_style_text_font(result_weight_label, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(result_weight_label, lv_color_hex(0x4ECDC4), 0);
    lv_obj_align(result_weight_label, LV_ALIGN_TOP_LEFT, 20, 185);
    
    // RIGHT SIDE - Nutrition card
    lv_obj_t *nutrition_card = lv_obj_create(screen_result);
    lv_obj_set_size(nutrition_card, 300, 160);
    lv_obj_align(nutrition_card, LV_ALIGN_TOP_RIGHT, -20, 50);
    lv_obj_set_style_bg_color(nutrition_card, lv_color_hex(0x1A1F3A), LV_PART_MAIN);
    lv_obj_set_style_border_width(nutrition_card, 2, LV_PART_MAIN);
    lv_obj_set_style_border_color(nutrition_card, lv_color_hex(0x667EEA), LV_PART_MAIN);
    lv_obj_set_style_radius(nutrition_card, 10, LV_PART_MAIN);
    lv_obj_set_style_pad_all(nutrition_card, 15, LV_PART_MAIN);
    
    lv_obj_t *nutrition_title = lv_label_create(nutrition_card);
    lv_label_set_text(nutrition_title, "Nutrition Facts");
    lv_obj_set_style_text_font(nutrition_title, &lv_font_montserrat_18, 0);
    lv_obj_set_style_text_color(nutrition_title, lv_color_hex(0xFFE66D), 0);
    lv_obj_set_pos(nutrition_title, 0, 0);
    
    result_nutrition_label = lv_label_create(nutrition_card);
    lv_label_set_text(result_nutrition_label, 
        "Calories: 95 kcal\n\n"
        "Protein: 0.5 g\n"
        "Carbs: 25.0 g\n"
        "Fat: 0.3 g");
    lv_obj_set_style_text_font(result_nutrition_label, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(result_nutrition_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_line_space(result_nutrition_label, 6, 0);
    lv_obj_set_pos(result_nutrition_label, 0, 35);
    
    // Done button at bottom
    lv_obj_t *btn_done = lv_button_create(screen_result);
    lv_obj_set_size(btn_done, 200, 50);
    lv_obj_align(btn_done, LV_ALIGN_BOTTOM_MID, 0, -10);
    lv_obj_set_style_bg_color(btn_done, lv_color_hex(0x667EEA), LV_PART_MAIN);
    lv_obj_set_style_radius(btn_done, 12, LV_PART_MAIN);
    lv_obj_add_event_cb(btn_done, btn_back_home_handler, LV_EVENT_CLICKED, NULL);
    
    lv_obj_t *done_label = lv_label_create(btn_done);
    lv_label_set_text(done_label, LV_SYMBOL_HOME " Done");
    lv_obj_set_style_text_font(done_label, &lv_font_montserrat_18, 0);
    lv_obj_center(done_label);
}

/* -------------------------------------------------------------------
 * Camera maintenance task
 * -------------------------------------------------------------------*/
static void camera_task(void *arg) {
    while (1) {
        captureThread(&myCAM); // ensures internal camera processing
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/* -------------------------------------------------------------------
 * LVGL handler task
 * -------------------------------------------------------------------*/
static void lvgl_task(void *arg) {
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Skip canvas buffer allocation to save memory for API calls
    ESP_LOGI(TAG, "Skipping canvas buffer allocation to conserve memory for API calls");
    canvas_buffer = NULL;
    
    // Create all screens
    create_home_screen();
    create_weighing_screen();
    create_result_screen();
    
    // Load home screen
    lv_screen_load(screen_home);
    
    ESP_LOGI(TAG, "Smart Plate UI initialized");
    
    while (1) {
        lv_timer_handler();
        
        // Update weight display if on weighing screen
        if (lv_screen_active() == screen_weighing && weight_label) {
            float current_weight = hx711_get_weight();
            char weight_str[32];
            snprintf(weight_str, sizeof(weight_str), "%.1f g", current_weight);
            lv_label_set_text(weight_label, weight_str);
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* -------------------------------------------------------------------
 * MAIN
 * -------------------------------------------------------------------*/
void app_main(void) {
    ESP_LOGI(TAG, "Starting Smart Plate System");

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // UART for debug
    uartBegin(115200);

    // WiFi
    ESP_LOGI(TAG, "Initializing WiFi...");
    wifi_init_sta();

    // Check available memory
    ESP_LOGI(TAG, "Free heap: %lu bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "Free PSRAM: %lu bytes", heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    ESP_LOGI(TAG, "Minimum free heap: %lu bytes", esp_get_minimum_free_heap_size());

    // Configure DC + RST pins
    gpio_config_t io = {
        .pin_bit_mask = (1ULL<<PIN_TFT_DC) | (1ULL<<PIN_TFT_RST),
        .mode = GPIO_MODE_OUTPUT
    };
    gpio_config(&io);
    gpio_set_level(PIN_TFT_DC, 1);
    gpio_set_level(PIN_TFT_RST, 1);
    
    // Initialize HX711
    hx711_init();

    // Initial tare with balanced samples for stability and speed
    hx711_tare(20);
    ESP_LOGI(TAG, "HX711 initialized");

    // Initialize camera (like working camera - SPI init handled inside begin())
    ESP_LOGI(TAG, "Initializing camera...");
    myCAM = createArducamCamera(CAMERA_CS);
    begin(&myCAM);
    ESP_LOGI(TAG, "Camera initialized");
    
    // Initialize I2C for touch controller
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized");

    // SPI init
    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_TFT_MOSI,
        .miso_io_num = PIN_TFT_MISO,
        .sclk_io_num = PIN_TFT_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = MAX_SPI_TRANS_BYTES
    };
    ESP_ERROR_CHECK(spi_bus_initialize(HSPI_HOST, &buscfg, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 40 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = PIN_TFT_CS,
        .queue_size = 7,
        .flags = SPI_DEVICE_NO_DUMMY,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(HSPI_HOST, &devcfg, &tft_spi));

    // LCD init
    ili9488_init();
    ESP_LOGI(TAG, "Display initialized");

    // Touch controller init
    touch_reset();
    touch_interrupt_init();
    ESP_ERROR_CHECK(ft6206_init(TOUCH_THRESHOLD));
    ESP_LOGI(TAG, "Touch controller initialized");

    // LVGL init
    lv_init();
    const esp_timer_create_args_t tick_args = {
        .callback = tick_inc,
        .name = "lv_tick"
    };
    esp_timer_handle_t tmr;
    esp_timer_create(&tick_args, &tmr);
    esp_timer_start_periodic(tmr, 1000);

    // Allocate buffers - try allocating as single block first to reduce fragmentation
    size_t lvgl_buf_size = BUF_PIXELS * sizeof(lv_color_t);
    size_t total_buf_size = RGB666_BUF_SIZE + (lvgl_buf_size * 2);
    size_t free_heap_before = esp_get_free_heap_size();
    ESP_LOGI(TAG, "Free heap before buffer allocation: %zu bytes", free_heap_before);
    ESP_LOGI(TAG, "Total buffer size needed: %zu bytes (rgb666: %zu, lv_buf1: %zu, lv_buf2: %zu)", 
             total_buf_size, RGB666_BUF_SIZE, lvgl_buf_size, lvgl_buf_size);
    
    // Try allocating all buffers as a single contiguous block first (reduces fragmentation)
    uint8_t *combined_buf = NULL;
    combined_buf = heap_caps_malloc(total_buf_size, MALLOC_CAP_DMA);
    if (!combined_buf) {
        ESP_LOGW(TAG, "DMA allocation failed for combined buffer (%zu bytes), trying regular heap", total_buf_size);
        combined_buf = heap_caps_malloc(total_buf_size, MALLOC_CAP_8BIT);
    }
    
    if (combined_buf) {
        // Successfully allocated as single block - split it up
        ESP_LOGI(TAG, "Allocated combined buffer: %zu bytes", total_buf_size);
        rgb666_buf = combined_buf;
        lv_buf1 = (lv_color_t *)(combined_buf + RGB666_BUF_SIZE);
        lv_buf2 = (lv_color_t *)(combined_buf + RGB666_BUF_SIZE + lvgl_buf_size);
        ESP_LOGI(TAG, "Split combined buffer into rgb666_buf, lv_buf1, lv_buf2");
    } else {
        // Fallback: try individual allocations
        ESP_LOGW(TAG, "Combined allocation failed, trying individual allocations");
        
        rgb666_buf = heap_caps_malloc(RGB666_BUF_SIZE, MALLOC_CAP_DMA);
        if (!rgb666_buf) {
            ESP_LOGW(TAG, "DMA allocation failed for rgb666_buf (%zu bytes), trying regular heap", RGB666_BUF_SIZE);
            rgb666_buf = heap_caps_malloc(RGB666_BUF_SIZE, MALLOC_CAP_8BIT);
        }
        if (!rgb666_buf) {
            ESP_LOGE(TAG, "Failed to allocate rgb666_buf (%zu bytes). Free heap: %zu", RGB666_BUF_SIZE, esp_get_free_heap_size());
            return;
        }
        ESP_LOGI(TAG, "rgb666_buf allocated: %zu bytes", RGB666_BUF_SIZE);
        
        lv_buf1 = heap_caps_malloc(lvgl_buf_size, MALLOC_CAP_DMA);
        if (!lv_buf1) {
            ESP_LOGW(TAG, "DMA allocation failed for lv_buf1 (%zu bytes), trying regular heap", lvgl_buf_size);
            lv_buf1 = heap_caps_malloc(lvgl_buf_size, MALLOC_CAP_8BIT);
        }
        if (!lv_buf1) {
            ESP_LOGE(TAG, "Failed to allocate lv_buf1 (%zu bytes). Free heap: %zu", lvgl_buf_size, esp_get_free_heap_size());
            heap_caps_free(rgb666_buf);
            rgb666_buf = NULL;
            return;
        }
        ESP_LOGI(TAG, "lv_buf1 allocated: %zu bytes", lvgl_buf_size);
        
        // Try allocating lv_buf2 - if it fails, try freeing and reallocating in different order
        lv_buf2 = heap_caps_malloc(lvgl_buf_size, MALLOC_CAP_DMA);
        if (!lv_buf2) {
            ESP_LOGW(TAG, "DMA allocation failed for lv_buf2 (%zu bytes), trying regular heap", lvgl_buf_size);
            lv_buf2 = heap_caps_malloc(lvgl_buf_size, MALLOC_CAP_8BIT);
        }
        
        // If still failing, try freeing lv_buf1 and reallocating both together
        if (!lv_buf2) {
            ESP_LOGW(TAG, "Direct allocation failed, trying reallocation strategy");
            heap_caps_free(lv_buf1);
            lv_buf1 = NULL;
            
            // Try allocating both lv buffers together
            size_t two_buf_size = lvgl_buf_size * 2;
            uint8_t *two_bufs = heap_caps_malloc(two_buf_size, MALLOC_CAP_DMA);
            if (!two_bufs) {
                two_bufs = heap_caps_malloc(two_buf_size, MALLOC_CAP_8BIT);
            }
            
            if (two_bufs) {
                lv_buf1 = (lv_color_t *)two_bufs;
                lv_buf2 = (lv_color_t *)(two_bufs + lvgl_buf_size);
                ESP_LOGI(TAG, "Allocated lv_buf1 and lv_buf2 together: %zu bytes each", lvgl_buf_size);
            } else {
                ESP_LOGE(TAG, "Failed to allocate lv_buf2 (%zu bytes). Free heap: %zu", lvgl_buf_size, esp_get_free_heap_size());
                heap_caps_free(rgb666_buf);
                rgb666_buf = NULL;
                return;
            }
        } else {
            ESP_LOGI(TAG, "lv_buf2 allocated: %zu bytes", lvgl_buf_size);
        }
    }
    
    size_t free_heap_after = esp_get_free_heap_size();
    ESP_LOGI(TAG, "All buffers allocated. Free heap after: %zu bytes (used: %zu)", 
             free_heap_after, free_heap_before - free_heap_after);
    
    ESP_LOGI(TAG, "Buffers allocated");

    // Create LVGL display
    lv_display_t *disp = lv_display_create(HOR_RES, VER_RES);
    lv_display_set_flush_cb(disp, ili9488_flush);
    lv_display_set_buffers(disp, lv_buf1, lv_buf2, lvgl_buf_size, LV_DISPLAY_RENDER_MODE_PARTIAL);
    
    // Register touch input
    lv_indev_t *indev = lv_indev_create();
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(indev, touch_read_cb);
    
    ESP_LOGI(TAG, "LVGL initialized");

    // Create LVGL task
    xTaskCreatePinnedToCore(lvgl_task, "lvgl", 8192, NULL, 5, NULL, 1);

    // Create camera maintenance task
    xTaskCreatePinnedToCore(camera_task, "camera_task", 4096, NULL, 4, NULL, 1);

    ESP_LOGI(TAG, "Smart Plate System Ready!");
}

