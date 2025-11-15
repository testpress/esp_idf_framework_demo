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

/* Transfer sizing */
#define MAX_SPI_TRANS_BYTES  (8 * 1024)
#define LINES                38
#define BUF_PIXELS           (HOR_RES * LINES)
#define RGB666_BUF_SIZE      (BUF_PIXELS * 3)

// Global variables
static spi_device_handle_t tft_spi;
static lv_color_t *lv_buf1 = NULL;
static lv_color_t *lv_buf2 = NULL;
static uint8_t *rgb666_buf = NULL;
static volatile bool touch_interrupt_flag = false;

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

// Canvas buffer for captured image display (120x90 pixels - reduced for memory)
#define CANVAS_WIDTH  120
#define CANVAS_HEIGHT 90
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
        vTaskDelay(10 / portTICK_PERIOD_MS);
        if ((esp_timer_get_time() - start) > timeout_ms * 1000) {
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
    ESP_LOGI(TAG, "HX711 Tared: offset=%ld", hx711_offset);
}

static float hx711_get_weight(void) {
    long raw = hx711_read_average(5);
    return (float)((raw - hx711_offset) * hx711_scale);
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
    vTaskDelay(pdMS_TO_TICKS(20));
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

static bool capture_image(uint8_t **image_data, size_t *image_size) {
    // Simulate image capture - replace with actual camera code
    // For now, create a dummy image (small JPEG-like structure)
    *image_size = 1024; // Dummy size
    
    // Try PSRAM first, fall back to regular heap if PSRAM not available
    *image_data = heap_caps_malloc(*image_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    
    if (*image_data == NULL) {
        ESP_LOGW(TAG, "PSRAM allocation failed, trying regular heap");
        *image_data = heap_caps_malloc(*image_size, MALLOC_CAP_8BIT);
    }
    
    if (*image_data == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for image (tried PSRAM and heap)");
        
        // Check available memory
        ESP_LOGE(TAG, "Free heap: %lu bytes, Free PSRAM: %lu bytes", 
                 esp_get_free_heap_size(), 
                 heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
        return false;
    }
    
    // Fill with dummy data
    memset(*image_data, 0xAA, *image_size);
    
    ESP_LOGI(TAG, "Image captured (simulated): %d bytes (allocated in %s)", 
             *image_size,
             heap_caps_get_allocated_size(*image_data) ? "heap" : "PSRAM");
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
        
        // Capture image
        if (!capture_image(&current_meal.image_data, &current_meal.image_size)) {
            ESP_LOGE(TAG, "Failed to capture image");
            return;
        }
        
        // Recognize food
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
    
    // LEFT SIDE - Canvas for captured image
    if (canvas_buffer) {
        result_image_canvas = lv_canvas_create(screen_result);
        lv_canvas_set_buffer(result_image_canvas, canvas_buffer, 
                            CANVAS_WIDTH, CANVAS_HEIGHT, LV_COLOR_FORMAT_RGB565);
        lv_obj_align(result_image_canvas, LV_ALIGN_TOP_LEFT, 20, 50);
        
        // Add border to canvas
        lv_obj_set_style_border_width(result_image_canvas, 2, LV_PART_MAIN);
        lv_obj_set_style_border_color(result_image_canvas, lv_color_hex(0x667EEA), LV_PART_MAIN);
        lv_obj_set_style_radius(result_image_canvas, 8, LV_PART_MAIN);
        
        // Initially fill with placeholder
        lv_color_t bg_color = lv_color_hex(0x1A1F3A);
        lv_color_t icon_color = lv_color_hex(0x667EEA);
        
        for (int i = 0; i < CANVAS_WIDTH * CANVAS_HEIGHT; i++) {
            canvas_buffer[i] = bg_color;
        }
        
        // Draw simple camera icon
        int icon_x = CANVAS_WIDTH / 2;
        int icon_y = CANVAS_HEIGHT / 2;
        
        for (int y = icon_y - 15; y < icon_y + 15; y++) {
            for (int x = icon_x - 15; x < icon_x + 15; x++) {
                if (y >= 0 && y < CANVAS_HEIGHT && x >= 0 && x < CANVAS_WIDTH) {
                    int dx = x - icon_x;
                    int dy = y - icon_y;
                    int dist_sq = dx * dx + dy * dy;
                    
                    if (dist_sq <= 12 * 12 && dist_sq >= 8 * 8) {
                        canvas_buffer[y * CANVAS_WIDTH + x] = icon_color;
                    }
                }
            }
        }
    } else {
        ESP_LOGE(TAG, "Canvas buffer not allocated");
        return;
    }
    
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
 * LVGL handler task
 * -------------------------------------------------------------------*/
static void lvgl_task(void *arg) {
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Allocate canvas buffer (120x90x2 = 21,600 bytes)
    canvas_buffer = heap_caps_malloc(CANVAS_WIDTH * CANVAS_HEIGHT * sizeof(lv_color_t), MALLOC_CAP_8BIT);
    if (!canvas_buffer) {
        ESP_LOGE(TAG, "Failed to allocate canvas buffer");
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "Canvas buffer allocated: %d bytes", CANVAS_WIDTH * CANVAS_HEIGHT * sizeof(lv_color_t));
    
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
    hx711_tare(20);
    ESP_LOGI(TAG, "HX711 initialized");
    
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

    // Allocate buffers
    rgb666_buf = heap_caps_malloc(RGB666_BUF_SIZE, MALLOC_CAP_DMA);
    size_t lvgl_buf_size = BUF_PIXELS * sizeof(lv_color_t);
    lv_buf1 = heap_caps_malloc(lvgl_buf_size, MALLOC_CAP_DMA);
    lv_buf2 = heap_caps_malloc(lvgl_buf_size, MALLOC_CAP_DMA);
    
    if (!rgb666_buf || !lv_buf1 || !lv_buf2) {
        ESP_LOGE(TAG, "Failed to allocate buffers");
        return;
    }
    
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

    ESP_LOGI(TAG, "Smart Plate System Ready!");
}
