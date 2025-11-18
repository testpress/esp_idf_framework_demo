// main.c — LVGL v9 + ILI9488 (chunked SPI) + FT6206 Touch - LVGL handler pinned to core 1
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/i2c.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_random.h"

#include <string.h>
#include "lvgl.h"

static const char *TAG = "LVGL9_ILI9488_PINNED";

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
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0
#define I2C_MASTER_TIMEOUT_MS       1000

#define PIN_TOUCH_IRQ  27
#define PIN_TOUCH_RST  32
#define FT6206_I2C_ADDR    0x38
#define TOUCH_THRESHOLD    40

static spi_device_handle_t tft_spi;

/* Touch interrupt flag */
static volatile bool touch_interrupt_flag = false;

/* Transfer sizing */
#define MAX_SPI_TRANS_BYTES  (8 * 1024)   // must match spi_bus_config.max_transfer_sz
#define LINES                38           // Large buffer = near-instant updates (80 lines = 1/4 screen)
#define BUF_PIXELS           (HOR_RES * LINES)
#define RGB666_BUF_SIZE      (BUF_PIXELS * 3)

// Dynamic buffers - allocated at runtime to save static RAM
static lv_color_t *lv_buf1 = NULL;    // First buffer (dynamically allocated)
static lv_color_t *lv_buf2 = NULL;    // Second buffer for double buffering (dynamically allocated)
static uint8_t *rgb666_buf = NULL;

/* -------------------------------------------------------------------
 * Low-level SPI helpers (with chunking)
 * -------------------------------------------------------------------*/
 static void send_cmd(uint8_t cmd)
 {
     gpio_set_level(PIN_TFT_DC, 0);
     spi_transaction_t t = {0};
     t.length = 8; // bits
     t.flags = SPI_TRANS_USE_TXDATA;
     t.tx_data[0] = cmd;
     esp_err_t err = spi_device_polling_transmit(tft_spi, &t);
     if (err != ESP_OK) {
         ESP_LOGW(TAG, "send_cmd spi tx err %s", esp_err_to_name(err));
     }
 } 

static void send_data_chunked(const uint8_t *data, size_t len)
{
    if (!data || len == 0) return;
    gpio_set_level(PIN_TFT_DC, 1);
    size_t sent = 0;
    while (sent < len) {
        size_t to_send = len - sent;
        if (to_send > MAX_SPI_TRANS_BYTES) to_send = MAX_SPI_TRANS_BYTES;
        spi_transaction_t t = {0};
        t.length = to_send * 8;
        t.tx_buffer = data + sent;
        esp_err_t err = spi_device_polling_transmit(tft_spi, &t);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "send_data_chunked spi tx err %s (sent=%u to_send=%u)",
                     esp_err_to_name(err), (unsigned)sent, (unsigned)to_send);
            // continue to attempt remaining transfers
        }
        sent += to_send;
    }
}

static void send_data(const uint8_t *data, size_t len)
{
    send_data_chunked(data, len);
}

/* -------------------------------------------------------------------
 * ILI9488 init
 * -------------------------------------------------------------------*/
static void ili9488_init(void)
{
    ESP_LOGI(TAG, "LCD Reset");
    gpio_set_level(PIN_TFT_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(PIN_TFT_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(120));

    send_cmd(0x11); // Sleep Out
    vTaskDelay(pdMS_TO_TICKS(120));

    send_cmd(0x36); // MADCTL
    uint8_t madctl = 0xE8; // landscape
    send_data(&madctl, 1);

    send_cmd(0x3A); // Pixel format
    uint8_t pix = 0x66; // RGB666
    send_data(&pix, 1);

    send_cmd(0x29); // Display ON
    vTaskDelay(pdMS_TO_TICKS(20));
}

/* -------------------------------------------------------------------
 * LVGL flush (v9): RGB565 -> RGB666, chunked SPI send
 * -------------------------------------------------------------------*/
static void ili9488_flush(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
    // Use the pixel data directly (LVGL v9 handles palette metadata internally)
    uint16_t *src = (uint16_t *)px_map;

    int32_t w = lv_area_get_width(area);
    int32_t h = lv_area_get_height(area);
    int total = w * h;

    // Safety check
    if (!rgb666_buf) {
        ESP_LOGE(TAG, "rgb666_buf NULL in flush");
        lv_display_flush_ready(disp);
        return;
    }

    // Convert RGB565 to RGB666 with proper scaling (original working algorithm)
    for (int i = 0; i < total; ++i) {
        uint16_t c = src[i];
        uint8_t r5 = (c >> 11) & 0x1F;
        uint8_t g6 = (c >> 5)  & 0x3F;
        uint8_t b5 = (c)       & 0x1F;
        rgb666_buf[3*i + 0] = (uint8_t)(r5 << 3);  // 5-bit -> 8-bit (multiply by 8)
        rgb666_buf[3*i + 1] = (uint8_t)(g6 << 2);  // 6-bit -> 8-bit (multiply by 4)
        rgb666_buf[3*i + 2] = (uint8_t)(b5 << 3);  // 5-bit -> 8-bit (multiply by 8)
    }

    uint8_t col[4] = {
        (uint8_t)((area->x1 >> 8) & 0xFF), (uint8_t)(area->x1 & 0xFF),
        (uint8_t)((area->x2 >> 8) & 0xFF), (uint8_t)(area->x2 & 0xFF)
    };
    send_cmd(0x2A);
    send_data(col, 4);

    uint8_t row[4] = {
        (uint8_t)((area->y1 >> 8) & 0xFF), (uint8_t)(area->y1 & 0xFF),
        (uint8_t)((area->y2 >> 8) & 0xFF), (uint8_t)(area->y2 & 0xFF)
    };
    send_cmd(0x2B);
    send_data(row, 4);

    send_cmd(0x2C);
    send_data_chunked(rgb666_buf, total * 3);

    lv_display_flush_ready(disp);
}

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

        // Calibrated touch mapping based on measured corner touches:
        // Top-left: raw(14,462) → screen(0,0)
        // Top-right: raw(53,0) → screen(479,0)
        // Bottom-left: raw(318,472) → screen(0,319)
        // Bottom-right: raw(304,2) → screen(479,319)
        //
        // NOTE: User values are approximate (finger touches near corners)
        // Adding margins to extend touch area beyond screen edges for better accuracy
        
        // Calibration bounds with margins (extend ~10 pixels beyond measured values)
        #define TOUCH_X_MIN 10       // Measured: 14, with margin: 10
        #define TOUCH_X_MAX 322      // Measured: 318, with margin: 322
        #define TOUCH_Y_MIN 0        // Already at edge
        #define TOUCH_Y_MAX 470      // Measured: 462, with margin: 470
        
        // Map raw_y (462→0) to screen X (0→479)
        int32_t mapped_x = ((int32_t)(TOUCH_Y_MAX - raw_y) * (HOR_RES - 1)) / TOUCH_Y_MAX;
        
        // Map raw_x (14→318) to screen Y (0→319)
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

/* -------------------------------------------------------------------
 * LVGL tick
 * -------------------------------------------------------------------*/
static void tick_inc(void *arg)
{
    lv_tick_inc(1);
}

/* -------------------------------------------------------------------
 * LVGL UI Components and Event Handlers
 * -------------------------------------------------------------------*/
static lv_obj_t *temp_label = NULL;
static lv_obj_t *humidity_label = NULL;
static lv_obj_t *pressure_label = NULL;
static lv_obj_t *cpu_arc = NULL;
static lv_obj_t *mem_arc = NULL;
static lv_obj_t *wifi_switch = NULL;
static lv_obj_t *bt_switch = NULL;
static lv_obj_t *brightness_slider = NULL;
static lv_obj_t *brightness_value = NULL;
static lv_obj_t *status_dot = NULL;
static lv_obj_t *nav_buttons[3] = {NULL, NULL, NULL};
static lv_obj_t *screen_overlay = NULL;

static void nav_btn_handler(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_CLICKED) {
        lv_obj_t *tabview = lv_event_get_user_data(e);
        lv_obj_t *btn = lv_event_get_target(e);
        uint32_t tab_idx = (uint32_t)(uintptr_t)lv_obj_get_user_data(btn);
        lv_tabview_set_active(tabview, tab_idx, LV_ANIM_OFF);  // Instant navigation - no animation!
        
        // Update all navigation button colors
        for (int i = 0; i < 3; i++) {
            if (nav_buttons[i]) {
                lv_obj_set_style_bg_color(nav_buttons[i], 
                    i == tab_idx ? lv_color_hex(0x667EEA) : lv_color_hex(0x2A2F4A), 
                    LV_PART_MAIN);
            }
        }
        
        ESP_LOGI(TAG, "Navigated to tab %d", (int)tab_idx);
    }
}

static void wifi_switch_handler(lv_event_t *e)
{
    lv_obj_t *sw = lv_event_get_target(e);
    bool state = lv_obj_has_state(sw, LV_STATE_CHECKED);
    ESP_LOGI(TAG, "WiFi %s", state ? "ON" : "OFF");
}

static void bt_switch_handler(lv_event_t *e)
{
    lv_obj_t *sw = lv_event_get_target(e);
    bool state = lv_obj_has_state(sw, LV_STATE_CHECKED);
    ESP_LOGI(TAG, "Bluetooth %s", state ? "ON" : "OFF");
}

static void brightness_handler(lv_event_t *e)
{
    lv_obj_t *slider = lv_event_get_target(e);
    int32_t value = lv_slider_get_value(slider);
    if (brightness_value) {
        lv_label_set_text_fmt(brightness_value, "%d%%", (int)value);
    }
    
    // Control actual brightness using screen overlay opacity
    if (screen_overlay) {
        // Map 0-100% slider to opacity: 100% brightness = 0 opacity, 0% brightness = 200 opacity
        int32_t opacity = (100 - value) * 2;
        if (opacity > 255) opacity = 255;
        if (opacity < 0) opacity = 0;
        lv_obj_set_style_opa(screen_overlay, opacity, LV_PART_MAIN);
    }
}

static void action_btn_handler(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_CLICKED) {
        lv_obj_t *btn = lv_event_get_target(e);
        const char *action = (const char *)lv_event_get_user_data(e);
        ESP_LOGI(TAG, "Action button pressed: %s", action);
        
        // Visual feedback
        lv_obj_set_style_bg_color(btn, lv_color_hex(0x00D4FF), LV_PART_MAIN);
        lv_obj_invalidate(btn);
    } else if (code == LV_EVENT_RELEASED) {
        lv_obj_t *btn = lv_event_get_target(e);
        lv_obj_set_style_bg_color(btn, lv_color_hex(0x2196F3), LV_PART_MAIN);
    }
}

static void create_demo_ui(void)
{
    lv_obj_t *scr = lv_screen_active();
    
    /* Professional dark theme background */
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x0A0E27), LV_PART_MAIN);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
    
    /* ==================== TOP STATUS BAR ==================== */
    lv_obj_t *top_bar = lv_obj_create(scr);
    lv_obj_set_size(top_bar, HOR_RES, 40);
    lv_obj_set_pos(top_bar, 0, 0);
    lv_obj_set_style_bg_color(top_bar, lv_color_hex(0x1A1F3A), LV_PART_MAIN);
    lv_obj_set_style_border_width(top_bar, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(top_bar, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(top_bar, 0, LV_PART_MAIN);
    lv_obj_clear_flag(top_bar, LV_OBJ_FLAG_SCROLLABLE);
    
    /* App Title */
    lv_obj_t *app_title = lv_label_create(top_bar);
    lv_label_set_text(app_title, "IoT Control");
    lv_obj_set_style_text_color(app_title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_pos(app_title, 10, 10);
    
    /* Status Indicator */
    status_dot = lv_obj_create(top_bar);
    lv_obj_set_size(status_dot, 10, 10);
    lv_obj_set_pos(status_dot, HOR_RES - 20, 15);
    lv_obj_set_style_bg_color(status_dot, lv_color_hex(0x00FF88), LV_PART_MAIN);
    lv_obj_set_style_border_width(status_dot, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(status_dot, 5, LV_PART_MAIN);
    lv_obj_clear_flag(status_dot, LV_OBJ_FLAG_SCROLLABLE);
    
    /* ==================== TABVIEW WITH SWIPE ==================== */
    lv_obj_t *tabview = lv_tabview_create(scr);
    lv_obj_set_size(tabview, HOR_RES, VER_RES - 90);
    lv_obj_set_pos(tabview, 0, 40);
    lv_obj_set_style_bg_color(tabview, lv_color_hex(0x0A0E27), LV_PART_MAIN);
    lv_obj_set_style_border_width(tabview, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(tabview, 0, LV_PART_MAIN);
    lv_tabview_set_tab_bar_size(tabview, 0); // Hide default tab bar
    
    /* ==================== TAB 1: DASHBOARD ==================== */
    lv_obj_t *tab_dashboard = lv_tabview_add_tab(tabview, "");
    lv_obj_set_style_bg_color(tab_dashboard, lv_color_hex(0x0A0E27), LV_PART_MAIN);
    lv_obj_set_style_pad_all(tab_dashboard, 0, LV_PART_MAIN);
    lv_obj_clear_flag(tab_dashboard, LV_OBJ_FLAG_SCROLLABLE);
    
    /* Sensor Cards Row 1 */
    const char *sensor_names[] = {"Temp", "Humid", "Press"};
    const char *sensor_icons[] = {LV_SYMBOL_WARNING, LV_SYMBOL_CHARGE, LV_SYMBOL_EYE_OPEN};
    lv_color_t sensor_colors[] = {
        lv_color_hex(0xFF6B6B),
        lv_color_hex(0x4ECDC4),
        lv_color_hex(0xFFE66D)
    };
    lv_obj_t **sensor_labels[] = {&temp_label, &humidity_label, &pressure_label};
    
    for (int i = 0; i < 3; i++) {
        lv_obj_t *card = lv_obj_create(tab_dashboard);
        lv_obj_set_size(card, 150, 70);
        lv_obj_set_pos(card, 8 + i * 158, 5);
        lv_obj_set_style_bg_color(card, lv_color_hex(0x1A1F3A), LV_PART_MAIN);
        lv_obj_set_style_border_color(card, sensor_colors[i], LV_PART_MAIN);
        lv_obj_set_style_border_width(card, 2, LV_PART_MAIN);
        lv_obj_set_style_radius(card, 10, LV_PART_MAIN);
        lv_obj_set_style_pad_all(card, 5, LV_PART_MAIN);
        lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);
        
        lv_obj_t *icon = lv_label_create(card);
        lv_label_set_text(icon, sensor_icons[i]);
        lv_obj_set_pos(icon, 8, 5);
        lv_obj_set_style_text_color(icon, sensor_colors[i], 0);
        
        lv_obj_t *name = lv_label_create(card);
        lv_label_set_text(name, sensor_names[i]);
        lv_obj_set_pos(name, 8, 25);
        lv_obj_set_style_text_color(name, lv_color_hex(0x9CA3AF), 0);
        
        *sensor_labels[i] = lv_label_create(card);
        lv_label_set_text(*sensor_labels[i], i == 0 ? "24.5C" : i == 1 ? "65%" : "1013");
        lv_obj_set_pos(*sensor_labels[i], 8, 43);
        lv_obj_set_style_text_color(*sensor_labels[i], lv_color_hex(0xFFFFFF), 0);
        lv_label_set_long_mode(*sensor_labels[i], LV_LABEL_LONG_CLIP);
    }
    
    /* System Performance Arcs */
    const char *perf_names[] = {"CPU", "MEM"};
    lv_obj_t **perf_arcs[] = {&cpu_arc, &mem_arc};
    lv_color_t perf_colors[] = {lv_color_hex(0x667EEA), lv_color_hex(0xF093FB)};
    
    for (int i = 0; i < 2; i++) {
        lv_obj_t *arc_card = lv_obj_create(tab_dashboard);
        lv_obj_set_size(arc_card, 232, 95);
        lv_obj_set_pos(arc_card, 8 + i * 240, 80);
        lv_obj_set_style_bg_color(arc_card, lv_color_hex(0x1A1F3A), LV_PART_MAIN);
        lv_obj_set_style_border_width(arc_card, 0, LV_PART_MAIN);
        lv_obj_set_style_radius(arc_card, 10, LV_PART_MAIN);
        lv_obj_set_style_pad_all(arc_card, 5, LV_PART_MAIN);
        lv_obj_clear_flag(arc_card, LV_OBJ_FLAG_SCROLLABLE);
        
        lv_obj_t *title = lv_label_create(arc_card);
        lv_label_set_text(title, perf_names[i]);
        lv_obj_set_pos(title, 10, 8);
        lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
        
        *perf_arcs[i] = lv_arc_create(arc_card);
        lv_obj_set_size(*perf_arcs[i], 75, 75);
        lv_obj_set_pos(*perf_arcs[i], 145, 8);
        lv_arc_set_range(*perf_arcs[i], 0, 100);
        lv_arc_set_value(*perf_arcs[i], 0);
        lv_obj_set_style_arc_color(*perf_arcs[i], lv_color_hex(0x2A2F4A), LV_PART_MAIN);
        lv_obj_set_style_arc_color(*perf_arcs[i], perf_colors[i], LV_PART_INDICATOR);
        lv_obj_set_style_arc_width(*perf_arcs[i], 8, LV_PART_MAIN);
        lv_obj_set_style_arc_width(*perf_arcs[i], 8, LV_PART_INDICATOR);
        lv_obj_clear_flag(*perf_arcs[i], LV_OBJ_FLAG_CLICKABLE);
        
        lv_obj_t *percent = lv_label_create(*perf_arcs[i]);
        lv_label_set_text(percent, "0%");
        lv_obj_center(percent);
        lv_obj_set_style_text_color(percent, perf_colors[i], 0);
    }
    
    /* ==================== TAB 2: CONTROLS ==================== */
    lv_obj_t *tab_controls = lv_tabview_add_tab(tabview, "");
    lv_obj_set_style_bg_color(tab_controls, lv_color_hex(0x0A0E27), LV_PART_MAIN);
    lv_obj_set_style_pad_all(tab_controls, 0, LV_PART_MAIN);
    lv_obj_clear_flag(tab_controls, LV_OBJ_FLAG_SCROLLABLE);
    
    /* WiFi Control Card */
    lv_obj_t *wifi_card = lv_obj_create(tab_controls);
    lv_obj_set_size(wifi_card, 464, 55);
    lv_obj_set_pos(wifi_card, 8, 5);
    lv_obj_set_style_bg_color(wifi_card, lv_color_hex(0x1A1F3A), LV_PART_MAIN);
    lv_obj_set_style_border_width(wifi_card, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(wifi_card, 10, LV_PART_MAIN);
    lv_obj_set_style_pad_all(wifi_card, 5, LV_PART_MAIN);
    lv_obj_clear_flag(wifi_card, LV_OBJ_FLAG_SCROLLABLE);
    
    lv_obj_t *wifi_icon = lv_label_create(wifi_card);
    lv_label_set_text(wifi_icon, LV_SYMBOL_WIFI);
    lv_obj_set_pos(wifi_icon, 10, 15);
    lv_obj_set_style_text_color(wifi_icon, lv_color_hex(0x667EEA), 0);
    
    lv_obj_t *wifi_label = lv_label_create(wifi_card);
    lv_label_set_text(wifi_label, "WiFi");
    lv_obj_set_pos(wifi_label, 45, 8);
    lv_obj_set_style_text_color(wifi_label, lv_color_hex(0xFFFFFF), 0);
    
    lv_obj_t *wifi_status = lv_label_create(wifi_card);
    lv_label_set_text(wifi_status, "Connected");
    lv_obj_set_pos(wifi_status, 45, 28);
    lv_obj_set_style_text_color(wifi_status, lv_color_hex(0x9CA3AF), 0);
    
    wifi_switch = lv_switch_create(wifi_card);
    lv_obj_set_pos(wifi_switch, 390, 12);
    lv_obj_set_style_bg_color(wifi_switch, lv_color_hex(0x667EEA), LV_PART_INDICATOR | LV_STATE_CHECKED);
    lv_obj_add_event_cb(wifi_switch, wifi_switch_handler, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_add_state(wifi_switch, LV_STATE_CHECKED);
    
    /* Bluetooth Control Card */
    lv_obj_t *bt_card = lv_obj_create(tab_controls);
    lv_obj_set_size(bt_card, 464, 55);
    lv_obj_set_pos(bt_card, 8, 65);
    lv_obj_set_style_bg_color(bt_card, lv_color_hex(0x1A1F3A), LV_PART_MAIN);
    lv_obj_set_style_border_width(bt_card, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(bt_card, 10, LV_PART_MAIN);
    lv_obj_set_style_pad_all(bt_card, 5, LV_PART_MAIN);
    lv_obj_clear_flag(bt_card, LV_OBJ_FLAG_SCROLLABLE);
    
    lv_obj_t *bt_icon = lv_label_create(bt_card);
    lv_label_set_text(bt_icon, LV_SYMBOL_BLUETOOTH);
    lv_obj_set_pos(bt_icon, 10, 15);
    lv_obj_set_style_text_color(bt_icon, lv_color_hex(0x2196F3), 0);
    
    lv_obj_t *bt_label = lv_label_create(bt_card);
    lv_label_set_text(bt_label, "Bluetooth");
    lv_obj_set_pos(bt_label, 45, 8);
    lv_obj_set_style_text_color(bt_label, lv_color_hex(0xFFFFFF), 0);
    
    lv_obj_t *bt_status = lv_label_create(bt_card);
    lv_label_set_text(bt_status, "Disconnected");
    lv_obj_set_pos(bt_status, 45, 28);
    lv_obj_set_style_text_color(bt_status, lv_color_hex(0x9CA3AF), 0);
    
    bt_switch = lv_switch_create(bt_card);
    lv_obj_set_pos(bt_switch, 390, 12);
    lv_obj_set_style_bg_color(bt_switch, lv_color_hex(0x2196F3), LV_PART_INDICATOR | LV_STATE_CHECKED);
    lv_obj_add_event_cb(bt_switch, bt_switch_handler, LV_EVENT_VALUE_CHANGED, NULL);
    
    /* Brightness Slider Card */
    lv_obj_t *bright_card = lv_obj_create(tab_controls);
    lv_obj_set_size(bright_card, 464, 65);
    lv_obj_set_pos(bright_card, 8, 125);
    lv_obj_set_style_bg_color(bright_card, lv_color_hex(0x1A1F3A), LV_PART_MAIN);
    lv_obj_set_style_border_width(bright_card, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(bright_card, 10, LV_PART_MAIN);
    lv_obj_set_style_pad_all(bright_card, 5, LV_PART_MAIN);
    lv_obj_clear_flag(bright_card, LV_OBJ_FLAG_SCROLLABLE);
    
    lv_obj_t *bright_label = lv_label_create(bright_card);
    lv_label_set_text(bright_label, LV_SYMBOL_IMAGE " Brightness");
    lv_obj_set_pos(bright_label, 10, 8);
    lv_obj_set_style_text_color(bright_label, lv_color_hex(0xFFFFFF), 0);
    
    brightness_value = lv_label_create(bright_card);
    lv_label_set_text(brightness_value, "80%");
    lv_obj_set_pos(brightness_value, 420, 8);
    lv_obj_set_style_text_color(brightness_value, lv_color_hex(0xFFE66D), 0);
    
    brightness_slider = lv_slider_create(bright_card);
    lv_obj_set_size(brightness_slider, 440, 12);
    lv_obj_set_pos(brightness_slider, 10, 38);
    lv_slider_set_value(brightness_slider, 80, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(brightness_slider, lv_color_hex(0x2A2F4A), LV_PART_MAIN);
    lv_obj_set_style_bg_color(brightness_slider, lv_color_hex(0xFFE66D), LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(brightness_slider, lv_color_hex(0xFFFFFF), LV_PART_KNOB);
    lv_obj_add_event_cb(brightness_slider, brightness_handler, LV_EVENT_VALUE_CHANGED, NULL);
    
    /* ==================== TAB 3: ACTIONS ==================== */
    lv_obj_t *tab_actions = lv_tabview_add_tab(tabview, "");
    lv_obj_set_style_bg_color(tab_actions, lv_color_hex(0x0A0E27), LV_PART_MAIN);
    lv_obj_set_style_pad_all(tab_actions, 0, LV_PART_MAIN);
    lv_obj_clear_flag(tab_actions, LV_OBJ_FLAG_SCROLLABLE);
    
    const char *action_labels[] = {"Restart", "Export", "Diagnostic", "Backup"};
    const char *action_icons[] = {LV_SYMBOL_REFRESH, LV_SYMBOL_SAVE, LV_SYMBOL_SETTINGS, LV_SYMBOL_UPLOAD};
    
    for (int i = 0; i < 4; i++) {
        lv_obj_t *action_btn = lv_button_create(tab_actions);
        lv_obj_set_size(action_btn, 227, 55);
        lv_obj_set_pos(action_btn, 8 + (i % 2) * 237, 5 + (i / 2) * 65);
        lv_obj_set_style_bg_color(action_btn, lv_color_hex(0x2196F3), LV_PART_MAIN);
        lv_obj_set_style_radius(action_btn, 10, LV_PART_MAIN);
        lv_obj_set_style_shadow_width(action_btn, 0, LV_PART_MAIN);
        lv_obj_set_style_pad_all(action_btn, 5, LV_PART_MAIN);
        lv_obj_add_event_cb(action_btn, action_btn_handler, LV_EVENT_ALL, (void*)action_labels[i]);
        
        lv_obj_t *icon = lv_label_create(action_btn);
        lv_label_set_text(icon, action_icons[i]);
        lv_obj_set_pos(icon, 10, 15);
        lv_obj_set_style_text_color(icon, lv_color_hex(0xFFFFFF), 0);
        
        lv_obj_t *label = lv_label_create(action_btn);
        lv_label_set_text(label, action_labels[i]);
        lv_obj_set_pos(label, 45, 15);
        lv_obj_set_style_text_color(label, lv_color_hex(0xFFFFFF), 0);
        lv_label_set_long_mode(label, LV_LABEL_LONG_CLIP);
    }
    
    /* ==================== BOTTOM NAVIGATION BAR ==================== */
    lv_obj_t *bottom_nav = lv_obj_create(scr);
    lv_obj_set_size(bottom_nav, HOR_RES, 50);
    lv_obj_set_pos(bottom_nav, 0, VER_RES - 50);
    lv_obj_set_style_bg_color(bottom_nav, lv_color_hex(0x1A1F3A), LV_PART_MAIN);
    lv_obj_set_style_border_width(bottom_nav, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(bottom_nav, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(bottom_nav, 0, LV_PART_MAIN);
    lv_obj_clear_flag(bottom_nav, LV_OBJ_FLAG_SCROLLABLE);
    
    const char *nav_icons[] = {LV_SYMBOL_HOME, LV_SYMBOL_SETTINGS, LV_SYMBOL_CALL};
    const char *nav_labels[] = {"Home", "Control", "Action"};
    
    for (int i = 0; i < 3; i++) {
        nav_buttons[i] = lv_button_create(bottom_nav);
        lv_obj_set_size(nav_buttons[i], 150, 42);
        lv_obj_set_pos(nav_buttons[i], 10 + i * 157, 4);
        lv_obj_set_style_bg_color(nav_buttons[i], i == 0 ? lv_color_hex(0x667EEA) : lv_color_hex(0x2A2F4A), LV_PART_MAIN);
        lv_obj_set_style_radius(nav_buttons[i], 8, LV_PART_MAIN);
        lv_obj_set_style_shadow_width(nav_buttons[i], 0, LV_PART_MAIN);
        lv_obj_set_style_pad_all(nav_buttons[i], 2, LV_PART_MAIN);
        lv_obj_set_user_data(nav_buttons[i], (void*)(uintptr_t)i);
        lv_obj_add_event_cb(nav_buttons[i], nav_btn_handler, LV_EVENT_CLICKED, tabview);
        
        lv_obj_t *icon = lv_label_create(nav_buttons[i]);
        lv_label_set_text(icon, nav_icons[i]);
        lv_obj_set_pos(icon, 8, 10);
        lv_obj_set_style_text_color(icon, lv_color_hex(0xFFFFFF), 0);
        
        lv_obj_t *label = lv_label_create(nav_buttons[i]);
        lv_label_set_text(label, nav_labels[i]);
        lv_obj_set_pos(label, 38, 10);
        lv_obj_set_style_text_color(label, lv_color_hex(0xFFFFFF), 0);
        lv_label_set_long_mode(label, LV_LABEL_LONG_CLIP);
    }
    
    /* ==================== BRIGHTNESS OVERLAY (on top of everything) ==================== */
    screen_overlay = lv_obj_create(scr);
    lv_obj_set_size(screen_overlay, HOR_RES, VER_RES);
    lv_obj_set_pos(screen_overlay, 0, 0);
    lv_obj_set_style_bg_color(screen_overlay, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_set_style_border_width(screen_overlay, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(screen_overlay, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(screen_overlay, 0, LV_PART_MAIN);
    lv_obj_set_style_opa(screen_overlay, LV_OPA_40, LV_PART_MAIN); // Start at 80% brightness (40 opacity)
    lv_obj_clear_flag(screen_overlay, LV_OBJ_FLAG_CLICKABLE); // Pass clicks through
    lv_obj_clear_flag(screen_overlay, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_move_foreground(screen_overlay); // Ensure it's on top
    
    ESP_LOGI(TAG, "Professional IoT Control UI created successfully");
}

/* -------------------------------------------------------------------
 * LVGL handler task (will be pinned to core 1)
 * -------------------------------------------------------------------*/
static void lvgl_task(void *arg)
{
    /* Create UI after LVGL task starts to avoid blocking CPU 0 */
    vTaskDelay(pdMS_TO_TICKS(100)); // Allow system to stabilize
    
    create_demo_ui();
    
    uint32_t update_counter = 0;
    float temp_value = 24.5f;
    float hum_value = 65.0f;
    int pressure_value = 1013;
    int cpu_load = 45;
    int mem_load = 67;
    
    while (1) {
        lv_timer_handler();          // do LVGL work
        
        /* Update sensor data every 2 seconds (200 cycles * 10ms) */
        if (update_counter % 200 == 0 && temp_label && humidity_label && pressure_label) {
            // Simulate realistic sensor fluctuations
            temp_value += (esp_random() % 10 - 5) * 0.1f; // ±0.5°C
            if (temp_value < 20.0f) temp_value = 20.0f;
            if (temp_value > 30.0f) temp_value = 30.0f;
            
            hum_value += (esp_random() % 10 - 5) * 0.5f; // ±2.5%
            if (hum_value < 40.0f) hum_value = 40.0f;
            if (hum_value > 80.0f) hum_value = 80.0f;
            
            pressure_value += (esp_random() % 10 - 5); // ±5 hPa
            if (pressure_value < 1000) pressure_value = 1000;
            if (pressure_value > 1025) pressure_value = 1025;
            
            char temp_str[16], hum_str[16], press_str[16];
            snprintf(temp_str, sizeof(temp_str), "%.1fC", temp_value);
            snprintf(hum_str, sizeof(hum_str), "%.0f%%", hum_value);
            snprintf(press_str, sizeof(press_str), "%d", pressure_value);
            
            lv_label_set_text(temp_label, temp_str);
            lv_label_set_text(humidity_label, hum_str);
            lv_label_set_text(pressure_label, press_str);
        }
        
        /* Update performance metrics every 1.5 seconds (150 cycles * 10ms) */
        if (update_counter % 150 == 50 && cpu_arc && mem_arc) {
            // Simulate system load changes
            cpu_load += (esp_random() % 20 - 10); // ±10%
            if (cpu_load < 10) cpu_load = 10;
            if (cpu_load > 90) cpu_load = 90;
            
            mem_load += (esp_random() % 10 - 5); // ±5%
            if (mem_load < 30) mem_load = 30;
            if (mem_load > 85) mem_load = 85;
            
            lv_arc_set_value(cpu_arc, cpu_load);
            lv_arc_set_value(mem_arc, mem_load);
            
            // Update arc labels
            lv_obj_t *cpu_percent = lv_obj_get_child(cpu_arc, 0);
            lv_obj_t *mem_percent = lv_obj_get_child(mem_arc, 0);
            if (cpu_percent) {
                char cpu_str[8];
                snprintf(cpu_str, sizeof(cpu_str), "%d%%", cpu_load);
                lv_label_set_text(cpu_percent, cpu_str);
            }
            if (mem_percent) {
                char mem_str[8];
                snprintf(mem_str, sizeof(mem_str), "%d%%", mem_load);
                lv_label_set_text(mem_percent, mem_str);
            }
        }
        
        /* Status indicator blink every 5 seconds */
        if (update_counter % 500 == 0 && status_dot) {
            static bool blink_state = true;
            lv_obj_set_style_bg_color(status_dot, 
                blink_state ? lv_color_hex(0x00FF88) : lv_color_hex(0x00CC66), 
                LV_PART_MAIN);
            blink_state = !blink_state;
        }
        
        update_counter++;
        vTaskDelay(pdMS_TO_TICKS(10)); // yield to OS frequently (10 ms)
    }
}

/* -------------------------------------------------------------------
 * MAIN
 * -------------------------------------------------------------------*/
void app_main(void)
{
    ESP_LOGI(TAG, "Starting LVGL9 + ILI9488 (pinned handler)");

    /* Configure DC + RST pins */
    gpio_config_t io = {
        .pin_bit_mask = (1ULL<<PIN_TFT_DC) | (1ULL<<PIN_TFT_RST),
        .mode = GPIO_MODE_OUTPUT
    };
    gpio_config(&io);
    gpio_set_level(PIN_TFT_DC, 1);
    gpio_set_level(PIN_TFT_RST, 1);
    
    /* Initialize I2C for touch controller */
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C master initialized for touch controller");

    /* SPI init with max_transfer_sz = MAX_SPI_TRANS_BYTES */
    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_TFT_MOSI,
        .miso_io_num = PIN_TFT_MISO,
        .sclk_io_num = PIN_TFT_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = MAX_SPI_TRANS_BYTES
    };
    esp_err_t err = spi_bus_initialize(HSPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_initialize failed: %s", esp_err_to_name(err));
        return;
    }

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 40 * 1000 * 1000,  // 40MHz for faster rendering
        .mode = 0,
        .spics_io_num = PIN_TFT_CS,
        .queue_size = 7,  // Larger queue for better throughput with big buffers
        .flags = SPI_DEVICE_NO_DUMMY,  // Optimize SPI transfer
    };
    ESP_ERROR_CHECK(spi_bus_add_device(HSPI_HOST, &devcfg, &tft_spi));

    /* LCD init */
    ili9488_init();

    /* Touch controller init (FT6206) */
    touch_reset();
    touch_interrupt_init();
    ESP_ERROR_CHECK(ft6206_init(TOUCH_THRESHOLD));
    ESP_LOGI(TAG, "FT6206 touch controller initialized");

    /* LVGL init + tick */
    lv_init();
    const esp_timer_create_args_t tick_args = {
        .callback = tick_inc,
        .name = "lv_tick"
    };
    esp_timer_handle_t tmr;
    esp_timer_create(&tick_args, &tmr);
    esp_timer_start_periodic(tmr, 1000);

    /* Allocate DMA buffer for RGB666 */
    rgb666_buf = heap_caps_malloc(RGB666_BUF_SIZE, MALLOC_CAP_DMA);
    if (!rgb666_buf) {
        ESP_LOGE(TAG, "Failed to alloc rgb666_buf (%u bytes). Reduce LINES.", (unsigned)RGB666_BUF_SIZE);
        return;
    }
    ESP_LOGI(TAG, "Allocated rgb666_buf %u bytes", (unsigned)RGB666_BUF_SIZE);
    
    /* Allocate LVGL buffers dynamically for double buffering */
    // number of pixels in one buffer (e.g., WIDTH * 38)
    #define LV_BUF_PIXELS (HOR_RES * LINES)

    // size in bytes
    size_t lvgl_buf_size_bytes = LV_BUF_PIXELS * sizeof(lv_color_t);

    // allocate DMA-capable memory +8 bytes for LVGL palette reserve
    lv_buf1 = heap_caps_malloc(lvgl_buf_size_bytes + 8, MALLOC_CAP_DMA);
    lv_buf2 = heap_caps_malloc(lvgl_buf_size_bytes + 8, MALLOC_CAP_DMA);

    if (!lv_buf1 || !lv_buf2) {
        ESP_LOGE(TAG, "Failed to allocate LVGL buffers");
        if (lv_buf1) free(lv_buf1);
        if (lv_buf2) free(lv_buf2);
        return;
    }
    ESP_LOGI(TAG, "Allocated LVGL double buffers: 2 × %u bytes (+8 each) = %u bytes total",
             (unsigned)lvgl_buf_size_bytes, (unsigned)(lvgl_buf_size_bytes * 2 + 16));

    /* Initialize LVGL draw buffers */
    /* Initialize LVGL draw buffers (skip first 8 bytes reserved by LVGL) */
    static lv_draw_buf_t draw_buf1, draw_buf2;

    /* LVGL reserves 8 bytes at start of each buffer for palette/metadata.
    We allocated +8 bytes above, so pass buffer pointer +8 bytes here. */
    lv_color_t *lv_buf1_aligned = (lv_color_t *)(((uint8_t *)lv_buf1) + 8);
    lv_color_t *lv_buf2_aligned = (lv_color_t *)(((uint8_t *)lv_buf2) + 8);

    lv_draw_buf_init(&draw_buf1, HOR_RES, LINES, LV_COLOR_FORMAT_RGB565, LV_STRIDE_AUTO, lv_buf1_aligned, lvgl_buf_size_bytes);
    lv_draw_buf_init(&draw_buf2, HOR_RES, LINES, LV_COLOR_FORMAT_RGB565, LV_STRIDE_AUTO, lv_buf2_aligned, lvgl_buf_size_bytes);

    /* Create LVGL display (v9) */
    lv_display_t *disp = lv_display_create(HOR_RES, VER_RES);
    if (!disp) {
        ESP_LOGE(TAG, "lv_display_create failed");
        return;
    }

    lv_display_set_flush_cb(disp, ili9488_flush);
    // Double buffering: use both buffers for tear-free rendering
    lv_display_set_draw_buffers(disp, &draw_buf1, &draw_buf2);
    
    /* Register touch input device with LVGL */
    lv_indev_t *indev = lv_indev_create();
    if (!indev) {
        ESP_LOGE(TAG, "lv_indev_create failed");
        return;
    }
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(indev, touch_read_cb);
    ESP_LOGI(TAG, "Touch input device registered");

    /* Create LVGL handler task pinned to core 1 to avoid starving IDLE0 */
    const int LVGL_TASK_STACK = 8192; // increase if you still see issues
    BaseType_t r = xTaskCreatePinnedToCore(lvgl_task, "lvgl", LVGL_TASK_STACK, NULL, 5, NULL, 1);
    if (r != pdPASS) {
        ESP_LOGE(TAG, "Failed to start lvgl task (err %d)", r);
        return;
    }

    ESP_LOGI(TAG, "Init complete! FT6206 touch-enabled LVGL9 running on core 1.");
    ESP_LOGI(TAG, "Touch pins: SDA=%d, SCL=%d, IRQ=%d, RST=%d | Display pins: CS=%d, DC=%d, RST=%d",
             I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO, PIN_TOUCH_IRQ, PIN_TOUCH_RST, PIN_TFT_CS, PIN_TFT_DC, PIN_TFT_RST);
}
