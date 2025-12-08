#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

#include "display_driver.h"
#include "esp_event.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

// ---------------- CONFIGURATION ----------------
// CHANGE THESE BEFORE BUILDING EACH VERSION
#define APP_VERSION 1 // Set to 1 or 2
#define CRASH_TEST                                                             \
  0 // Set to 1 to simulate crash/rollback (only if APP_VERSION 2)

#define OTA_URL "http://192.168.0.23:8000/ota_image.bin"

#if APP_VERSION == 1
#define VERSION_STR "v1.0.0"
#define BG_COLOR COLOR_BLACK
#define TXT_COLOR COLOR_WHITE
#else
#if CRASH_TEST
#define VERSION_STR "v2.0.0-BAD"
#define BG_COLOR COLOR_RED
#define TXT_COLOR COLOR_WHITE
#else
#define VERSION_STR "v2.0.0"
#define BG_COLOR COLOR_BLUE
#define TXT_COLOR COLOR_WHITE
#endif
#endif
// -----------------------------------------------

#define WIFI_CONNECTED_BIT BIT0
static EventGroupHandle_t wifi_event_group;

static const char *TAG = "ota_poc";

static volatile bool ota_running = false;

// ------------ WIFI EVENT HANDLER ------------------
static void wifi_handler(void *arg, esp_event_base_t event_base,
                         int32_t event_id, void *event_data) {
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
    esp_wifi_connect();
    if (!ota_running)
      lcd_draw_string(10, 100, "WiFi Connecting...", TXT_COLOR, BG_COLOR, 2);
  } else if (event_base == WIFI_EVENT &&
             event_id == WIFI_EVENT_STA_DISCONNECTED) {
    esp_wifi_connect();
    ESP_LOGI(TAG, "WiFi disconnected. Retrying...");
    if (!ota_running)
      lcd_draw_string(10, 100, "WiFi Retrying...  ", TXT_COLOR, BG_COLOR, 2);
  } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
    ESP_LOGI(TAG, "Got IP");
    xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    if (!ota_running)
      lcd_draw_string(10, 100, "WiFi Connected!   ", TXT_COLOR, BG_COLOR, 2);
  }
}

// ------------ HTTP EVENT HANDLER (optional) ----------
esp_err_t _http_event_handler(esp_http_client_event_t *evt) { return ESP_OK; }

// ------------ OTA PROCESS -------------------------
void perform_ota_update() {
  ota_running = true;
  ESP_LOGI(TAG, "Starting OTA: %s", OTA_URL);
  lcd_draw_string(10, 140, "Starting OTA...", TXT_COLOR, BG_COLOR, 2);

  esp_http_client_config_t config = {
      .url = OTA_URL,
      .event_handler = _http_event_handler,
      .timeout_ms = 5000,
      .keep_alive_enable = true,
  };

  esp_http_client_handle_t client = esp_http_client_init(&config);
  if (client == NULL) {
    ESP_LOGE(TAG, "Failed to init HTTP client");
    lcd_draw_string(10, 160, "HTTP Init Failed", TXT_COLOR, BG_COLOR, 2);
    ota_running = false;
    return;
  }

  if (esp_http_client_open(client, 0) != ESP_OK) {
    ESP_LOGE(TAG, "Failed to open HTTP connection");
    lcd_draw_string(10, 160, "Conn Failed", TXT_COLOR, BG_COLOR, 2);
    esp_http_client_cleanup(client);
    ota_running = false;
    return;
  }

  int length = esp_http_client_fetch_headers(client);
  if (length <= 0) {
    ESP_LOGW(TAG, "Content-Length missing, continuing...");
  }

  const esp_partition_t *update_partition =
      esp_ota_get_next_update_partition(NULL);
  ESP_LOGI(TAG, "Writing to partition offset: 0x%08X",
           update_partition->address);

  esp_ota_handle_t ota_handle;
  ESP_ERROR_CHECK(esp_ota_begin(update_partition, length, &ota_handle));

  // Allocate buffer in internal memory to avoid PSRAM/Cache issues during Flash
  // Write
  char *buffer = heap_caps_malloc(1024, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (buffer == NULL) {
    ESP_LOGE(TAG, "No memory for buffer");
    ota_running = false;
    return;
  }

  int data_read;
  int total_read = 0;
  while ((data_read = esp_http_client_read(client, buffer, 1024)) > 0) {
    ESP_ERROR_CHECK(esp_ota_write(ota_handle, buffer, data_read));
    total_read += data_read;
    if (total_read % 10240 == 0) {
      ESP_LOGI(TAG, "Downloaded %d bytes", total_read);
    }
  }
  free(buffer);

  ESP_ERROR_CHECK(esp_ota_end(ota_handle));

  ESP_ERROR_CHECK(esp_ota_set_boot_partition(update_partition));
  ESP_LOGI(TAG, "OTA done. Rebooting...");
  lcd_draw_string(10, 160, "OTA Done. Rebooting...", TXT_COLOR, BG_COLOR, 2);

  esp_http_client_cleanup(client);

  vTaskDelay(pdMS_TO_TICKS(1000));
  esp_restart();
}

// ------------ MAIN START -------------------------
void app_main(void) {
  // Initialize NVS
  ESP_ERROR_CHECK(nvs_flash_init());

  // Initialize Display
  lcd_init();
  lcd_clear(BG_COLOR);
  lcd_draw_string(10, 10, "OTA Demo", TXT_COLOR, BG_COLOR, 3);
  char buf[32];
  snprintf(buf, sizeof(buf), "Ver: %s", VERSION_STR);
  lcd_draw_string(10, 60, buf, TXT_COLOR, BG_COLOR, 2);

  // Rollback Check / Verification
  const esp_partition_t *running = esp_ota_get_running_partition();
  esp_ota_img_states_t ota_state;
  esp_err_t err = esp_ota_get_state_partition(running, &ota_state);

  ESP_LOGI(TAG, "Running partition: address 0x%08X",
           running ? running->address : 0);
  ESP_LOGI(TAG, "OTA State Check: err=0x%x, state=%d", err, ota_state);

  if (err == ESP_OK) {
    if (ota_state == ESP_OTA_IMG_PENDING_VERIFY) {
#if CRASH_TEST
      ESP_LOGW(TAG, "Pending verify... Simulating CRASH in 5s!");
      lcd_draw_string(10, 200, "BAD FIRMWARE!", COLOR_YELLOW, BG_COLOR, 2);
      lcd_draw_string(10, 230, "CRASH IN 5...", COLOR_YELLOW, BG_COLOR, 2);

      vTaskDelay(pdMS_TO_TICKS(1000));
      lcd_draw_string(10, 230, "CRASH IN 4...", COLOR_YELLOW, BG_COLOR, 2);
      vTaskDelay(pdMS_TO_TICKS(1000));
      lcd_draw_string(10, 230, "CRASH IN 3...", COLOR_YELLOW, BG_COLOR, 2);
      vTaskDelay(pdMS_TO_TICKS(1000));
      lcd_draw_string(10, 230, "CRASH IN 2...", COLOR_YELLOW, BG_COLOR, 2);
      vTaskDelay(pdMS_TO_TICKS(1000));
      lcd_draw_string(10, 230, "CRASH IN 1...", COLOR_YELLOW, BG_COLOR, 2);

      // Mark invalid and reboot to rollback
      // Note: In real life a WDT reset would do this if we don't confirm valid
      // But we force it here to demonstrate rollback
      esp_ota_mark_app_invalid_rollback_and_reboot();
      return;
#else
      ESP_LOGI(TAG, "Pending verify... Marking VALID!");
      lcd_draw_string(10, 200, "Verifying...", TXT_COLOR, BG_COLOR, 2);
      esp_ota_mark_app_valid_cancel_rollback();
      lcd_draw_string(10, 230, "App Marked VALID", TXT_COLOR, BG_COLOR, 2);
#endif
    } else {
      ESP_LOGI(TAG, "OTA State is not PENDING_VERIFY (state=%d)", ota_state);
    }
  } else if (err == ESP_ERR_NOT_SUPPORTED) {
    ESP_LOGW(TAG, "Running from Factory or non-OTA partition (No OTA State)");
  } else {
    ESP_LOGE(TAG, "Failed to get OTA state: %s", esp_err_to_name(err));
  }

  // Initialize WiFi
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_create_default_wifi_sta();

  wifi_event_group = xEventGroupCreate();
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_handler, NULL, NULL));
  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_handler, NULL, NULL));

  wifi_config_t wifi_cfg = {
      .sta =
          {
              .ssid = "Testpress_4G",
              .password = "Tp@12345",
          },
  };

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
  ESP_ERROR_CHECK(esp_wifi_start());

// Only start OTA if we are V1. If we are V2, we just stay there (unless we want
// to downgrade or cycle)
#if APP_VERSION == 1
  ESP_LOGI(TAG, "Waiting for Wi-Fi...");
  xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, false, true,
                      portMAX_DELAY);

  ESP_LOGI(TAG, "Connected! Starting OTA in 5 seconds...");
  lcd_draw_string(10, 140, "OTA in 5 sec...", TXT_COLOR, BG_COLOR, 2);
  vTaskDelay(pdMS_TO_TICKS(5000));

  perform_ota_update();
#endif

  // Main loop
  int count = 0;
  char s[32];
  while (1) {
    snprintf(s, sizeof(s), "Count: %d", count++);
    lcd_draw_string(10, 280, s, TXT_COLOR, BG_COLOR, 2);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
