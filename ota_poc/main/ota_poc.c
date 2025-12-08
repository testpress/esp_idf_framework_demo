#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include "esp_netif.h"
#include "nvs_flash.h"

#define OTA_URL "http://192.168.0.23:8000/ota_image.bin"

#define WIFI_CONNECTED_BIT BIT0
static EventGroupHandle_t wifi_event_group;

static const char *TAG = "ota_poc";

// ------------ WIFI EVENT HANDLER ------------------
static void wifi_handler(void *arg, esp_event_base_t event_base,
                         int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        esp_wifi_connect();
        ESP_LOGI(TAG, "WiFi disconnected. Retrying...");
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ESP_LOGI(TAG, "Got IP");
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

// ------------ HTTP EVENT HANDLER (optional) ----------
esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
    return ESP_OK;
}

// ------------ OTA PROCESS -------------------------
void perform_ota_update()
{
    ESP_LOGI(TAG, "Starting OTA: %s", OTA_URL);

    esp_http_client_config_t config = {
        .url = OTA_URL,
        .event_handler = _http_event_handler,
        .timeout_ms = 5000,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == NULL)
    {
        ESP_LOGE(TAG, "Failed to init HTTP client");
        return;
    }

    if (esp_http_client_open(client, 0) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to open HTTP connection");
        esp_http_client_cleanup(client);
        return;
    }

    int length = esp_http_client_fetch_headers(client);
    if (length <= 0)
    {
        ESP_LOGW(TAG, "Content-Length missing, continuing...");
    }

    const esp_partition_t *update_partition = esp_ota_get_next_update_partition(NULL);
    ESP_LOGI(TAG, "Writing to partition offset: 0x%08X", update_partition->address);

    esp_ota_handle_t ota_handle;
    ESP_ERROR_CHECK(esp_ota_begin(update_partition, length, &ota_handle));

    uint8_t buffer[1024];
    int data_read;

    while ((data_read = esp_http_client_read(client, (char *)buffer, sizeof(buffer))) > 0)
    {
        ESP_ERROR_CHECK(esp_ota_write(ota_handle, buffer, data_read));
    }

    ESP_ERROR_CHECK(esp_ota_end(ota_handle));

    ESP_ERROR_CHECK(esp_ota_set_boot_partition(update_partition));
    ESP_LOGI(TAG, "OTA done. Rebooting...");
    esp_http_client_cleanup(client);

    esp_restart();
}

// ------------ MAIN START -------------------------
void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_event_group = xEventGroupCreate();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // register event handlers
    ESP_ERROR_CHECK(
        esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_handler, NULL, NULL));
    ESP_ERROR_CHECK(
        esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_handler, NULL, NULL));

    wifi_config_t wifi_cfg = {
        .sta = {
            .ssid = "Testpress_4G",
            .password = "Tp@12345",
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Waiting for Wi-Fi...");
    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, false, true, portMAX_DELAY);

    ESP_LOGI(TAG, "Connected! Starting OTA in 2 seconds...");
    vTaskDelay(pdMS_TO_TICKS(2000));

    perform_ota_update();
}
