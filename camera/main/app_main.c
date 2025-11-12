
#include "ArducamCamera.h"
#include "ArducamLink.h"
#include "delay.h"
#include "driver/gpio.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "uart.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

ArducamCamera myCAM;
const int cs = 33;

uint8_t temp             = 0xff;
uint8_t commandBuff[20]  = {0};
uint8_t commandLength    = 0;
uint8_t sendFlag         = TRUE;
uint32_t readImageLength = 0;
uint8_t jpegHeadFlag     = 0;

// JPEG data buffer for base64 encoding
#define MAX_JPEG_SIZE 100000  // 100KB should be enough for VGA JPEG
uint8_t jpegBuffer[MAX_JPEG_SIZE];
uint32_t jpegBufferIndex = 0;

// Base64 encoding table
const char base64_table[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

// WiFi configuration
#define WIFI_SSID "Testpress_4G"
#define WIFI_PASS "Tp@12345"
#define MAXIMUM_RETRY 5

// HTTP server
static httpd_handle_t server = NULL;

// Event group for WiFi connection
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static const char *TAG = "camera_server";
static int s_retry_num = 0;

static void event_handler(void* arg, esp_event_base_t event_base,
                         int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
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

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returned because either WIFI_CONNECTED_BIT or WIFI_FAIL_BIT was set by the event handler.
     * If the connection is established, WIFI_CONNECTED_BIT is set, otherwise WIFI_FAIL_BIT is set. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 WIFI_SSID, WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 WIFI_SSID, WIFI_PASS);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
}

void base64_encode(const uint8_t* data, size_t input_length, char* output) {
    size_t output_index = 0;

    for (size_t i = 0; i < input_length; i += 3) {
        uint32_t octet_a = i < input_length ? data[i] : 0;
        uint32_t octet_b = i + 1 < input_length ? data[i + 1] : 0;
        uint32_t octet_c = i + 2 < input_length ? data[i + 2] : 0;

        uint32_t triple = (octet_a << 16) | (octet_b << 8) | octet_c;

        output[output_index++] = base64_table[(triple >> 18) & 0x3F];
        output[output_index++] = base64_table[(triple >> 12) & 0x3F];
        output[output_index++] = base64_table[(triple >> 6) & 0x3F];
        output[output_index++] = base64_table[triple & 0x3F];
    }

    // Add padding
    size_t padding_start = ((input_length + 2) / 3) * 4;
    if (input_length % 3 == 1) {
        output[padding_start - 2] = '=';
        output[padding_start - 1] = '=';
    } else if (input_length % 3 == 2) {
        output[padding_start - 1] = '=';
    }

    output[output_index] = '\0';
}

uint8_t ReadBuffer(uint8_t* imagebuf, uint8_t length)
{
    // This function is kept for compatibility but main image processing
    // now happens directly in the main loop
    return sendFlag;
}

static esp_err_t capture_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "Capture request received");

    // Take VGA JPEG picture (resolution 0x02, format JPEG 0x10)
    CamStatus status = takePicture(&myCAM, CAM_IMAGE_MODE_VGA, CAM_IMAGE_PIX_FMT_JPG);

    if (status != CAM_ERR_SUCCESS) {
        ESP_LOGE(TAG, "Failed to take picture, status: %d", status);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    // Read and send the image data directly
    uint32_t image_length = imageAvailable(&myCAM);
    if (image_length == 0 || image_length > MAX_JPEG_SIZE) {
        ESP_LOGE(TAG, "No image data available or image too large: %lu", image_length);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Image size: %lu bytes", image_length);

    // Allocate image buffer in PSRAM to avoid heap fragmentation
    uint8_t* image_data = (uint8_t*)heap_caps_malloc(image_length, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!image_data) {
        ESP_LOGE(TAG, "Failed to allocate PSRAM buffer for image!");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    // Read all image data at once
    uint32_t bytes_read = 0;
    uint32_t chunk_size = 200; // Read in chunks
    uint8_t buffer[200];

    while (bytes_read < image_length) {
        uint32_t remaining = image_length - bytes_read;
        uint32_t read_size = (remaining > chunk_size) ? chunk_size : remaining;

        uint8_t bytes_in_chunk = readBuff(&myCAM, buffer, read_size);
        if (bytes_in_chunk > 0) {
            memcpy(&image_data[bytes_read], buffer, bytes_in_chunk);
            bytes_read += bytes_in_chunk;
        } else {
            ESP_LOGE(TAG, "Failed to read image data chunk");
            heap_caps_free(image_data);
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }
    }

    if (bytes_read != image_length) {
        ESP_LOGE(TAG, "Incomplete image data read: %lu/%lu", bytes_read, image_length);
        heap_caps_free(image_data);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    // Set HTTP response headers
    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=\"capture.jpg\"");
    char content_length_hdr[32];
    sprintf(content_length_hdr, "%lu", image_length);
    httpd_resp_set_hdr(req, "Content-Length", content_length_hdr);

    // Send the image data
    esp_err_t err = httpd_resp_send(req, (const char*)image_data, image_length);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send HTTP response: %d", err);
    }

    // Free the allocated buffer
    heap_caps_free(image_data);

    ESP_LOGI(TAG, "Image sent successfully");
    return err;
}

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

httpd_handle_t start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;

    // Start the httpd server
    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        // Set URI handlers
        ESP_LOGI(TAG, "Registering URI handlers");
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
        return server;
    }

    ESP_LOGI(TAG, "Error starting server!");
    return NULL;
}

void stop_preivew()
{
    readImageLength = 0;
    jpegHeadFlag    = 0;
    jpegBufferIndex = 0;
    uint32_t len    = 9;

    arducamUartWrite(0xff);
    arducamUartWrite(0xBB);
    arducamUartWrite(0xff);
    arducamUartWrite(0xAA);
    arducamUartWrite(0x06);
    arducamUartWriteBuff((uint8_t*)&len, 4);
    printf("streamoff");
    arducamUartWrite(0xff);
    arducamUartWrite(0xBB);
}

void app_main(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize UART for debugging
    uartBegin(115200);

    ESP_LOGI(TAG, "ESP32 Camera Server starting...");

    // Initialize WiFi
    ESP_LOGI(TAG, "Initializing WiFi...");
    wifi_init_sta();

    // Initialize camera
    ESP_LOGI(TAG, "Initializing camera...");
    myCAM = createArducamCamera(cs);
    begin(&myCAM);
    registerCallback(&myCAM, ReadBuffer, 200, stop_preivew);

    // Start web server
    ESP_LOGI(TAG, "Starting web server...");
    server = start_webserver();

    if (server) {
        ESP_LOGI(TAG, "Camera server started successfully!");
        ESP_LOGI(TAG, "Visit http://<ESP32_IP>/ to access the camera");
        ESP_LOGI(TAG, "Visit http://<ESP32_IP>/capture to capture an image");
    } else {
        ESP_LOGE(TAG, "Failed to start web server!");
    }

    // Keep the camera streaming for the callback system
    while (1) {
        captureThread(&myCAM);
        vTaskDelay(20/portTICK_PERIOD_MS);
    }
}
