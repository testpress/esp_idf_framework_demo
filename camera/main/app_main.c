#include "ArducamCamera.h"
#include "ArducamLink.h"
#include "delay.h"
#include "driver/gpio.h"
#include "esp_event.h"
#include "esp_http_client.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "uart.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TAG "camera_server"

// ---- WiFi config (change to your AP) ----
#define WIFI_SSID "Testpress_4G"
#define WIFI_PASS "Tp@12345"
#define MAXIMUM_RETRY 5

// ---- HTTP upload config (change URL/token) ----
// Example: LAN URL (recommended while developing): http://192.168.1.42:8000/api/v1/meal_logs/
// Example: ngrok HTTPS URL: https://<your-ngrok-id>.ngrok-free.app/api/v1/meal_logs/
static const char *UPLOAD_URL = "http://192.168.0.4:8000/api/v1/meal_logs/"; // <-- CHANGE THIS
static const char *AUTH_HEADER_VALUE = "Bearer iKnCz7E2Mtfl_V0bDcasJWDMzVN39L_BCySvj1hLDSc";

// If you plan to use HTTPS (ngrok), you'll likely need the server CA certificate compiled in and set:
// static const char server_root_cert_pem[] = "-----BEGIN CERTIFICATE-----\n....\n-----END CERTIFICATE-----\n";
// Then set .cert_pem = server_root_cert_pem in esp_http_client_config_t
// (I left it out to avoid accidentally breaking TLS validation.)

// ---- Camera globals ----
ArducamCamera myCAM;
const int CS_PIN = 33; // chip select
static httpd_handle_t server = NULL;

// WiFi event group
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static int s_retry_num = 0;

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

/* ---------- Helper: POST multipart/form-data ---------- */

/*
  send_image_to_server()
  - Builds a multipart/form-data body in PSRAM
  - Fields hardcoded to match your curl example (timestamp, weight_g, user_id)
  - Posts the JPEG as field name image_file, filename capture.jpg
*/
static esp_err_t send_image_to_server(const uint8_t *jpeg, size_t jpeg_len)
{
    const char *url = UPLOAD_URL; // change at top of file
    const char *auth_header_value = AUTH_HEADER_VALUE;

    const char *timestamp_value = "2024-01-15T12:30:00Z";
    const char *weight_value = "10";
    const char *user_id_value = "11";
    const char *file_field_name = "image_file";
    const char *file_name = "capture.jpg";
    const char *file_mime = "image/jpeg";
    const char *boundary = "----ESP32FormBoundary7MA4YWxkTrZu0gW"; // chosen safe boundary

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
        // If you plan to use HTTPS and have a cert compiled in:
        // .cert_pem = server_root_cert_pem,
        // optionally set .transport_type depending on IDF version
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

        // Optionally, read response body to buffer here if you want to parse it
        // (use esp_http_client_read_response if desired)
    } else {
        ESP_LOGE(TAG, "HTTP POST failed: %d", err);
    }

    esp_http_client_cleanup(client);
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
    // Return empty response for favicon.ico to prevent 404 errors
    httpd_resp_set_type(req, "image/x-icon");
    return httpd_resp_send(req, "", 0);
}

/*
  capture_handler:
  - Takes a picture (VGA JPEG),
  - Reads image into PSRAM,
  - Sends it to your backend via send_image_to_server(),
  - Responds to browser with the JPEG inline (so /capture still works)
*/
static esp_err_t capture_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "Capture request received");

    // Trigger capture (VGA JPEG)
    CamStatus status = takePicture(&myCAM, CAM_IMAGE_MODE_VGA, CAM_IMAGE_PIX_FMT_JPG);
    if (status != CAM_ERR_SUCCESS) {
        ESP_LOGE(TAG, "Failed to take picture, status: %d", status);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    // Wait until imageAvailable() reports a length or timeout
    const TickType_t wait_tick = pdMS_TO_TICKS(50);
    const int max_wait_cycles = 40; // 40 * 50ms = 2s total wait
    int wait_count = 0;
    uint32_t image_length = 0;

    ESP_LOGI(TAG, "Waiting for image to be ready...");
    while (wait_count < max_wait_cycles) {
        captureThread(&myCAM); // drive camera state machine
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

    // Allocate PSRAM buffer for image
    uint8_t *image_data = heap_caps_malloc(image_length, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!image_data) {
        ESP_LOGE(TAG, "Failed to allocate PSRAM for image!");
        httpd_resp_send_500(req);
        return ESP_ERR_NO_MEM;
    }

    // Read image in small chunks (<=255) because many readBuff implementations use uint8_t length
    const uint32_t chunk_size = 200; // MUST be <= 255
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

    // Upload to backend (blocking) — you can move this into a worker task if you want non-blocking server responses.
    esp_err_t post_err = send_image_to_server(image_data, image_length);
    if (post_err != ESP_OK) {
        ESP_LOGW(TAG, "send_image_to_server failed: %d", post_err);
        // continue to serve the image to the browser even if upload failed
    } else {
        ESP_LOGI(TAG, "Image uploaded to backend successfully");
    }

    // Serve image to browser
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

/* ---------- Cleanup preview / helper ---------- */
// Keep a simple stop preview function in case other libs call it.
void stop_preview(void)
{
    // You can call camera-specific stop/stream off commands here if needed
    // Keeping minimal implementation
    ESP_LOGI(TAG, "stop_preview() called");
}

/* ---------- app_main ---------- */

void app_main(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // UART for debug
    uartBegin(115200);
    ESP_LOGI(TAG, "ESP32 Camera Server starting...");

    // WiFi
    ESP_LOGI(TAG, "Initializing WiFi...");
    wifi_init_sta();

    // Camera init
    ESP_LOGI(TAG, "Initializing camera...");
    myCAM = createArducamCamera(CS_PIN);
    begin(&myCAM);
    // You can register callbacks if you use stream mode; for capture mode we don't need callback
    // registerCallback(&myCAM, ReadBuffer, 200, stop_preview);

    // Start webserver
    ESP_LOGI(TAG, "Starting web server...");
    server = start_webserver();
    if (server) {
        ESP_LOGI(TAG, "Camera server started successfully!");
        ESP_LOGI(TAG, "Visit http://<ESP32_IP>/");
        ESP_LOGI(TAG, "Visit http://<ESP32_IP>/capture to capture an image");
    } else {
        ESP_LOGE(TAG, "Failed to start web server!");
    }

    // Capture thread loop - keep calling captureThread so camera internal state is maintained
    while (1) {
        captureThread(&myCAM); // ensures internal camera processing; cheap to call frequently
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
