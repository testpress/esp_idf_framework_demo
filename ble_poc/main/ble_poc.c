/*
 * Minimal BLE GATT server (ESP-IDF Bluedroid stack)
 * - Exposes one service with two characteristics:
 *   - Read/Write char (can be read and written by client)
 *   - Notify char  (client can subscribe to notifications)
 *
 * Build with standard ESP-IDF for esp32 (no NimBLE required).
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatt_common_api.h"
#include "esp_gatts_api.h"

static const char *TAG = "BLE_PERIPHERAL";

/* UUIDs (16-bit for simplicity) */
#define GATTS_SERVICE_UUID 0x00FF
#define GATTS_CHAR_UUID_READ_WRITE 0x00EE
#define GATTS_CHAR_UUID_NOTIFY 0x00DD

static const uint16_t GATTS_SERVICE_UUID_VAL = GATTS_SERVICE_UUID;
static const uint16_t GATTS_CHAR_UUID_READ_WRITE_VAL =
    GATTS_CHAR_UUID_READ_WRITE;
static const uint16_t GATTS_CHAR_UUID_NOTIFY_VAL = GATTS_CHAR_UUID_NOTIFY;

/* GATT profile */
#define PROFILE_APP_ID 0

/* attribute table indexes */
enum {
  IDX_SVC,

  IDX_CHAR_RW_DECL,
  IDX_CHAR_RW_VALUE,
  IDX_CHAR_RW_CCC, // Client characteristic configuration (for notifications not
                   // required for RW)

  IDX_CHAR_NOTIFY_DECL,
  IDX_CHAR_NOTIFY_VALUE,
  IDX_CHAR_NOTIFY_CCC,

  IDX_NB
};

/* Initial value for read/write char */
static uint8_t char_rw_value[10] = {0x01, 0x02, 0x03};
static uint16_t notify_conn_id = 0;
static esp_gatt_if_t gatts_if_for_notify = 0;
static uint16_t service_handle = 0;

/* Attribute table */
static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t char_decl_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t char_client_config_uuid =
    ESP_GATT_UUID_CHAR_CLIENT_CONFIG;

/* Full attribute table with values and permissions */
static esp_gatts_attr_db_t gatt_db[IDX_NB] = {
    /* Service Declaration */
    [IDX_SVC] = {{ESP_GATT_AUTO_RSP},
                 {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid,
                  ESP_GATT_PERM_READ, sizeof(uint16_t),
                  sizeof(GATTS_SERVICE_UUID_VAL),
                  (uint8_t *)&GATTS_SERVICE_UUID_VAL}},

    /* Characteristic: Read/Write - Declaration */
    [IDX_CHAR_RW_DECL] = {{ESP_GATT_AUTO_RSP},
                          {ESP_UUID_LEN_16, (uint8_t *)&char_decl_uuid,
                           ESP_GATT_PERM_READ, sizeof(uint8_t), sizeof(uint8_t),
                           (uint8_t[]){ESP_GATT_CHAR_PROP_BIT_READ |
                                       ESP_GATT_CHAR_PROP_BIT_WRITE}}},

    /* Characteristic: Read/Write - Value */
    [IDX_CHAR_RW_VALUE] = {{ESP_GATT_AUTO_RSP},
                           {ESP_UUID_LEN_16,
                            (uint8_t *)&GATTS_CHAR_UUID_READ_WRITE_VAL,
                            ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                            sizeof(char_rw_value), sizeof(char_rw_value),
                            (uint8_t *)char_rw_value}},

    /* CCC (not strictly necessary for RW, but reserved) */
    [IDX_CHAR_RW_CCC] = {{ESP_GATT_AUTO_RSP},
                         {ESP_UUID_LEN_16, (uint8_t *)&char_client_config_uuid,
                          ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                          sizeof(uint16_t), 0, NULL}},

    /* Characteristic: Notify - Declaration */
    [IDX_CHAR_NOTIFY_DECL] = {{ESP_GATT_AUTO_RSP},
                              {ESP_UUID_LEN_16, (uint8_t *)&char_decl_uuid,
                               ESP_GATT_PERM_READ, sizeof(uint8_t),
                               sizeof(uint8_t),
                               (uint8_t[]){ESP_GATT_CHAR_PROP_BIT_READ |
                                           ESP_GATT_CHAR_PROP_BIT_NOTIFY}}},

    /* Characteristic: Notify - Value */
    [IDX_CHAR_NOTIFY_VALUE] = {{ESP_GATT_AUTO_RSP},
                               {ESP_UUID_LEN_16,
                                (uint8_t *)&GATTS_CHAR_UUID_NOTIFY_VAL,
                                ESP_GATT_PERM_READ, 20, 0, NULL}},

    /* Client Characteristic Config for notify (CCC) */
    [IDX_CHAR_NOTIFY_CCC] = {{ESP_GATT_AUTO_RSP},
                             {ESP_UUID_LEN_16,
                              (uint8_t *)&char_client_config_uuid,
                              ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                              sizeof(uint16_t), 0, NULL}},
};

/* advertising data */
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x20,
    .max_interval = 0x40,
    .appearance = 0x00,
    .manufacturer_len = 0, // manufacturer data length
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = 2,
    .p_service_uuid = (uint8_t[]){0xFF, 0x00}, // little endian for 0x00FF
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static void gap_event_handler(esp_gap_ble_cb_event_t event,
                              esp_ble_gap_cb_param_t *param) {
  switch (event) {
  case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
    ESP_LOGI(TAG, "Advertising data set, start advertising now");
    esp_ble_gap_start_advertising(&adv_params);
    break;
  case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
    if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
      ESP_LOGE(TAG, "Failed to start advertising");
    } else {
      ESP_LOGI(TAG, "Advertising started");
    }
    break;
  case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
    ESP_LOGI(TAG, "Advertising stopped");
    break;
  default:
    break;
  }
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event,
                                        esp_gatt_if_t gatts_if,
                                        esp_ble_gatts_cb_param_t *param) {
  switch (event) {
  case ESP_GATTS_REG_EVT: {
    ESP_LOGI(TAG, "GATTS_REG_EVT, app_id=%d", param->reg.app_id);
    /* create the service using the attribute table */
    esp_err_t err = esp_ble_gatts_create_attr_tab(gatt_db, gatts_if, IDX_NB, 0);
    if (err) {
      ESP_LOGE(TAG, "create attr table failed, error code = %x", err);
    }
    break;
  }

  case ESP_GATTS_CREAT_ATTR_TAB_EVT: {
    if (param->add_attr_tab.status != ESP_GATT_OK) {
      ESP_LOGE(TAG, "Create attribute table failed, status %d",
               param->add_attr_tab.status);
      break;
    }
    if (param->add_attr_tab.num_handle != IDX_NB) {
      ESP_LOGE(
          TAG,
          "Create attribute table abnormally, num_handle (%d) != IDX_NB(%d)",
          param->add_attr_tab.num_handle, IDX_NB);
      break;
    }
    /* save service handle and start service */
    service_handle = param->add_attr_tab.handles[IDX_SVC];
    ESP_LOGI(TAG, "Service handle: %d", service_handle);

    /* set values for characteristic value handles */
    // Value handles assigned in add_attr_tab.handles array
    esp_ble_gatts_start_service(service_handle);

    // store handles for later use if needed
    // For this example, no extra storage needed beyond add_attr_tab.handles

    break;
  }

  case ESP_GATTS_READ_EVT: {
    ESP_LOGI(TAG, "GATTS_READ_EVT, conn_id=%d, trans_id=%d, handle=%d",
             param->read.conn_id, param->read.trans_id, param->read.handle);

    // If reading our read/write characteristic (match handle)
    // Here we simply respond with char_rw_value
    esp_gatt_rsp_t rsp;
    memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
    rsp.attr_value.len = sizeof(char_rw_value);
    memcpy(rsp.attr_value.value, char_rw_value, sizeof(char_rw_value));
    esp_ble_gatts_send_response(gatts_if, param->read.conn_id,
                                param->read.trans_id, ESP_GATT_OK, &rsp);
    break;
  }

  case ESP_GATTS_WRITE_EVT: {
    ESP_LOGI(TAG, "GATTS_WRITE_EVT, conn_id=%d, trans_id=%d, handle=%d, len=%d",
             param->write.conn_id, param->write.trans_id, param->write.handle,
             param->write.len);

    // Accept writes to the RW characteristic: copy into char_rw_value (prevent
    // overflow)
    int len = param->write.len;
    if (len > 0 && len <= (int)sizeof(char_rw_value)) {
      memcpy(char_rw_value, param->write.value, len);
      ESP_LOGI(TAG, "New RW char value: %02X %02X %02X ...", char_rw_value[0],
               char_rw_value[1], char_rw_value[2]);
    }

    // If client wrote to CCCD (subscribe/unsubscribe) for notify char:
    if (param->write.is_prep == false) {
      // send response if needed
      esp_ble_gatts_send_response(gatts_if, param->write.conn_id,
                                  param->write.trans_id, ESP_GATT_OK, NULL);
    }
    break;
  }

  case ESP_GATTS_CONNECT_EVT: {
    ESP_LOGI(TAG, "Client connected, conn_id=%d, if=%d", param->connect.conn_id,
             gatts_if);
    notify_conn_id = param->connect.conn_id;
    gatts_if_for_notify = gatts_if;
    break;
  }

  case ESP_GATTS_DISCONNECT_EVT: {
    ESP_LOGI(TAG, "Client disconnected, reason=%d", param->disconnect.reason);
    // restart advertising
    esp_ble_gap_start_advertising(&adv_params);
    break;
  }

  default:
    break;
  }
}

void app_main(void) {
  esp_err_t ret;

  /* init nvs */
  ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  /* initialize BT controller with default config */
  esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
  ret = esp_bt_controller_init(&bt_cfg);
  if (ret) {
    ESP_LOGE(TAG, "bt controller init failed: %s", esp_err_to_name(ret));
    return;
  }

  ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
  if (ret) {
    ESP_LOGE(TAG, "bt controller enable failed: %s", esp_err_to_name(ret));
    return;
  }

  /* initialize Bluedroid stack */
  ret = esp_bluedroid_init();
  if (ret) {
    ESP_LOGE(TAG, "bluedroid init failed: %s", esp_err_to_name(ret));
    return;
  }

  ret = esp_bluedroid_enable();
  if (ret) {
    ESP_LOGE(TAG, "bluedroid enable failed: %s", esp_err_to_name(ret));
    return;
  }

  /* register GAP and GATT callbacks and create service */
  esp_ble_gap_register_callback(gap_event_handler);
  esp_ble_gatts_register_callback(gatts_profile_event_handler);
  esp_ble_gatts_app_register(PROFILE_APP_ID);

  /* set device name */
  esp_ble_gap_set_device_name("ESP32_BLE_Peripheral");

  /* set advertising data (asynchronous) */
  esp_ble_gap_config_adv_data(&adv_data);

  ESP_LOGI(TAG, "BLE GATT server initialized");
}
