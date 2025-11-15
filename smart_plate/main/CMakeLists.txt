idf_component_register(SRCS "smart_plate_main.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_http_client esp-tls nvs_flash esp_wifi esp_netif esp_event esp_timer
                    PRIV_REQUIRES driver esp_driver_gpio esp_driver_i2c esp_driver_spi)



