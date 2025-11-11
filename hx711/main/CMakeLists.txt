idf_component_register(SRCS "hx711.c" "hx711.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES esp_driver_gpio esp_timer vfs esp_driver_uart)
