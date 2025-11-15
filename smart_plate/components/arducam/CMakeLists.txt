idf_component_register(SRCS "ArducamCamera.c" "ArducamLink.c"
                    INCLUDE_DIRS "."
                    REQUIRES driver
                    PRIV_REQUIRES esp_driver_spi esp_driver_gpio esp_driver_uart)
