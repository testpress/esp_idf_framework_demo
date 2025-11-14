idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES lvgl
    PRIV_REQUIRES esp_driver_gpio esp_driver_spi esp_driver_i2c esp_timer driver
)
