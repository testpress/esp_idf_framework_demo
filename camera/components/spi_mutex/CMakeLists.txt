set(srcs "src/spi_mutex.c")
set(inc_dirs "include")

idf_component_register(
    SRCS ${srcs}
    INCLUDE_DIRS ${inc_dirs}
    REQUIRES esp_driver_spi esp_system freertos
)
