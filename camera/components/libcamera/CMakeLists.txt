set(srcs        spi.c
                Arducam/ArducamCamera.c
                )

set(inc_dirs    Arducam
                .)

idf_component_register(SRCS ${srcs} INCLUDE_DIRS ${inc_dirs} REQUIRES esp_driver_spi esp_driver_gpio spi_mutex)
