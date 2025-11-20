set(srcs        uart.c
                ArducamLink.c
                )

set(inc_dirs    .)

idf_component_register(SRCS ${srcs} INCLUDE_DIRS ${inc_dirs} REQUIRES libcamera esp_driver_uart PRIV_REQUIRES main)
