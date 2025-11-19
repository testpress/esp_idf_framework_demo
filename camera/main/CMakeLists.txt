set(srcs        app_main.c)

set(inc_dirs    .)

# set(requires    libcamera)
# Force rebuild after LVGL font fixes

idf_component_register(SRCS ${srcs} INCLUDE_DIRS ${inc_dirs} REQUIRES diagnostics libcamera uart esp_wifi esp_http_server esp_http_client nvs_flash lvgl PRIV_REQUIRES esp_driver_gpio esp_driver_i2c esp_timer driver)
set_source_files_properties(${srcs} PROPERTIES LANGUAGE C)
