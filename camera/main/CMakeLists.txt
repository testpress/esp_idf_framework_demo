set(srcs        app_main.c)

set(inc_dirs    .)

# set(requires    libcamera)

idf_component_register(SRCS ${srcs} INCLUDE_DIRS ${inc_dirs} REQUIRES libcamera uart esp_driver_gpio esp_wifi esp_http_server esp_http_client nvs_flash esp_timer)
set_source_files_properties(${srcs} PROPERTIES LANGUAGE C)
