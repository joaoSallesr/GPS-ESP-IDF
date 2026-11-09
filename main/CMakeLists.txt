idf_component_register(
    SRCS "main.c" "global.c" 
    INCLUDE_DIRS "."
    REQUIRES esp_driver_uart esp_driver_gpio
)
