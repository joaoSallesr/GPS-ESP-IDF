list(APPEND SOURCES main.c config.c)

list(APPEND REQUIRES esp_driver_uart esp_driver_gpio)

idf_component_register(
    SRCS ${SOURCES}
    INCLUDE_DIRS "."
    REQUIRES ${REQUIRES}
)