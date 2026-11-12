if(IDF_TARGET STREQUAL "esp32s3")
    set(EPD_DRIVER_SRC "epd_x4pro.c")
else()
    set(EPD_DRIVER_SRC "it8951.c")
endif()

idf_component_register(
    SRCS "display.c" ${EPD_DRIVER_SRC} "silkscreen.c" "surface.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "."
    REQUIRES bus dragonball layout
    PRIV_REQUIRES esp_driver_spi esp_driver_gpio esp_timer
)
