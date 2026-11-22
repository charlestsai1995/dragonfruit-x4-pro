idf_component_register(
    SRCS "main.c" "debug_console.c" "serial_bridge.c" "wifi_ntp.c"
    INCLUDE_DIRS "."
    REQUIRES m68k bus dragonball display input esp_driver_gpio esp_driver_uart esp_driver_usb_serial_jtag esp_adc rom_loader layout config installer nvs_flash esp_wifi esp_netif esp_event lwip
)
