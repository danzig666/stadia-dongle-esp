idf_component_register(
    SRCS "xbox_dev.c" "main.c" "bridge.c" "usb_xbox.c" "ble_central.c"
    INCLUDE_DIRS "."
    REQUIRES nvs_flash bt esp_timer driver
)
