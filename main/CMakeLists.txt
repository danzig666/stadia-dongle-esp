idf_component_register(
    SRCS "xbox_dev.c" "main.c" "bridge.c" "usb_xbox.c" "ble_central.c"
         "dongle_state.c" "status_led.c" "controller_manager.c"
         "config_store.c" "web_server.c" "captive_portal.c"
         "hid_extra.c" "hid_mouse.c" "button_actions.c" "mouse_mode.c"
    INCLUDE_DIRS "."
    REQUIRES nvs_flash bt esp_timer driver esp_driver_rmt esp_wifi esp_netif esp_http_server app_update esp_app_format esp_coex
)
