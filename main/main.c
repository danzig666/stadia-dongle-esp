/*
 * main.c — app_main for the Stadia BT dongle.
 *
 * Task layout:
 *   Core 0, pri 5 — nimble_host_task    (NimBLE host stack, via nimble_port_freertos_init)
 *   Core 0, pri 3 — ble_rumble_task     (drain usb_to_ble → ble_central_send_rumble)
 *   Core 1, pri 4 — tinyusb device task (created internally by tinyusb_driver_install)
 *   Core 1, pri 4 — usb_xbox_task       (drain ble_to_usb → xbox_send_report)
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "nvs.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"

#include "bridge.h"
#include "usb_xbox.h"
#include "ble_central.h"
#include "button_actions.h"
#include "config_store.h"
#include "controller_manager.h"
#include "dongle_state.h"
#include "hid_extra.h"
#include "hid_mouse.h"
#include "mouse_mode.h"
#include "status_led.h"
#include "web_server.h"

static const char *TAG = "MAIN";

static void recovery_clear_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(30000));
    nvs_handle_t nvs;
    if (nvs_open("recovery", NVS_READWRITE, &nvs) == ESP_OK) {
        nvs_set_u8(nvs, "boots", 0);
        nvs_commit(nvs);
        nvs_close(nvs);
    }
    vTaskDelete(NULL);
}

static void recovery_check(void)
{
    nvs_handle_t nvs;
    uint8_t boots = 0;
    if (nvs_open("recovery", NVS_READWRITE, &nvs) == ESP_OK) {
        nvs_get_u8(nvs, "boots", &boots);
        if (boots < 255) boots++;
        nvs_set_u8(nvs, "boots", boots);
        nvs_commit(nvs);
        nvs_close(nvs);
    }
    if (boots >= 5) {
        controller_manager_forget_all();
        web_server_start(true);
    } else if (boots >= 3) {
        // web_server_start(true); // Now started unconditionally below
    }
    xTaskCreate(recovery_clear_task, "recovery_clear", 2048, NULL, 1, NULL);
}

/* ---- Rumble relay task (Core 0 — same core as NimBLE) ------------------- */

static void ble_rumble_task(void *arg)
{
    usb_to_ble_msg_t msg;
    while (1) {
        if (xQueueReceive(usb_to_ble_queue, &msg, portMAX_DELAY) == pdTRUE) {
            ble_central_send_rumble(msg.gamepad_index, msg.data, msg.len);
        }
    }
}

/* ---- Entry point --------------------------------------------------------- */

void app_main(void)
{
    dongle_state_init();

    // NVS is required for BLE bonding persistence
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
        ESP_ERROR_CHECK(ret);
    }
    ESP_ERROR_CHECK(ret);
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    config_store_init();
    status_led_init();

    // Create inter-task queues
    bridge_init();
    button_actions_init();

    // USB: install TinyUSB driver (also starts the USB device task on Core 1)
    hid_extra_init();
    hid_mouse_init();
    mouse_mode_init();
    usb_xbox_init();
    web_server_init();

    // Initialise NimBLE port (controller + host transport) before ble_central_init()
    // so that nimble_port_get_dflt_eventq() returns a valid queue.
    nimble_port_init();

    // BLE: configure NimBLE host (callbacks, pairing params, callouts)
    // controller_manager_init() is called from on_sync() after NimBLE sync.
    ble_central_init();
    recovery_check();

    // Start setup AP at boot; it switches to the configured timeout after BLE connects.
    web_server_start(true);

    // Spawn tasks
    xTaskCreatePinnedToCore(usb_xbox_task,   "usb_xbox",   4096, NULL, 4, NULL, 1);
    xTaskCreatePinnedToCore(ble_rumble_task, "ble_rumble", 4096, NULL, 3, NULL, 0);

    // Start NimBLE host task on Core 0 (priority 5, managed by nimble_port)
    nimble_port_freertos_init(nimble_host_task);

    ESP_LOGI(TAG, "Stadia BT dongle started");
}
