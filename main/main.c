/*
 * main.c — app_main for the Stadia BT dongle.
 *
 * Task layout:
 *   Core 0, pri 5 — nimble_host_task    (NimBLE host stack, via nimble_port_freertos_init)
 *   Core 0, pri 3 — ble_rumble_task     (drain usb_to_ble → ble_central_send_rumble)
 *   Core 1, pri 4 — tinyusb device task (created internally by tinyusb_driver_install)
 *   Core 1, pri 4 — usb_xbox_task       (drain ble_to_usb → tud_vendor_n_write)
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"

#include "bridge.h"
#include "usb_xbox.h"
#include "ble_central.h"

static const char *TAG = "MAIN";

/* ---- Rumble relay task (Core 0 — same core as NimBLE) ------------------- */

static void ble_rumble_task(void *arg)
{
    uint8_t payload[4];
    while (1) {
        if (xQueueReceive(usb_to_ble_queue, payload, portMAX_DELAY) == pdTRUE) {
            ble_central_send_rumble(payload);
        }
    }
}

/* ---- Entry point --------------------------------------------------------- */

void app_main(void)
{
    // NVS is required for BLE bonding persistence
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Create inter-task queues
    bridge_init();

    // USB: install TinyUSB driver (also starts the USB device task on Core 1)
    usb_xbox_init();

    // Initialise NimBLE port (controller + host transport) before ble_central_init()
    // so that nimble_port_get_dflt_eventq() returns a valid queue.
    nimble_port_init();

    // BLE: configure NimBLE host (callbacks, pairing params, callouts)
    ble_central_init();

    // Spawn tasks
    xTaskCreatePinnedToCore(usb_xbox_task,   "usb_xbox",   4096, NULL, 4, NULL, 1);
    xTaskCreatePinnedToCore(ble_rumble_task, "ble_rumble", 4096, NULL, 3, NULL, 0);

    // Start NimBLE host task on Core 0 (priority 5, managed by nimble_port)
    nimble_port_freertos_init(nimble_host_task);

    ESP_LOGI(TAG, "Stadia BT dongle started");
}
