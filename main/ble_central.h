#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Initialise NimBLE host configuration and reconnect timer.
// Must be called before nimble_port_init().
void ble_central_init(void);

// Start or restart Stadia BLE discovery. Safe to call from task context after
// NimBLE has synced.
void ble_central_start_scan(void);

// Temporarily pause BLE discovery while Wi-Fi AP association/DHCP is active.
// Existing BLE controller connections and notifications are not affected.
void ble_central_set_scan_paused(bool paused);

// Schedule a GATT Write-With-Response of `payload` (4 bytes) to the Stadia
// rumble output characteristic.  Safe to call from any FreeRTOS task — the
// write is executed via a NimBLE callout on the NimBLE event queue.
void ble_central_send_rumble(uint8_t gamepad_index, const uint8_t *payload, size_t len);

// FreeRTOS task entry-point for the NimBLE host.
// Pass to nimble_port_freertos_init().
void nimble_host_task(void *param);
