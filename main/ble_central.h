#pragma once
#include <stdint.h>

// Initialise NimBLE host configuration and reconnect timer.
// Must be called before nimble_port_init().
void ble_central_init(void);

// Schedule a GATT Write-With-Response of `payload` (4 bytes) to the Stadia
// rumble output characteristic.  Safe to call from any FreeRTOS task — the
// write is executed via a NimBLE callout on the NimBLE event queue.
void ble_central_send_rumble(const uint8_t *payload);

// FreeRTOS task entry-point for the NimBLE host.
// Pass to nimble_port_freertos_init().
void nimble_host_task(void *param);
