#pragma once
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// Set to 1 to enable verbose HID report logging on BLE and USB paths.
#define DONGLE_DEBUG 0

// ble_to_usb: 20-byte Xbox 360 input reports translated from Stadia BLE notifications
// usb_to_ble: 4-byte Stadia rumble payloads from Xbox 360 rumble output reports
extern QueueHandle_t ble_to_usb_queue;
extern QueueHandle_t usb_to_ble_queue;

void bridge_init(void);
void stadia_to_xbox360(const uint8_t *stadia, uint8_t *xbox);
