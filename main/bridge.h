#pragma once
#include <stdint.h>
#include <stddef.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "dongle_config.h"
#include "dongle_state.h"

// Set to 1 to enable verbose HID report logging on BLE and USB paths.
#define DONGLE_DEBUG 0

// ble_to_usb: 20-byte Xbox 360 input reports translated from Stadia BLE notifications
// usb_to_ble: 4-byte Stadia rumble payloads from Xbox 360 rumble output reports
extern QueueHandle_t ble_to_usb_queue;
extern QueueHandle_t usb_to_ble_queue;

typedef struct {
    uint8_t gamepad_index;
    uint8_t data[DONGLE_XBOX_REPORT_SIZE];
    size_t len;
} ble_to_usb_msg_t;

typedef struct {
    uint8_t gamepad_index;
    uint8_t data[DONGLE_RUMBLE_REPORT_SIZE];
    size_t len;
} usb_to_ble_msg_t;

void bridge_init(void);
void bridge_send_neutral(uint8_t gamepad_index);
void stadia_parse_state(const uint8_t *stadia, size_t len, stadia_controller_state_t *out);
void stadia_to_xbox360(const uint8_t *stadia, uint8_t *xbox);
void stadia_pressed_names(const stadia_controller_state_t *state, char *out, size_t out_len);
