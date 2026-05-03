#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "device/usbd_pvt.h"

#define HID_MOUSE_REPORT_DESC_LEN 52

void hid_mouse_init(void);
bool hid_mouse_send_report(uint8_t buttons, int8_t x, int8_t y, int8_t wheel);
const uint8_t *hid_mouse_report_descriptor(void);
usbd_class_driver_t const *hid_mouse_class_driver(void);
