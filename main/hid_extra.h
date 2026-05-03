#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "device/usbd_pvt.h"

#define HID_EXTRA_REPORT_DESC_LEN 90

void hid_extra_init(void);
bool hid_extra_send_key_press(uint8_t modifier, uint8_t keycode);
bool hid_extra_send_key_release(void);
bool hid_extra_send_consumer_press(uint16_t usage);
bool hid_extra_send_consumer_release(void);
bool hid_extra_remote_wakeup(void);
const uint8_t *hid_extra_report_descriptor(void);
usbd_class_driver_t const *hid_extra_class_driver(void);
