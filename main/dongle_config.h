#pragma once

#include <stdint.h>

#define DONGLE_MAX_CONTROLLERS 2
#define DONGLE_ENABLE_UTILITY_HID 1
#define DONGLE_ENABLE_MOUSE_HID 1

#define DONGLE_MAX_RAW_REPORT_SIZE 32
#define DONGLE_XBOX_REPORT_SIZE 20
#define DONGLE_RUMBLE_REPORT_SIZE 4

#define DONGLE_MOUSE_ITF_NUM (DONGLE_MAX_CONTROLLERS + 1)
#define DONGLE_MOUSE_EP_IN 0x82
#define DONGLE_MOUSE_EP_SIZE 8

#if DONGLE_MAX_CONTROLLERS < 1
#error "DONGLE_MAX_CONTROLLERS must be at least 1"
#endif

/*
 * ESP32-S3 FS USB endpoint budget is tight. This firmware assigns one IN and
 * one OUT endpoint per Xbox gamepad, plus one IN endpoint for the optional HID
 * utility interface. The descriptor table below currently defines endpoints
 * for up to two Xbox gamepads.
 */
#if DONGLE_MAX_CONTROLLERS > 2
#error "This ESP32-S3 descriptor set supports at most 2 Xbox gamepads"
#endif

#if DONGLE_ENABLE_UTILITY_HID && DONGLE_MAX_CONTROLLERS > 2
#error "Utility HID plus more than 2 Xbox gamepads exceeds the supported endpoint layout"
#endif

static inline uint8_t dongle_controller_count(void)
{
    return (uint8_t)DONGLE_MAX_CONTROLLERS;
}
