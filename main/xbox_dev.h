#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Custom TinyUSB class driver for Xbox 360 controller.
// Replaces the generic vendor class to use direct interrupt endpoint transfers.
void xbox_dev_init(void);
int xbox_send_report(uint8_t gamepad_index, const uint8_t *report, size_t len);
void xbox_recover_after_resume(void);
