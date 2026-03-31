#pragma once
#include <stdbool.h>
#include <stdint.h>

// Custom TinyUSB class driver for Xbox 360 controller.
// Replaces the generic vendor class to use direct interrupt endpoint transfers.
void xbox_dev_init(void);
bool xbox_send_report(const uint8_t *report);
