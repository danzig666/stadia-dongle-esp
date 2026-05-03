#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "dongle_state.h"

void mouse_mode_init(void);
bool mouse_mode_is_active(uint8_t usb_gamepad_index);
void mouse_mode_toggle(uint8_t usb_gamepad_index);
void mouse_mode_reset(uint8_t usb_gamepad_index);
void mouse_mode_cache_state(uint8_t usb_gamepad_index, const stadia_controller_state_t *state);
void mouse_mode_timer_task(void *arg);
