#pragma once

#include "dongle_state.h"

void button_actions_init(void);
void button_actions_on_state(uint8_t gamepad_index, const stadia_controller_state_t *state);
