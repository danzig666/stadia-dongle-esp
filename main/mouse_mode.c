#include "mouse_mode.h"

#include "ble_central.h"
#include "dongle_state.h"
#include "hid_mouse.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "MOUSE";

static bool s_mouse_mode[DONGLE_MAX_CONTROLLERS];

static int32_t s_smooth_x[DONGLE_MAX_CONTROLLERS];
static int32_t s_smooth_y[DONGLE_MAX_CONTROLLERS];
static bool s_dpad_up_prev[DONGLE_MAX_CONTROLLERS];
static bool s_dpad_down_prev[DONGLE_MAX_CONTROLLERS];
static bool s_a_prev[DONGLE_MAX_CONTROLLERS];
static bool s_b_prev[DONGLE_MAX_CONTROLLERS];
static bool s_x_prev[DONGLE_MAX_CONTROLLERS];
static int32_t s_scroll_accum[DONGLE_MAX_CONTROLLERS];
static stadia_controller_state_t s_cached_state[DONGLE_MAX_CONTROLLERS];
static bool s_cached_valid[DONGLE_MAX_CONTROLLERS];

#define MOUSE_STICK_DEADZONE 3277
#define MOUSE_SCROLL_DEADZONE 6000
#define MOUSE_SPEED_MAX 18
#define MOUSE_PRECISION_DIV 3
#define MOUSE_FAST_MUL 2

void mouse_mode_init(void)
{
    for (uint8_t i = 0; i < DONGLE_MAX_CONTROLLERS; i++) {
        s_mouse_mode[i] = false;
        s_smooth_x[i] = 0;
        s_smooth_y[i] = 0;
        s_dpad_up_prev[i] = false;
        s_dpad_down_prev[i] = false;
        s_a_prev[i] = false;
        s_b_prev[i] = false;
        s_x_prev[i] = false;
        s_scroll_accum[i] = 0;
        s_cached_valid[i] = false;
    }
    xTaskCreate(mouse_mode_timer_task, "mouse_timer", 2048, NULL, 3, NULL);
}

bool mouse_mode_is_active(uint8_t usb_gamepad_index)
{
    if (usb_gamepad_index >= DONGLE_MAX_CONTROLLERS) return false;
    return s_mouse_mode[usb_gamepad_index];
}

static void mouse_mode_rumble(uint8_t usb_gamepad_index, int pulse_count)
{
    uint8_t rumble_on[] = {0x00, 0xFF, 0x00, 0xFF};
    uint8_t rumble_off[] = {0x00, 0x00, 0x00, 0x00};
    for (int i = 0; i < pulse_count; i++) {
        ble_central_send_rumble(usb_gamepad_index, rumble_on, sizeof(rumble_on));
        vTaskDelay(pdMS_TO_TICKS(80));
        ble_central_send_rumble(usb_gamepad_index, rumble_off, sizeof(rumble_off));
        if (i < pulse_count - 1) vTaskDelay(pdMS_TO_TICKS(120));
    }
}

void mouse_mode_toggle(uint8_t usb_gamepad_index)
{
    if (usb_gamepad_index >= DONGLE_MAX_CONTROLLERS) return;

    s_mouse_mode[usb_gamepad_index] = !s_mouse_mode[usb_gamepad_index];
    s_smooth_x[usb_gamepad_index] = 0;
    s_smooth_y[usb_gamepad_index] = 0;
    dongle_state_controller_set_mouse_mode(usb_gamepad_index, s_mouse_mode[usb_gamepad_index]);

    if (s_mouse_mode[usb_gamepad_index]) {
        ESP_LOGI(TAG, "usb %u mouse mode ON", usb_gamepad_index);
        mouse_mode_rumble(usb_gamepad_index, 2);
    } else {
        ESP_LOGI(TAG, "usb %u mouse mode OFF", usb_gamepad_index);
        mouse_mode_rumble(usb_gamepad_index, 3);
    }
}

void mouse_mode_reset(uint8_t usb_gamepad_index)
{
    if (usb_gamepad_index >= DONGLE_MAX_CONTROLLERS) return;
    s_mouse_mode[usb_gamepad_index] = false;
    s_smooth_x[usb_gamepad_index] = 0;
    s_smooth_y[usb_gamepad_index] = 0;
    s_cached_valid[usb_gamepad_index] = false;
    dongle_state_controller_set_mouse_mode(usb_gamepad_index, false);
}

static int32_t apply_deadzone(int32_t v)
{
    if (v > -MOUSE_STICK_DEADZONE && v < MOUSE_STICK_DEADZONE) return 0;
    return v;
}

static int8_t accelerate(int32_t v)
{
    if (v == 0) return 0;
    int sign = (v > 0) ? 1 : -1;
    uint32_t mag = (uint32_t)(v > 0 ? v : -v);

    uint32_t min_mag = MOUSE_STICK_DEADZONE;
    if (mag <= min_mag) return 0;
    mag -= min_mag;

    uint32_t full_range = 32767 - min_mag;
    uint64_t norm = (uint64_t)mag * 32767 / full_range;

    // Blend: 40% linear, 60% quadratic for gentler ramp-up
    uint64_t linear = norm * 4;
    uint64_t quad = norm * norm / 32767 * 6;
    uint64_t curve = (linear + quad) / 10;

    int64_t result = (int64_t)(curve * MOUSE_SPEED_MAX / 32767) * sign;
    if (result > MOUSE_SPEED_MAX) result = MOUSE_SPEED_MAX;
    if (result < -MOUSE_SPEED_MAX) result = -MOUSE_SPEED_MAX;
    return (int8_t)result;
}

static int32_t smooth(int32_t current, int32_t previous)
{
    int32_t diff = current - previous;
    if (diff > 0) return previous + (diff + 1) / 2;
    if (diff < 0) return previous + (diff - 1) / 2;
    return previous;
}

void mouse_mode_cache_state(uint8_t usb_gamepad_index, const stadia_controller_state_t *state)
{
    if (usb_gamepad_index >= DONGLE_MAX_CONTROLLERS || !state) return;
    s_cached_state[usb_gamepad_index] = *state;
    s_cached_valid[usb_gamepad_index] = true;
}

void mouse_mode_process_state(uint8_t usb_gamepad_index, const stadia_controller_state_t *state)
{
    if (usb_gamepad_index >= DONGLE_MAX_CONTROLLERS || !state) return;
    if (!s_mouse_mode[usb_gamepad_index]) return;

    int32_t raw_x = state->lx;
    int32_t raw_y = state->ly;

    raw_x = apply_deadzone(raw_x);
    raw_y = apply_deadzone(raw_y);

    int8_t accel_x = accelerate(raw_x);
    int8_t accel_y = accelerate(raw_y);

    // Invert Y: map_stick gives +32767 for stick UP, but USB HID mouse
    // uses positive Y = down on most hosts. Negate to match.
    accel_y = -accel_y;

    // Circle-to-square diagonal compensation: per-axis acceleration
    // naturally makes diagonals slower than cardinals (f(0.707)² combined
    // < f(1.0) for any sub-linear curve).  Boost non-cardinal directions
    // so cursor speed is independent of stick angle.
    {
        int32_t ax = accel_x > 0 ? accel_x : -accel_x;
        int32_t ay = accel_y > 0 ? accel_y : -accel_y;
        if (ax > ay && ax > 0) {
            int32_t boost = 256 + ay * 54 / ax;
            accel_x = (int8_t)((int32_t)accel_x * boost / 256);
            accel_y = (int8_t)((int32_t)accel_y * boost / 256);
        } else if (ay > 0) {
            int32_t boost = 256 + ax * 54 / ay;
            accel_x = (int8_t)((int32_t)accel_x * boost / 256);
            accel_y = (int8_t)((int32_t)accel_y * boost / 256);
        }
    }

    if (state->lt > 128) {
        accel_x = accel_x / MOUSE_PRECISION_DIV;
        accel_y = accel_y / MOUSE_PRECISION_DIV;
    }

    if (state->rt > 128) {
        int fast_x = accel_x * MOUSE_FAST_MUL;
        int fast_y = accel_y * MOUSE_FAST_MUL;
        if (fast_x > MOUSE_SPEED_MAX) fast_x = MOUSE_SPEED_MAX;
        if (fast_x < -MOUSE_SPEED_MAX) fast_x = -MOUSE_SPEED_MAX;
        if (fast_y > MOUSE_SPEED_MAX) fast_y = MOUSE_SPEED_MAX;
        if (fast_y < -MOUSE_SPEED_MAX) fast_y = -MOUSE_SPEED_MAX;
        accel_x = (int8_t)fast_x;
        accel_y = (int8_t)fast_y;
    }

    s_smooth_x[usb_gamepad_index] = smooth(accel_x, s_smooth_x[usb_gamepad_index]);
    s_smooth_y[usb_gamepad_index] = smooth(accel_y, s_smooth_y[usb_gamepad_index]);

    uint8_t buttons = 0;
    if (state->a) buttons |= 0x01;
    if (state->b) buttons |= 0x02;
    if (state->x) buttons |= 0x04;

    int8_t wheel = 0;
    if (state->dpad_up && !s_dpad_up_prev[usb_gamepad_index]) wheel += 1;
    if (state->dpad_down && !s_dpad_down_prev[usb_gamepad_index]) wheel -= 1;
    s_dpad_up_prev[usb_gamepad_index] = state->dpad_up;
    s_dpad_down_prev[usb_gamepad_index] = state->dpad_down;

    // Right stick Y for scrolling (sub-pixel accumulator avoids truncation)
    {
        int32_t sr = state->ry;
        if (sr > -MOUSE_SCROLL_DEADZONE && sr < MOUSE_SCROLL_DEADZONE) sr = 0;
        s_scroll_accum[usb_gamepad_index] += (int64_t)sr * 160 / 32767;
        int8_t st = (int8_t)(s_scroll_accum[usb_gamepad_index] / 256);
        s_scroll_accum[usb_gamepad_index] -= (int32_t)st * 256;
        wheel += st;
    }

    int8_t out_x = (int8_t)s_smooth_x[usb_gamepad_index];
    int8_t out_y = (int8_t)s_smooth_y[usb_gamepad_index];

    if (out_x == 0 && out_y == 0 && wheel == 0 &&
        buttons == (uint8_t)((s_a_prev[usb_gamepad_index] ? 0x01 : 0) |
                              (s_b_prev[usb_gamepad_index] ? 0x02 : 0) |
                              (s_x_prev[usb_gamepad_index] ? 0x04 : 0))) {
        return;
    }

    s_a_prev[usb_gamepad_index] = state->a;
    s_b_prev[usb_gamepad_index] = state->b;
    s_x_prev[usb_gamepad_index] = state->x;

    hid_mouse_send_report(buttons, out_x, out_y, wheel);
}

void mouse_mode_timer_task(void *arg)
{
    (void)arg;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(8));
        for (uint8_t i = 0; i < DONGLE_MAX_CONTROLLERS; i++) {
            if (!s_mouse_mode[i] || !s_cached_valid[i]) continue;
            mouse_mode_process_state(i, &s_cached_state[i]);
        }
    }
}
