#include "dongle_state.h"

#include "esp_app_desc.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include <stdio.h>
#include <string.h>

static SemaphoreHandle_t s_lock;
static dongle_status_t s_status;

static void lock_state(void)
{
    if (s_lock) xSemaphoreTake(s_lock, portMAX_DELAY);
}

static void unlock_state(void)
{
    if (s_lock) xSemaphoreGive(s_lock);
}

void dongle_state_init(void)
{
    s_lock = xSemaphoreCreateMutex();
    configASSERT(s_lock != NULL);
    memset(&s_status, 0, sizeof(s_status));
    s_status.state = DONGLE_STATE_BOOTING;
    s_status.battery_percent = -1;
    for (uint8_t i = 0; i < DONGLE_MAX_CONTROLLERS; i++) {
        s_status.controllers[i].slot_index = i;
        s_status.controllers[i].usb_gamepad_index = i;
        s_status.controllers[i].battery_percent = -1;
    }
    const esp_app_desc_t *desc = esp_app_get_description();
    snprintf(s_status.firmware_version, sizeof(s_status.firmware_version), "%.*s",
             (int)sizeof(s_status.firmware_version) - 1, desc->version);
    snprintf(s_status.build_date, sizeof(s_status.build_date), "%s %s", desc->date, desc->time);
}

const char *dongle_state_name(dongle_state_t state)
{
    switch (state) {
    case DONGLE_STATE_BOOTING: return "booting";
    case DONGLE_STATE_NO_BOND_SETUP: return "no_bond_setup";
    case DONGLE_STATE_SCANNING: return "scanning";
    case DONGLE_STATE_PAIRING: return "pairing";
    case DONGLE_STATE_CONNECTING: return "connecting";
    case DONGLE_STATE_CONNECTED: return "connected";
    case DONGLE_STATE_CONNECTED_WEBUI_ACTIVE: return "connected_webui_active";
    case DONGLE_STATE_DISCONNECTED: return "disconnected";
    case DONGLE_STATE_OTA_UPDATE: return "ota_update";
    case DONGLE_STATE_ERROR: return "error";
    default: return "unknown";
    }
}

void dongle_state_set(dongle_state_t state)
{
    lock_state();
    s_status.state = state;
    unlock_state();
}

dongle_state_t dongle_state_get(void)
{
    lock_state();
    dongle_state_t state = s_status.state;
    unlock_state();
    return state;
}

void dongle_state_get_status(dongle_status_t *out)
{
    if (!out) return;
    lock_state();
    *out = s_status;
    unlock_state();
}

void dongle_state_set_ble_connected(bool connected)
{
    lock_state();
    s_status.ble_connected = connected;
    unlock_state();
}

static void update_compat_locked(void)
{
    s_status.connected_count = 0;
    s_status.ble_connected = false;
    const dongle_controller_status_t *first = NULL;
    for (uint8_t i = 0; i < DONGLE_MAX_CONTROLLERS; i++) {
        if (s_status.controllers[i].connected) {
            s_status.connected_count++;
            s_status.ble_connected = true;
            if (!first) first = &s_status.controllers[i];
        }
    }
    if (!first) first = &s_status.controllers[0];
    snprintf(s_status.controller_address, sizeof(s_status.controller_address), "%s", first->controller_address);
    snprintf(s_status.controller_name, sizeof(s_status.controller_name), "%s", first->controller_name);
    s_status.controller_bonded = first->bonded;
    s_status.battery_percent = first->battery_percent;
    s_status.stadia = first->stadia;
    if (first->xbox_valid) {
        memcpy(s_status.xbox, first->xbox, DONGLE_XBOX_REPORT_SIZE);
        s_status.xbox_valid = true;
    }
}

void dongle_state_set_controller(const char *addr, const char *name, bool bonded)
{
    lock_state();
    snprintf(s_status.controller_address, sizeof(s_status.controller_address), "%s", addr ? addr : "");
    snprintf(s_status.controller_name, sizeof(s_status.controller_name), "%s", name ? name : "");
    s_status.controller_bonded = bonded;
    unlock_state();
}

void dongle_state_set_stored_bonds(int count)
{
    lock_state();
    s_status.stored_bonds = count;
    unlock_state();
}

void dongle_state_set_pairing_mode(bool active)
{
    lock_state();
    s_status.pairing_mode = active;
    unlock_state();
}

void dongle_state_set_webui(bool active, int64_t auto_off_deadline_us)
{
    lock_state();
    s_status.webui_active = active;
    s_status.webui_auto_off_deadline_us = auto_off_deadline_us;
    if (s_status.ble_connected && active && s_status.state == DONGLE_STATE_CONNECTED) {
        s_status.state = DONGLE_STATE_CONNECTED_WEBUI_ACTIVE;
    } else if (!active && s_status.state == DONGLE_STATE_CONNECTED_WEBUI_ACTIVE) {
        s_status.state = s_status.ble_connected ? DONGLE_STATE_CONNECTED : DONGLE_STATE_DISCONNECTED;
    }
    unlock_state();
}

void dongle_state_set_battery(int percent)
{
    if (percent > 100) percent = 100;
    if (percent < -1) percent = -1;
    lock_state();
    s_status.battery_percent = percent;
    dongle_controller_status_t *first = &s_status.controllers[0];
    for (uint8_t i = 0; i < DONGLE_MAX_CONTROLLERS; i++) {
        if (s_status.controllers[i].connected) { first = &s_status.controllers[i]; break; }
    }
    first->battery_percent = percent;
    unlock_state();
}

void dongle_state_controller_clear(uint8_t slot_index)
{
    if (slot_index >= DONGLE_MAX_CONTROLLERS) return;
    lock_state();
    uint8_t usb_index = s_status.controllers[slot_index].usb_gamepad_index;
    memset(&s_status.controllers[slot_index], 0, sizeof(s_status.controllers[slot_index]));
    s_status.controllers[slot_index].slot_index = slot_index;
    s_status.controllers[slot_index].usb_gamepad_index = usb_index;
    s_status.controllers[slot_index].battery_percent = -1;
    s_status.controllers[slot_index].mouse_mode = false;
    update_compat_locked();
    unlock_state();
}

void dongle_state_controller_set_info(uint8_t slot_index, uint8_t usb_gamepad_index,
                                      const char *addr, const char *name, bool bonded)
{
    if (slot_index >= DONGLE_MAX_CONTROLLERS) return;
    lock_state();
    dongle_controller_status_t *c = &s_status.controllers[slot_index];
    c->used = true;
    c->slot_index = slot_index;
    c->usb_gamepad_index = usb_gamepad_index;
    c->bonded = bonded;
    snprintf(c->controller_address, sizeof(c->controller_address), "%s", addr ? addr : "");
    snprintf(c->controller_name, sizeof(c->controller_name), "%s", name ? name : "");
    update_compat_locked();
    unlock_state();
}

void dongle_state_controller_set_connected(uint8_t slot_index, bool connected, bool ready)
{
    if (slot_index >= DONGLE_MAX_CONTROLLERS) return;
    lock_state();
    s_status.controllers[slot_index].used = connected || s_status.controllers[slot_index].used;
    s_status.controllers[slot_index].connected = connected;
    s_status.controllers[slot_index].ready = ready;
    update_compat_locked();
    unlock_state();
}

void dongle_state_controller_set_battery(uint8_t slot_index, int percent)
{
    if (slot_index >= DONGLE_MAX_CONTROLLERS) return;
    if (percent > 100) percent = 100;
    if (percent < -1) percent = -1;
    lock_state();
    s_status.controllers[slot_index].battery_percent = percent;
    update_compat_locked();
    unlock_state();
}

void dongle_state_update_controller_reports(uint8_t slot_index, const uint8_t *raw, size_t raw_len,
                                            const stadia_controller_state_t *stadia,
                                            const uint8_t *xbox)
{
    if (slot_index >= DONGLE_MAX_CONTROLLERS) return;
    lock_state();
    dongle_controller_status_t *c = &s_status.controllers[slot_index];
    c->used = true;
    if (stadia) {
        c->stadia = *stadia;
    }
    if (raw) {
        size_t n = raw_len > sizeof(c->stadia.raw) ? sizeof(c->stadia.raw) : raw_len;
        memcpy(c->stadia.raw, raw, n);
        c->stadia.raw_len = n;
    }
    if (xbox) {
        memcpy(c->xbox, xbox, DONGLE_XBOX_REPORT_SIZE);
        c->xbox_valid = true;
    }
    update_compat_locked();
    unlock_state();
}

void dongle_state_set_usb_configured(bool configured)
{
    lock_state();
    s_status.usb_configured = configured;
    unlock_state();
}

void dongle_state_set_usb_suspended(bool suspended)
{
    lock_state();
    s_status.usb_suspended = suspended;
    unlock_state();
}

void dongle_state_set_usb_remote_wakeup_enabled(bool enabled)
{
    lock_state();
    s_status.usb_remote_wakeup_enabled = enabled;
    unlock_state();
}

void dongle_state_record_wake_attempt(bool allowed)
{
    lock_state();
    s_status.last_wake_attempt_us = esp_timer_get_time();
    s_status.last_wake_attempt_allowed = allowed;
    unlock_state();
}

void dongle_state_set_error(const char *error)
{
    lock_state();
    snprintf(s_status.last_error, sizeof(s_status.last_error), "%s", error ? error : "");
    if (error && error[0]) s_status.state = DONGLE_STATE_ERROR;
    unlock_state();
}

void dongle_state_update_reports(const uint8_t *raw, size_t raw_len,
                                 const stadia_controller_state_t *stadia,
                                 const uint8_t *xbox)
{
    lock_state();
    if (stadia) {
        s_status.stadia = *stadia;
    }
    if (raw) {
        size_t n = raw_len > sizeof(s_status.stadia.raw) ? sizeof(s_status.stadia.raw) : raw_len;
        memcpy(s_status.stadia.raw, raw, n);
        s_status.stadia.raw_len = n;
    }
    if (xbox) {
        memcpy(s_status.xbox, xbox, DONGLE_XBOX_REPORT_SIZE);
        s_status.xbox_valid = true;
    }
    unlock_state();
}

void dongle_state_controller_set_mouse_mode(uint8_t slot_index, bool active)
{
    if (slot_index >= DONGLE_MAX_CONTROLLERS) return;
    lock_state();
    s_status.controllers[slot_index].mouse_mode = active;
    unlock_state();
}
