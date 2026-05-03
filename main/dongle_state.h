#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "dongle_config.h"

#define DONGLE_CONTROLLER_NAME_LEN 32
#define DONGLE_ADDR_STR_LEN 18
#define DONGLE_ERROR_LEN 64

typedef enum {
    DONGLE_STATE_BOOTING,
    DONGLE_STATE_NO_BOND_SETUP,
    DONGLE_STATE_SCANNING,
    DONGLE_STATE_PAIRING,
    DONGLE_STATE_CONNECTING,
    DONGLE_STATE_CONNECTED,
    DONGLE_STATE_CONNECTED_WEBUI_ACTIVE,
    DONGLE_STATE_DISCONNECTED,
    DONGLE_STATE_OTA_UPDATE,
    DONGLE_STATE_ERROR
} dongle_state_t;

typedef struct {
    bool dpad_up;
    bool dpad_down;
    bool dpad_left;
    bool dpad_right;
    bool a;
    bool b;
    bool x;
    bool y;
    bool lb;
    bool rb;
    bool ls;
    bool rs;
    bool menu;
    bool options;
    bool stadia;
    bool assistant;
    bool capture;
    uint8_t lt;
    uint8_t rt;
    int16_t lx;
    int16_t ly;
    int16_t rx;
    int16_t ry;
    uint8_t raw[DONGLE_MAX_RAW_REPORT_SIZE];
    size_t raw_len;
} stadia_controller_state_t;

typedef struct {
    bool used;
    bool connected;
    bool ready;
    bool bonded;
    uint8_t slot_index;
    uint8_t usb_gamepad_index;
    char controller_address[DONGLE_ADDR_STR_LEN];
    char controller_name[DONGLE_CONTROLLER_NAME_LEN];
    stadia_controller_state_t stadia;
    uint8_t xbox[DONGLE_XBOX_REPORT_SIZE];
    bool xbox_valid;
    int battery_percent;
    bool mouse_mode;
} dongle_controller_status_t;

typedef struct {
    dongle_state_t state;
    bool ble_connected;
    int connected_count;
    char controller_address[DONGLE_ADDR_STR_LEN];
    char controller_name[DONGLE_CONTROLLER_NAME_LEN];
    bool controller_bonded;
    int stored_bonds;
    bool pairing_mode;
    bool webui_active;
    int64_t webui_auto_off_deadline_us;
    stadia_controller_state_t stadia;
    uint8_t xbox[DONGLE_XBOX_REPORT_SIZE];
    bool xbox_valid;
    int battery_percent;
    char firmware_version[24];
    char build_date[32];
    bool usb_configured;
    bool usb_suspended;
    bool usb_remote_wakeup_enabled;
    int64_t last_wake_attempt_us;
    bool last_wake_attempt_allowed;
    char last_error[DONGLE_ERROR_LEN];
    dongle_controller_status_t controllers[DONGLE_MAX_CONTROLLERS];
} dongle_status_t;

void dongle_state_init(void);
void dongle_state_set(dongle_state_t state);
dongle_state_t dongle_state_get(void);
const char *dongle_state_name(dongle_state_t state);
void dongle_state_get_status(dongle_status_t *out);
void dongle_state_set_ble_connected(bool connected);
void dongle_state_set_controller(const char *addr, const char *name, bool bonded);
void dongle_state_set_stored_bonds(int count);
void dongle_state_set_pairing_mode(bool active);
void dongle_state_set_webui(bool active, int64_t auto_off_deadline_us);
void dongle_state_set_battery(int percent);
void dongle_state_controller_clear(uint8_t slot_index);
void dongle_state_controller_set_info(uint8_t slot_index, uint8_t usb_gamepad_index,
                                      const char *addr, const char *name, bool bonded);
void dongle_state_controller_set_connected(uint8_t slot_index, bool connected, bool ready);
void dongle_state_controller_set_battery(uint8_t slot_index, int percent);
void dongle_state_update_controller_reports(uint8_t slot_index, const uint8_t *raw, size_t raw_len,
                                            const stadia_controller_state_t *stadia,
                                            const uint8_t *xbox);
void dongle_state_set_usb_configured(bool configured);
void dongle_state_set_usb_suspended(bool suspended);
void dongle_state_set_usb_remote_wakeup_enabled(bool enabled);
void dongle_state_record_wake_attempt(bool allowed);
void dongle_state_set_error(const char *error);
void dongle_state_update_reports(const uint8_t *raw, size_t raw_len,
                                 const stadia_controller_state_t *stadia,
                                 const uint8_t *xbox);
void dongle_state_controller_set_mouse_mode(uint8_t slot_index, bool active);
