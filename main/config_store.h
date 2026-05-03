#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    DONGLE_ACTION_NONE = 0,
    DONGLE_ACTION_KEY_F13,
    DONGLE_ACTION_KEY_F14,
    DONGLE_ACTION_KEY_PRINTSCREEN,
    DONGLE_ACTION_KEY_ESCAPE,
    DONGLE_ACTION_KEY_SPACE,
    DONGLE_ACTION_KEY_ENTER,
    DONGLE_ACTION_CONSUMER_VOLUME_UP,
    DONGLE_ACTION_CONSUMER_VOLUME_DOWN,
    DONGLE_ACTION_CONSUMER_PLAY_PAUSE,
    DONGLE_ACTION_START_WEBUI,
    DONGLE_ACTION_REMOTE_WAKE_ONLY,
    DONGLE_ACTION_KEY_F15,
    DONGLE_ACTION_KEY_F16,
    DONGLE_ACTION_KEY_F17,
    DONGLE_ACTION_KEY_F18,
    DONGLE_ACTION_KEY_F19,
    DONGLE_ACTION_KEY_F20,
    DONGLE_ACTION_KEY_F21,
    DONGLE_ACTION_KEY_F22,
    DONGLE_ACTION_KEY_F23,
    DONGLE_ACTION_KEY_F24,
    DONGLE_ACTION_KEY_TAB,
    DONGLE_ACTION_KEY_BACKSPACE,
    DONGLE_ACTION_KEY_INSERT,
    DONGLE_ACTION_KEY_DELETE,
    DONGLE_ACTION_KEY_HOME,
    DONGLE_ACTION_KEY_END,
    DONGLE_ACTION_KEY_PAGE_UP,
    DONGLE_ACTION_KEY_PAGE_DOWN,
    DONGLE_ACTION_KEY_ARROW_UP,
    DONGLE_ACTION_KEY_ARROW_DOWN,
    DONGLE_ACTION_KEY_ARROW_LEFT,
    DONGLE_ACTION_KEY_ARROW_RIGHT,
    DONGLE_ACTION_CONSUMER_MUTE,
    DONGLE_ACTION_CONSUMER_SCAN_NEXT,
    DONGLE_ACTION_CONSUMER_SCAN_PREV,
} dongle_action_t;

typedef struct {
    uint32_t schema_version;
    uint8_t assistant_short_action;
    uint8_t assistant_long_action;
    uint8_t capture_short_action;
    uint8_t capture_long_action;
    uint16_t long_press_ms;
    bool webui_auto_start_if_no_bond;
    uint32_t webui_timeout_after_ble_connected_ms;
    bool disable_ap_on_usb_suspend;
} dongle_config_t;

void config_store_defaults(dongle_config_t *cfg);
void config_store_init(void);
void config_store_get(dongle_config_t *out);
void config_store_set(const dongle_config_t *cfg);
const char *config_store_action_name(uint8_t action);
uint8_t config_store_action_from_name(const char *name);
