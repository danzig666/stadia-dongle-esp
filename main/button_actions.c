#include "button_actions.h"

#include "config_store.h"
#include "hid_extra.h"
#include "mouse_mode.h"
#include "web_server.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

static QueueHandle_t s_queue;
static const char *TAG = "ACT";

typedef struct {
    uint8_t gamepad_index;
    stadia_controller_state_t state;
} button_action_msg_t;

typedef struct {
    bool pressed;
    bool long_sent;
    int64_t press_time_us;
    uint8_t gamepad_index;
    uint8_t short_action;
    uint8_t long_action;
} action_button_state_t;

static uint8_t action_to_key(uint8_t action)
{
    switch ((dongle_action_t)action) {
    case DONGLE_ACTION_KEY_F13: return 0x68;
    case DONGLE_ACTION_KEY_F14: return 0x69;
    case DONGLE_ACTION_KEY_PRINTSCREEN: return 0x46;
    case DONGLE_ACTION_KEY_ESCAPE: return 0x29;
    case DONGLE_ACTION_KEY_SPACE: return 0x2c;
    case DONGLE_ACTION_KEY_ENTER: return 0x28;
    case DONGLE_ACTION_KEY_F15: return 0x6a;
    case DONGLE_ACTION_KEY_F16: return 0x6b;
    case DONGLE_ACTION_KEY_F17: return 0x6c;
    case DONGLE_ACTION_KEY_F18: return 0x6d;
    case DONGLE_ACTION_KEY_F19: return 0x6e;
    case DONGLE_ACTION_KEY_F20: return 0x6f;
    case DONGLE_ACTION_KEY_F21: return 0x70;
    case DONGLE_ACTION_KEY_F22: return 0x71;
    case DONGLE_ACTION_KEY_F23: return 0x72;
    case DONGLE_ACTION_KEY_F24: return 0x73;
    case DONGLE_ACTION_KEY_TAB: return 0x2b;
    case DONGLE_ACTION_KEY_BACKSPACE: return 0x2a;
    case DONGLE_ACTION_KEY_INSERT: return 0x49;
    case DONGLE_ACTION_KEY_DELETE: return 0x4c;
    case DONGLE_ACTION_KEY_HOME: return 0x4a;
    case DONGLE_ACTION_KEY_END: return 0x4d;
    case DONGLE_ACTION_KEY_PAGE_UP: return 0x4b;
    case DONGLE_ACTION_KEY_PAGE_DOWN: return 0x4e;
    case DONGLE_ACTION_KEY_ARROW_UP: return 0x52;
    case DONGLE_ACTION_KEY_ARROW_DOWN: return 0x51;
    case DONGLE_ACTION_KEY_ARROW_LEFT: return 0x50;
    case DONGLE_ACTION_KEY_ARROW_RIGHT: return 0x4f;
    default: return 0;
    }
}

static uint16_t action_to_consumer(uint8_t action)
{
    switch ((dongle_action_t)action) {
    case DONGLE_ACTION_CONSUMER_VOLUME_UP: return 0x00e9;
    case DONGLE_ACTION_CONSUMER_VOLUME_DOWN: return 0x00ea;
    case DONGLE_ACTION_CONSUMER_PLAY_PAUSE: return 0x00cd;
    case DONGLE_ACTION_CONSUMER_MUTE: return 0x00e2;
    case DONGLE_ACTION_CONSUMER_SCAN_NEXT: return 0x00b5;
    case DONGLE_ACTION_CONSUMER_SCAN_PREV: return 0x00b6;
    default: return 0;
    }
}

static void run_action(uint8_t gamepad_index, uint8_t action)
{
    uint8_t key = action_to_key(action);
    uint16_t consumer = action_to_consumer(action);
    if (key) {
        bool sent = hid_extra_send_key_press(0, key);
        vTaskDelay(pdMS_TO_TICKS(20));
        hid_extra_send_key_release();
        ESP_LOGI(TAG, "usb %u keyboard action %s key=0x%02x sent=%s",
                 gamepad_index, config_store_action_name(action), key, sent ? "yes" : "no");
    } else if (consumer) {
        bool sent = hid_extra_send_consumer_press(consumer);
        vTaskDelay(pdMS_TO_TICKS(20));
        hid_extra_send_consumer_release();
        ESP_LOGI(TAG, "usb %u consumer action %s usage=0x%04x sent=%s",
                 gamepad_index, config_store_action_name(action), consumer, sent ? "yes" : "no");
    } else if (action == DONGLE_ACTION_START_WEBUI) {
        ESP_LOGI(TAG, "usb %u action start_webui", gamepad_index);
        web_server_start(true);
    } else if (action == DONGLE_ACTION_REMOTE_WAKE_ONLY) {
        bool sent = hid_extra_remote_wakeup();
        ESP_LOGI(TAG, "usb %u action remote_wake_only sent=%s", gamepad_index, sent ? "yes" : "no");
    } else {
        ESP_LOGI(TAG, "usb %u action none", gamepad_index);
    }
}

static void update_action_button(uint8_t gamepad_index, action_button_state_t *btn, bool now_pressed,
                                 uint8_t short_action, uint8_t long_action)
{
    int64_t now = esp_timer_get_time();
    if (now_pressed && !btn->pressed) {
        btn->pressed = true;
        btn->long_sent = false;
        btn->press_time_us = now;
        btn->gamepad_index = gamepad_index;
        btn->short_action = short_action;
        btn->long_action = long_action;
        ESP_LOGI(TAG, "usb %u action button pressed short=%s long=%s",
                 gamepad_index, config_store_action_name(short_action),
                 config_store_action_name(long_action));
    } else if (now_pressed && btn->pressed) {
        btn->short_action = short_action;
        btn->long_action = long_action;
    } else if (!now_pressed && btn->pressed) {
        if (!btn->long_sent) run_action(gamepad_index, btn->short_action);
        btn->pressed = false;
        btn->long_sent = false;
    }
}

static void poll_long_action(action_button_state_t *btn, uint16_t long_press_ms)
{
    if (!btn->pressed || btn->long_sent) return;
    if (esp_timer_get_time() - btn->press_time_us < (int64_t)long_press_ms * 1000) return;
    btn->long_sent = true;
    ESP_LOGI(TAG, "usb %u long press action %s", btn->gamepad_index,
             config_store_action_name(btn->long_action));
    run_action(btn->gamepad_index, btn->long_action);
}

static void button_actions_task(void *arg)
{
    (void)arg;
    if (!s_queue) { vTaskDelete(NULL); return; }
    button_action_msg_t msg = {0};
    action_button_state_t assistant[DONGLE_MAX_CONTROLLERS] = {0};
    action_button_state_t capture[DONGLE_MAX_CONTROLLERS] = {0};
    int64_t setup_combo_start_us[DONGLE_MAX_CONTROLLERS] = {0};
    int64_t mouse_combo_start_us[DONGLE_MAX_CONTROLLERS] = {0};
    bool mouse_combo_triggered[DONGLE_MAX_CONTROLLERS] = {0};

    while (1) {
        dongle_config_t cfg;
        config_store_get(&cfg);

        if (xQueueReceive(s_queue, &msg, pdMS_TO_TICKS(25)) == pdTRUE) {
            uint8_t idx = msg.gamepad_index < DONGLE_MAX_CONTROLLERS ? msg.gamepad_index : 0;
            stadia_controller_state_t st = msg.state;

            bool setup_combo = st.stadia && st.options && st.menu;
            if (setup_combo) {
                if (!setup_combo_start_us[idx]) setup_combo_start_us[idx] = esp_timer_get_time();
                if (esp_timer_get_time() - setup_combo_start_us[idx] > 5000000) {
                    web_server_start(true);
                    setup_combo_start_us[idx] = esp_timer_get_time();
                }
            } else {
                setup_combo_start_us[idx] = 0;
            }

            dongle_status_t status;
            dongle_state_get_status(&status);
            if (status.usb_suspended && (st.a || st.stadia || st.assistant || st.capture)) {
                hid_extra_remote_wakeup();
            }

            // Process individual buttons first so pressed/released state is tracked.
            update_action_button(idx, &assistant[idx], st.assistant, cfg.assistant_short_action,
                                 cfg.assistant_long_action);
            update_action_button(idx, &capture[idx], st.capture, cfg.capture_short_action,
                                 cfg.capture_long_action);

            // Track mouse combo: both assistant and capture pressed simultaneously.
            // Timer is checked in the poll loop below to survive brief BLE glitches.
            bool mouse_combo = st.assistant && st.capture;
            if (mouse_combo) {
                if (!mouse_combo_start_us[idx]) {
                    mouse_combo_start_us[idx] = esp_timer_get_time();
                    ESP_LOGI(TAG, "usb %u mouse combo started", idx);
                }
            } else {
                // Only reset combo when BOTH buttons are released (prevents
                // single-frame BLE glitches from cancelling the combo).
                if (!st.assistant && !st.capture) {
                    if (mouse_combo_start_us[idx]) {
                        ESP_LOGI(TAG, "usb %u mouse combo cancelled (both released)", idx);
                    }
                    mouse_combo_start_us[idx] = 0;
                    mouse_combo_triggered[idx] = false;
                }
            }
        }

        for (uint8_t i = 0; i < DONGLE_MAX_CONTROLLERS; i++) {
            bool combo_pending = (mouse_combo_start_us[i] != 0 && !mouse_combo_triggered[i]);
            if (combo_pending) {
                if (esp_timer_get_time() - mouse_combo_start_us[i] > 2000000) {
                    mouse_combo_triggered[i] = true;
                    ESP_LOGI(TAG, "usb %u mouse combo triggering toggle", i);
                    mouse_mode_toggle(i);
                } else {
                    assistant[i].long_sent = true;
                    capture[i].long_sent = true;
                }
            }
            poll_long_action(&assistant[i], cfg.long_press_ms);
            poll_long_action(&capture[i], cfg.long_press_ms);
        }
    }
}

void button_actions_init(void)
{
    s_queue = xQueueCreate(DONGLE_MAX_CONTROLLERS, sizeof(button_action_msg_t));
    configASSERT(s_queue != NULL);
    xTaskCreate(button_actions_task, "button_actions", 3072, NULL, 2, NULL);
}

void button_actions_on_state(uint8_t gamepad_index, const stadia_controller_state_t *state)
{
    if (!s_queue || !state) return;
    button_action_msg_t msg = {
        .gamepad_index = gamepad_index,
        .state = *state,
    };
    if (xQueueSendToBack(s_queue, &msg, 0) != pdTRUE) {
        button_action_msg_t stale;
        xQueueReceive(s_queue, &stale, 0);
        xQueueSendToBack(s_queue, &msg, 0);
    }
}
