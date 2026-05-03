#include "config_store.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "nvs.h"
#include "nvs_flash.h"

#include <string.h>

#define CFG_NS "dongle_cfg"
#define CFG_KEY "config"
#define CFG_SCHEMA 3

static const char *TAG = "CFG";

static SemaphoreHandle_t s_cfg_lock;
static dongle_config_t s_cfg;

void config_store_defaults(dongle_config_t *cfg)
{
    memset(cfg, 0, sizeof(*cfg));
    cfg->schema_version = CFG_SCHEMA;
    cfg->assistant_short_action = DONGLE_ACTION_KEY_F14;
    cfg->assistant_long_action = DONGLE_ACTION_START_WEBUI;
    cfg->capture_short_action = DONGLE_ACTION_KEY_PRINTSCREEN;
    cfg->capture_long_action = DONGLE_ACTION_NONE;
    cfg->long_press_ms = 1000;
    cfg->webui_auto_start_if_no_bond = true;
    cfg->webui_timeout_after_ble_connected_ms = 120000;
    cfg->disable_ap_on_usb_suspend = true;
}

void config_store_init(void)
{
    s_cfg_lock = xSemaphoreCreateMutex();
    configASSERT(s_cfg_lock != NULL);
    config_store_defaults(&s_cfg);
    nvs_handle_t nvs;
    if (nvs_open(CFG_NS, NVS_READWRITE, &nvs) != ESP_OK) return;
    size_t len = sizeof(s_cfg);
    esp_err_t err = nvs_get_blob(nvs, CFG_KEY, &s_cfg, &len);
    if (err == ESP_OK && s_cfg.schema_version == 1) {
        s_cfg.schema_version = CFG_SCHEMA;
        s_cfg.webui_timeout_after_ble_connected_ms = 120000;
        s_cfg.disable_ap_on_usb_suspend = true;
        nvs_set_blob(nvs, CFG_KEY, &s_cfg, sizeof(s_cfg));
        nvs_commit(nvs);
    } else if (err == ESP_OK && s_cfg.schema_version == 2) {
        s_cfg.schema_version = CFG_SCHEMA;
        s_cfg.disable_ap_on_usb_suspend = true;
        nvs_set_blob(nvs, CFG_KEY, &s_cfg, sizeof(s_cfg));
        nvs_commit(nvs);
    } else if (err != ESP_OK || s_cfg.schema_version != CFG_SCHEMA) {
        config_store_defaults(&s_cfg);
        nvs_set_blob(nvs, CFG_KEY, &s_cfg, sizeof(s_cfg));
        nvs_commit(nvs);
    }
    nvs_close(nvs);
}

void config_store_get(dongle_config_t *out)
{
    if (!out) return;
    if (s_cfg_lock) xSemaphoreTake(s_cfg_lock, portMAX_DELAY);
    *out = s_cfg;
    if (s_cfg_lock) xSemaphoreGive(s_cfg_lock);
}

void config_store_set(const dongle_config_t *cfg)
{
    if (!cfg) return;
    if (s_cfg_lock) xSemaphoreTake(s_cfg_lock, portMAX_DELAY);
    s_cfg = *cfg;
    s_cfg.schema_version = CFG_SCHEMA;
    if (s_cfg_lock) xSemaphoreGive(s_cfg_lock);
    nvs_handle_t nvs;
    if (nvs_open(CFG_NS, NVS_READWRITE, &nvs) == ESP_OK) {
        esp_err_t err = nvs_set_blob(nvs, CFG_KEY, &s_cfg, sizeof(s_cfg));
        if (err == ESP_OK) err = nvs_commit(nvs);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Saved keymap: assistant short=%s long=%s capture short=%s long=%s",
                     config_store_action_name(s_cfg.assistant_short_action),
                     config_store_action_name(s_cfg.assistant_long_action),
                     config_store_action_name(s_cfg.capture_short_action),
                     config_store_action_name(s_cfg.capture_long_action));
        } else {
            ESP_LOGW(TAG, "Failed to save config: %s", esp_err_to_name(err));
        }
        nvs_close(nvs);
    } else {
        ESP_LOGW(TAG, "Failed to open NVS namespace %s", CFG_NS);
    }
}

const char *config_store_action_name(uint8_t action)
{
    switch ((dongle_action_t)action) {
    case DONGLE_ACTION_NONE: return "none";
    case DONGLE_ACTION_KEY_F13: return "f13";
    case DONGLE_ACTION_KEY_F14: return "f14";
    case DONGLE_ACTION_KEY_PRINTSCREEN: return "printscreen";
    case DONGLE_ACTION_KEY_ESCAPE: return "escape";
    case DONGLE_ACTION_KEY_SPACE: return "space";
    case DONGLE_ACTION_KEY_ENTER: return "enter";
    case DONGLE_ACTION_CONSUMER_VOLUME_UP: return "volume_up";
    case DONGLE_ACTION_CONSUMER_VOLUME_DOWN: return "volume_down";
    case DONGLE_ACTION_CONSUMER_PLAY_PAUSE: return "play_pause";
    case DONGLE_ACTION_START_WEBUI: return "start_webui";
    case DONGLE_ACTION_REMOTE_WAKE_ONLY: return "remote_wake_only";
    case DONGLE_ACTION_KEY_F15: return "f15";
    case DONGLE_ACTION_KEY_F16: return "f16";
    case DONGLE_ACTION_KEY_F17: return "f17";
    case DONGLE_ACTION_KEY_F18: return "f18";
    case DONGLE_ACTION_KEY_F19: return "f19";
    case DONGLE_ACTION_KEY_F20: return "f20";
    case DONGLE_ACTION_KEY_F21: return "f21";
    case DONGLE_ACTION_KEY_F22: return "f22";
    case DONGLE_ACTION_KEY_F23: return "f23";
    case DONGLE_ACTION_KEY_F24: return "f24";
    case DONGLE_ACTION_KEY_TAB: return "tab";
    case DONGLE_ACTION_KEY_BACKSPACE: return "backspace";
    case DONGLE_ACTION_KEY_INSERT: return "insert";
    case DONGLE_ACTION_KEY_DELETE: return "delete";
    case DONGLE_ACTION_KEY_HOME: return "home";
    case DONGLE_ACTION_KEY_END: return "end";
    case DONGLE_ACTION_KEY_PAGE_UP: return "page_up";
    case DONGLE_ACTION_KEY_PAGE_DOWN: return "page_down";
    case DONGLE_ACTION_KEY_ARROW_UP: return "arrow_up";
    case DONGLE_ACTION_KEY_ARROW_DOWN: return "arrow_down";
    case DONGLE_ACTION_KEY_ARROW_LEFT: return "arrow_left";
    case DONGLE_ACTION_KEY_ARROW_RIGHT: return "arrow_right";
    case DONGLE_ACTION_CONSUMER_MUTE: return "mute";
    case DONGLE_ACTION_CONSUMER_SCAN_NEXT: return "next_track";
    case DONGLE_ACTION_CONSUMER_SCAN_PREV: return "prev_track";
    default: return "none";
    }
}

uint8_t config_store_action_from_name(const char *name)
{
    if (!name) return DONGLE_ACTION_NONE;
    if (strcmp(name, "f13") == 0) return DONGLE_ACTION_KEY_F13;
    if (strcmp(name, "f14") == 0) return DONGLE_ACTION_KEY_F14;
    if (strcmp(name, "printscreen") == 0) return DONGLE_ACTION_KEY_PRINTSCREEN;
    if (strcmp(name, "escape") == 0) return DONGLE_ACTION_KEY_ESCAPE;
    if (strcmp(name, "space") == 0) return DONGLE_ACTION_KEY_SPACE;
    if (strcmp(name, "enter") == 0) return DONGLE_ACTION_KEY_ENTER;
    if (strcmp(name, "volume_up") == 0) return DONGLE_ACTION_CONSUMER_VOLUME_UP;
    if (strcmp(name, "volume_down") == 0) return DONGLE_ACTION_CONSUMER_VOLUME_DOWN;
    if (strcmp(name, "play_pause") == 0) return DONGLE_ACTION_CONSUMER_PLAY_PAUSE;
    if (strcmp(name, "start_webui") == 0) return DONGLE_ACTION_START_WEBUI;
    if (strcmp(name, "remote_wake_only") == 0) return DONGLE_ACTION_REMOTE_WAKE_ONLY;
    if (strcmp(name, "f15") == 0) return DONGLE_ACTION_KEY_F15;
    if (strcmp(name, "f16") == 0) return DONGLE_ACTION_KEY_F16;
    if (strcmp(name, "f17") == 0) return DONGLE_ACTION_KEY_F17;
    if (strcmp(name, "f18") == 0) return DONGLE_ACTION_KEY_F18;
    if (strcmp(name, "f19") == 0) return DONGLE_ACTION_KEY_F19;
    if (strcmp(name, "f20") == 0) return DONGLE_ACTION_KEY_F20;
    if (strcmp(name, "f21") == 0) return DONGLE_ACTION_KEY_F21;
    if (strcmp(name, "f22") == 0) return DONGLE_ACTION_KEY_F22;
    if (strcmp(name, "f23") == 0) return DONGLE_ACTION_KEY_F23;
    if (strcmp(name, "f24") == 0) return DONGLE_ACTION_KEY_F24;
    if (strcmp(name, "tab") == 0) return DONGLE_ACTION_KEY_TAB;
    if (strcmp(name, "backspace") == 0) return DONGLE_ACTION_KEY_BACKSPACE;
    if (strcmp(name, "insert") == 0) return DONGLE_ACTION_KEY_INSERT;
    if (strcmp(name, "delete") == 0) return DONGLE_ACTION_KEY_DELETE;
    if (strcmp(name, "home") == 0) return DONGLE_ACTION_KEY_HOME;
    if (strcmp(name, "end") == 0) return DONGLE_ACTION_KEY_END;
    if (strcmp(name, "page_up") == 0) return DONGLE_ACTION_KEY_PAGE_UP;
    if (strcmp(name, "page_down") == 0) return DONGLE_ACTION_KEY_PAGE_DOWN;
    if (strcmp(name, "arrow_up") == 0) return DONGLE_ACTION_KEY_ARROW_UP;
    if (strcmp(name, "arrow_down") == 0) return DONGLE_ACTION_KEY_ARROW_DOWN;
    if (strcmp(name, "arrow_left") == 0) return DONGLE_ACTION_KEY_ARROW_LEFT;
    if (strcmp(name, "arrow_right") == 0) return DONGLE_ACTION_KEY_ARROW_RIGHT;
    if (strcmp(name, "mute") == 0) return DONGLE_ACTION_CONSUMER_MUTE;
    if (strcmp(name, "next_track") == 0) return DONGLE_ACTION_CONSUMER_SCAN_NEXT;
    if (strcmp(name, "prev_track") == 0) return DONGLE_ACTION_CONSUMER_SCAN_PREV;
    return DONGLE_ACTION_NONE;
}
