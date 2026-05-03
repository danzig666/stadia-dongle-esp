/*
 * usb_xbox.c - USB descriptors for multiple Xbox 360-style gamepad
 * interfaces plus optional HID utility interface.
 */

#include "usb_xbox.h"

#include "bridge.h"
#include "dongle_config.h"
#include "dongle_state.h"
#include "hid_extra.h"
#include "hid_mouse.h"
#include "web_server.h"
#include "xbox_dev.h"

#include "esp_log.h"
#include "tinyusb.h"
#include "tusb.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

static const char *TAG = "USB";
static SemaphoreHandle_t s_flag_lock;
static bool s_last_polled_suspended;
static volatile bool s_resync_reports_requested;
static volatile bool s_reenumerate_requested;

static void set_resync_requested(bool val)
{
    if (s_flag_lock) xSemaphoreTake(s_flag_lock, portMAX_DELAY);
    s_resync_reports_requested = val;
    if (s_flag_lock) xSemaphoreGive(s_flag_lock);
}

static void set_reenumerate_requested(bool val)
{
    if (s_flag_lock) xSemaphoreTake(s_flag_lock, portMAX_DELAY);
    s_reenumerate_requested = val;
    if (s_flag_lock) xSemaphoreGive(s_flag_lock);
}

static bool get_and_clear_resync_requested(void)
{
    if (!s_flag_lock) return false;
    xSemaphoreTake(s_flag_lock, portMAX_DELAY);
    bool val = s_resync_reports_requested;
    s_resync_reports_requested = false;
    xSemaphoreGive(s_flag_lock);
    return val;
}

static bool get_and_clear_reenumerate_requested(void)
{
    if (!s_flag_lock) return false;
    xSemaphoreTake(s_flag_lock, portMAX_DELAY);
    bool val = s_reenumerate_requested;
    s_reenumerate_requested = false;
    xSemaphoreGive(s_flag_lock);
    return val;
}

static void request_usb_report_resync(void)
{
    set_resync_requested(true);
}

static void enqueue_latest_reports(void)
{
    dongle_status_t st;
    dongle_state_get_status(&st);
    for (uint8_t i = 0; i < DONGLE_MAX_CONTROLLERS; i++) {
        const dongle_controller_status_t *c = &st.controllers[i];
        if (!c->connected && !c->xbox_valid) continue;

        ble_to_usb_msg_t msg = {
            .gamepad_index = c->usb_gamepad_index,
            .len = DONGLE_XBOX_REPORT_SIZE,
        };
        ble_to_usb_msg_t neutral = {
            .gamepad_index = c->usb_gamepad_index,
            .data = {0x00, 0x14},
            .len = DONGLE_XBOX_REPORT_SIZE,
        };
        xQueueSendToBack(ble_to_usb_queue, &neutral, 0);

        if (c->xbox_valid) {
            memcpy(msg.data, c->xbox, DONGLE_XBOX_REPORT_SIZE);
        } else {
            msg.data[0] = 0x00;
            msg.data[1] = 0x14;
        }
        xQueueSendToBack(ble_to_usb_queue, &msg, 0);
    }
}

static void tinyusb_event_handler(tinyusb_event_t *event, void *arg)
{
    (void)arg;
    if (!event) return;

    switch (event->id) {
    case TINYUSB_EVENT_ATTACHED:
        dongle_state_set_usb_configured(true);
        dongle_state_set_usb_suspended(false);
        request_usb_report_resync();
        ESP_LOGI(TAG, "USB attached/configured");
        break;
    case TINYUSB_EVENT_DETACHED:
        dongle_state_set_usb_configured(false);
        dongle_state_set_usb_suspended(false);
        dongle_state_set_usb_remote_wakeup_enabled(false);
        ESP_LOGI(TAG, "USB detached");
        break;
#ifdef CONFIG_TINYUSB_SUSPEND_CALLBACK
    case TINYUSB_EVENT_SUSPENDED:
        dongle_state_set_usb_suspended(true);
        dongle_state_set_usb_remote_wakeup_enabled(event->suspended.remote_wakeup);
        set_resync_requested(false);
        set_reenumerate_requested(false);
        web_server_notify_usb_suspend(true);
        ESP_LOGI(TAG, "USB suspended, remote_wakeup_enabled=%d", event->suspended.remote_wakeup);
        break;
#endif
#ifdef CONFIG_TINYUSB_RESUME_CALLBACK
    case TINYUSB_EVENT_RESUMED:
        dongle_state_set_usb_suspended(false);
        set_reenumerate_requested(true);
        request_usb_report_resync();
        web_server_notify_usb_suspend(false);
        ESP_LOGI(TAG, "USB resumed");
        break;
#endif
    default:
        break;
    }
}

static void usb_state_monitor_task(void *arg)
{
    (void)arg;
    while (1) {
        bool suspended = tud_suspended();
        if (suspended != s_last_polled_suspended) {
            s_last_polled_suspended = suspended;
            dongle_state_set_usb_suspended(suspended);
            web_server_notify_usb_suspend(suspended);
            if (suspended) {
                /* Polled suspend: assume remote wake is allowed; the event-
                   driven TINYUSB_EVENT_SUSPENDED callback will refine this
                   if it fires. Also clear any pending resync so it won't
                   trigger a re-enumeration that breaks the suspend state. */
                dongle_state_set_usb_remote_wakeup_enabled(true);
                set_resync_requested(false);
                set_reenumerate_requested(false);
            } else {
                dongle_state_set_usb_remote_wakeup_enabled(false);
                set_reenumerate_requested(true);
                request_usb_report_resync();
            }
            ESP_LOGI(TAG, "USB poll state: suspended=%d ready=%d mounted=%d",
                     suspended, tud_ready(), tud_mounted());
        }
        if (get_and_clear_resync_requested() && !tud_suspended() && tud_ready()) {
            if (get_and_clear_reenumerate_requested()) {
                ESP_LOGW(TAG, "USB soft re-enumeration after host resume");
                tud_disconnect();
                vTaskDelay(pdMS_TO_TICKS(300));
                tud_connect();
                vTaskDelay(pdMS_TO_TICKS(900));
            } else {
                vTaskDelay(pdMS_TO_TICKS(120));
            }
            xbox_recover_after_resume();
            enqueue_latest_reports();
            ESP_LOGI(TAG, "USB reports resynchronised after resume/attach");
        }
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

static const tusb_desc_device_t s_device_desc = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0210,
    .bDeviceClass = 0xEF,
    .bDeviceSubClass = 0x02,
    .bDeviceProtocol = 0x01,
    .bMaxPacketSize0 = 64,
    .idVendor = 0x045E,
    .idProduct = 0x0289,
    .bcdDevice = 0x0114,
    .iManufacturer = 1,
    .iProduct = 2,
    .iSerialNumber = 3,
    .bNumConfigurations = 1,
};

#define XBOX_FUNCTION_LEN 47
#define HID_UTILITY_FUNCTION_LEN 33
#define HID_MOUSE_FUNCTION_LEN 33
#define XBOX_CFG_LEN (9 + (DONGLE_MAX_CONTROLLERS * XBOX_FUNCTION_LEN) + \
                     (DONGLE_ENABLE_UTILITY_HID ? HID_UTILITY_FUNCTION_LEN : 0) + \
                     (DONGLE_ENABLE_MOUSE_HID ? HID_MOUSE_FUNCTION_LEN : 0))
#define USB_NUM_INTERFACES (DONGLE_MAX_CONTROLLERS + (DONGLE_ENABLE_UTILITY_HID ? 1 : 0) + (DONGLE_ENABLE_MOUSE_HID ? 1 : 0))

static const uint8_t s_cfg_desc[XBOX_CFG_LEN] = {
    0x09, 0x02, XBOX_CFG_LEN & 0xff, XBOX_CFG_LEN >> 8,
    USB_NUM_INTERFACES,
    0x01,
    0x00,
    0xa0,
    0xfa,

    /* Xbox gamepad 0: interface 0, EP1 IN, EP2 OUT. */
    0x08, 0x0B, 0x00, 0x01, 0xff, 0x5d, 0x01, 0x00,
    0x09, 0x04, 0x00, 0x00, 0x02, 0xff, 0x5d, 0x01, 0x00,
    0x10, 0x21, 0x10, 0x01, 0x01, 0x24, 0x81, 0x14,
    0x03, 0x00, 0x03, 0x13, 0x02, 0x00, 0x03, 0x00,
    0x07, 0x05, 0x81, 0x03, 0x20, 0x00, 0x04,
    0x07, 0x05, 0x02, 0x03, 0x20, 0x00, 0x08,

#if DONGLE_MAX_CONTROLLERS >= 2
    /* Xbox gamepad 1: interface 1, EP4 IN, EP5 OUT. */
    0x08, 0x0B, 0x01, 0x01, 0xff, 0x5d, 0x01, 0x00,
    0x09, 0x04, 0x01, 0x00, 0x02, 0xff, 0x5d, 0x01, 0x00,
    0x10, 0x21, 0x10, 0x01, 0x01, 0x24, 0x84, 0x14,
    0x03, 0x00, 0x03, 0x13, 0x05, 0x00, 0x03, 0x00,
    0x07, 0x05, 0x84, 0x03, 0x20, 0x00, 0x04,
    0x07, 0x05, 0x05, 0x03, 0x20, 0x00, 0x08,
#endif

#if DONGLE_ENABLE_UTILITY_HID
    /*
     * HID utility: interface DONGLE_MAX_CONTROLLERS, EP3 IN.
     * Advertise as a boot keyboard so Windows is more likely to expose and
     * arm this interface for USB wake. The report descriptor still carries
     * keyboard and consumer-control reports for normal operation.
     */
    0x08, 0x0B, DONGLE_MAX_CONTROLLERS, 0x01, 0x03, 0x01, 0x01, 0x00,
    0x09, 0x04, DONGLE_MAX_CONTROLLERS, 0x00, 0x01, 0x03, 0x01, 0x01, 0x00,
    0x09, 0x21, 0x11, 0x01, 0x00, 0x01, 0x22,
    HID_EXTRA_REPORT_DESC_LEN & 0xff, HID_EXTRA_REPORT_DESC_LEN >> 8,
     0x07, 0x05, 0x83, 0x03, 0x10, 0x00, 0x0a,
#endif

#if DONGLE_ENABLE_MOUSE_HID
    /*
     * HID mouse: interface DONGLE_MOUSE_ITF_NUM, EP6 IN.
     */
    0x08, 0x0B, DONGLE_MOUSE_ITF_NUM, 0x01, 0x03, 0x01, 0x02, 0x00,
    0x09, 0x04, DONGLE_MOUSE_ITF_NUM, 0x00, 0x01, 0x03, 0x01, 0x02, 0x00,
    0x09, 0x21, 0x11, 0x01, 0x00, 0x01, 0x22,
    HID_MOUSE_REPORT_DESC_LEN & 0xff, HID_MOUSE_REPORT_DESC_LEN >> 8,
    0x07, 0x05, DONGLE_MOUSE_EP_IN, 0x03, DONGLE_MOUSE_EP_SIZE, 0x00, 0x02,
#endif
};

static const char *s_str_desc[] = {
    (char[]){0x09, 0x04},
    "Microsoft",
    "Controller",
    "000000000001",
};

void usb_xbox_task(void *arg)
{
    (void)arg;
    ble_to_usb_msg_t msg = {0};
    ble_to_usb_msg_t pending[DONGLE_MAX_CONTROLLERS] = {0};
    bool have_report[DONGLE_MAX_CONTROLLERS] = {0};
    TickType_t busy_since[DONGLE_MAX_CONTROLLERS] = {0};
    TickType_t last_sent[DONGLE_MAX_CONTROLLERS] = {0};

    while (1) {
        if (xQueueReceive(ble_to_usb_queue, &msg, pdMS_TO_TICKS(2)) == pdTRUE) {
            if (msg.gamepad_index < DONGLE_MAX_CONTROLLERS) {
                pending[msg.gamepad_index] = msg;
                have_report[msg.gamepad_index] = true;
            }
        }

        while (xQueueReceive(ble_to_usb_queue, &msg, 0) == pdTRUE) {
            if (msg.gamepad_index < DONGLE_MAX_CONTROLLERS) {
                pending[msg.gamepad_index] = msg;
                have_report[msg.gamepad_index] = true;
            }
        }

        for (uint8_t i = 0; i < DONGLE_MAX_CONTROLLERS; i++) {
            if (!have_report[i]) continue;
            int res = xbox_send_report(i, pending[i].data, pending[i].len);
            if (res == 1 || res == -1) {
                have_report[i] = false;
                last_sent[i] = xTaskGetTickCount();
                busy_since[i] = 0;
            } else if (res == 0) {
                TickType_t now = xTaskGetTickCount();
                if (busy_since[i] == 0) busy_since[i] = now;
                if (now - busy_since[i] > pdMS_TO_TICKS(1200)) {
                    ESP_LOGW(TAG, "usb %u IN endpoint stuck busy; forcing USB soft reconnect", i);
                    busy_since[i] = now;
                    set_reenumerate_requested(true);
                    set_resync_requested(true);
                }
            }
        }

        // Keepalive: if no Xbox report was sent for 1000 ms, send a neutral
        // one to clear any stale Guide button state that Windows might latch.
        TickType_t now = xTaskGetTickCount();
        for (uint8_t i = 0; i < DONGLE_MAX_CONTROLLERS; i++) {
            if (!have_report[i] && (now - last_sent[i]) > pdMS_TO_TICKS(1000) &&
                tud_ready() && !tud_suspended()) {
                uint8_t neutral[DONGLE_XBOX_REPORT_SIZE];
                memset(neutral, 0, sizeof(neutral));
                neutral[0] = 0x00;
                neutral[1] = 0x14;
                int res = xbox_send_report(i, neutral, sizeof(neutral));
                if (res == 1) last_sent[i] = now;
            }
        }
    }
}

void usb_xbox_init(void)
{
    s_flag_lock = xSemaphoreCreateMutex();
    configASSERT(s_flag_lock != NULL);

    tinyusb_config_t cfg = {
        .port = TINYUSB_PORT_FULL_SPEED_0,
        .phy = { .skip_setup = false, .self_powered = false },
        .task = { .size = 4096, .priority = 4, .xCoreID = 1 },
        .descriptor = {
            .device = &s_device_desc,
            .qualifier = NULL,
            .string = s_str_desc,
            .string_count = 4,
            .full_speed_config = s_cfg_desc,
            .high_speed_config = NULL,
        },
        .event_cb = tinyusb_event_handler,
        .event_arg = NULL,
    };
    ESP_ERROR_CHECK(tinyusb_driver_install(&cfg));
    xTaskCreate(usb_state_monitor_task, "usb_state", 2048, NULL, 2, NULL);
    ESP_LOGI(TAG, "Xbox USB initialised controllers=%u utility_hid=%u mouse_hid=%u cfg_len=%u",
             (unsigned)DONGLE_MAX_CONTROLLERS, (unsigned)DONGLE_ENABLE_UTILITY_HID,
             (unsigned)DONGLE_ENABLE_MOUSE_HID, (unsigned)XBOX_CFG_LEN);
}

#define MS_OS_20_SET_HEADER_DESCRIPTOR        0x00
#define MS_OS_20_SUBSET_HEADER_CONFIGURATION  0x01
#define MS_OS_20_SUBSET_HEADER_FUNCTION       0x02
#define MS_OS_20_FEATURE_COMPATBLE_ID         0x03

#define MS_OS_20_FUNCTION_LEN 0x1C
#define MS_OS_20_DESC_LEN (0x0A + 0x08 + (DONGLE_MAX_CONTROLLERS * MS_OS_20_FUNCTION_LEN))

const uint8_t s_ms_os_20_desc[MS_OS_20_DESC_LEN] = {
    U16_TO_U8S_LE(0x000A), U16_TO_U8S_LE(MS_OS_20_SET_HEADER_DESCRIPTOR),
    U32_TO_U8S_LE(0x06030000), U16_TO_U8S_LE(MS_OS_20_DESC_LEN),

    U16_TO_U8S_LE(0x0008), U16_TO_U8S_LE(MS_OS_20_SUBSET_HEADER_CONFIGURATION),
    0, 0, U16_TO_U8S_LE(MS_OS_20_DESC_LEN - 0x0A),

    U16_TO_U8S_LE(0x0008), U16_TO_U8S_LE(MS_OS_20_SUBSET_HEADER_FUNCTION),
    0, 0, U16_TO_U8S_LE(MS_OS_20_FUNCTION_LEN),
    U16_TO_U8S_LE(0x0014), U16_TO_U8S_LE(MS_OS_20_FEATURE_COMPATBLE_ID),
    'X', 'U', 'S', 'B', '2', '0', 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

#if DONGLE_MAX_CONTROLLERS >= 2
    U16_TO_U8S_LE(0x0008), U16_TO_U8S_LE(MS_OS_20_SUBSET_HEADER_FUNCTION),
    1, 0, U16_TO_U8S_LE(MS_OS_20_FUNCTION_LEN),
    U16_TO_U8S_LE(0x0014), U16_TO_U8S_LE(MS_OS_20_FEATURE_COMPATBLE_ID),
    'X', 'U', 'S', 'B', '2', '0', 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
#endif
};

#define BOS_TOTAL_LEN (TUD_BOS_DESC_LEN + TUD_BOS_MICROSOFT_OS_DESC_LEN)

static const uint8_t s_bos_desc[] = {
    TUD_BOS_DESCRIPTOR(BOS_TOTAL_LEN, 1),
    TUD_BOS_MS_OS_20_DESCRIPTOR(MS_OS_20_DESC_LEN, 1)
};

uint8_t const *tud_descriptor_bos_cb(void)
{
    return s_bos_desc;
}
