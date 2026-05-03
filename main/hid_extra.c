#include "hid_extra.h"

#include "dongle_config.h"
#include "dongle_state.h"
#include "status_led.h"

#include "soc/usb_dwc_struct.h"
#include "tinyusb.h"
#include "class/hid/hid.h"
#include "class/hid/hid_device.h"
#include "device/usbd.h"
#include "device/usbd_pvt.h"
#include "esp_err.h"
#include "esp_log.h"
#include "tusb.h"

#include <string.h>

#define RID_KEYBOARD 1
#define RID_CONSUMER 2
#define HID_EXTRA_ITF_NUM DONGLE_MAX_CONTROLLERS
#define HID_EXTRA_EP_IN 0x83
#define HID_EXTRA_EP_SIZE 16

static const char *TAG = "HID_EXTRA";
static uint8_t s_rhport;
static uint8_t s_ep_in;
static bool s_open;
static uint8_t s_report_buf[HID_EXTRA_EP_SIZE] TU_ATTR_ALIGNED(4);

static const uint8_t s_hid_report_desc[HID_EXTRA_REPORT_DESC_LEN] = {
    0x05, 0x01, 0x09, 0x06, 0xa1, 0x01, 0x85, RID_KEYBOARD,
    0x05, 0x07, 0x19, 0xe0, 0x29, 0xe7, 0x15, 0x00,
    0x25, 0x01, 0x75, 0x01, 0x95, 0x08, 0x81, 0x02,
    0x95, 0x01, 0x75, 0x08, 0x81, 0x01, 0x95, 0x05,
    0x75, 0x01, 0x05, 0x08, 0x19, 0x01, 0x29, 0x05,
    0x91, 0x02, 0x95, 0x01, 0x75, 0x03, 0x91, 0x01,
    0x95, 0x06, 0x75, 0x08, 0x15, 0x00, 0x25, 0x73,
    0x05, 0x07, 0x19, 0x00, 0x29, 0x73, 0x81, 0x00,
    0xc0,
    0x05, 0x0c, 0x09, 0x01, 0xa1, 0x01, 0x85, RID_CONSUMER,
    0x15, 0x00, 0x26, 0xff, 0x03, 0x19, 0x00, 0x2a,
    0xff, 0x03, 0x75, 0x10, 0x95, 0x01, 0x81, 0x00,
    0xc0
};

const uint8_t *hid_extra_report_descriptor(void)
{
    return s_hid_report_desc;
}

void hid_extra_init(void)
{
}

static bool hid_extra_ready(void)
{
    return tud_ready() && s_open && s_ep_in && !usbd_edpt_busy(s_rhport, s_ep_in);
}

static bool hid_extra_send_report(uint8_t report_id, const void *report, uint16_t len)
{
    if (!hid_extra_ready() || len + 1 > sizeof(s_report_buf)) return false;
    s_report_buf[0] = report_id;
    memcpy(&s_report_buf[1], report, len);
    if (!usbd_edpt_claim(s_rhport, s_ep_in)) return false;
    bool ok = usbd_edpt_xfer(s_rhport, s_ep_in, s_report_buf, len + 1);
    if (!ok) usbd_edpt_release(s_rhport, s_ep_in);
    return ok;
}

bool hid_extra_send_key_press(uint8_t modifier, uint8_t keycode)
{
    if (!hid_extra_ready()) {
        ESP_LOGW(TAG, "Keyboard report not ready: tud_ready=%d open=%d ep=0x%02x busy=%d",
                 tud_ready(), s_open, s_ep_in, s_ep_in ? usbd_edpt_busy(s_rhport, s_ep_in) : 0);
        return false;
    }
    uint8_t report[8] = { modifier, 0, keycode, 0, 0, 0, 0, 0 };
    return hid_extra_send_report(RID_KEYBOARD, report, sizeof(report));
}

bool hid_extra_send_key_release(void)
{
    if (!hid_extra_ready()) return false;
    uint8_t report[8] = {0};
    return hid_extra_send_report(RID_KEYBOARD, report, sizeof(report));
}

bool hid_extra_send_consumer_press(uint16_t usage)
{
    if (!hid_extra_ready()) {
        ESP_LOGW(TAG, "Consumer report not ready: tud_ready=%d open=%d ep=0x%02x busy=%d",
                 tud_ready(), s_open, s_ep_in, s_ep_in ? usbd_edpt_busy(s_rhport, s_ep_in) : 0);
        return false;
    }
    return hid_extra_send_report(RID_CONSUMER, &usage, sizeof(usage));
}

bool hid_extra_send_consumer_release(void)
{
    uint16_t usage = 0;
    if (!hid_extra_ready()) return false;
    return hid_extra_send_report(RID_CONSUMER, &usage, sizeof(usage));
}

bool hid_extra_remote_wakeup(void)
{
    dongle_status_t st;
    dongle_state_get_status(&st);
    if (!st.usb_suspended) {
        ESP_LOGW(TAG, "Remote wake skipped: not suspended");
        dongle_state_record_wake_attempt(false);
        return false;
    }

    USB_DWC.dctl_reg.rmtwkupsig = 1;
    vTaskDelay(pdMS_TO_TICKS(5));
    USB_DWC.dctl_reg.rmtwkupsig = 0;

    dongle_state_record_wake_attempt(true);
    status_led_flash_remote_wake();
    ESP_LOGI(TAG, "Remote wake signal sent (host enabled=%d)",
             st.usb_remote_wakeup_enabled);
    return true;
}

uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance)
{
    (void)instance;
    return s_hid_report_desc;
}

void tud_hid_mount_cb(uint8_t instance)
{
    ESP_LOGI(TAG, "HID interface mounted: instance=%u", instance);
}

void tud_hid_umount_cb(uint8_t instance)
{
    ESP_LOGI(TAG, "HID interface unmounted: instance=%u", instance);
}

uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id,
                               hid_report_type_t report_type, uint8_t *buffer,
                               uint16_t reqlen)
{
    (void)instance;
    (void)report_id;
    (void)report_type;
    if (buffer && reqlen) memset(buffer, 0, reqlen);
    return reqlen;
}

void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id,
                           hid_report_type_t report_type, uint8_t const *buffer,
                           uint16_t bufsize)
{
    (void)instance;
    (void)report_id;
    (void)report_type;
    (void)buffer;
    (void)bufsize;
}

static void hid_extra_driver_init(void)
{
    s_open = false;
    s_ep_in = 0;
}

static bool hid_extra_driver_deinit(void)
{
    s_open = false;
    s_ep_in = 0;
    return true;
}

static void hid_extra_driver_reset(uint8_t rhport)
{
    (void)rhport;
    s_open = false;
    s_ep_in = 0;
}

static uint16_t hid_extra_driver_open(uint8_t rhport, tusb_desc_interface_t const *itf_desc,
                                      uint16_t max_len)
{
    if (itf_desc->bInterfaceClass != TUSB_CLASS_HID ||
        itf_desc->bInterfaceNumber != HID_EXTRA_ITF_NUM) {
        return 0;
    }

    uint16_t const drv_len = (uint16_t)(sizeof(tusb_desc_interface_t) +
                                        sizeof(tusb_hid_descriptor_hid_t) +
                                        itf_desc->bNumEndpoints * sizeof(tusb_desc_endpoint_t));
    TU_VERIFY(max_len >= drv_len, 0);

    uint8_t const *p_desc = tu_desc_next(itf_desc);
    if (tu_desc_type(p_desc) != HID_DESC_TYPE_HID) return 0;
    p_desc = tu_desc_next(p_desc);

    uint8_t ep_out = 0;
    uint8_t ep_in = 0;
    TU_ASSERT(usbd_open_edpt_pair(rhport, p_desc, itf_desc->bNumEndpoints,
                                  TUSB_XFER_INTERRUPT, &ep_out, &ep_in), 0);
    (void)ep_out;

    s_rhport = rhport;
    s_ep_in = ep_in;
    s_open = true;
    ESP_LOGI(TAG, "HID utility interface opened: itf=%u ep_in=0x%02x", itf_desc->bInterfaceNumber, s_ep_in);
    return drv_len;
}

static bool hid_extra_driver_control(uint8_t rhport, uint8_t stage,
                                     tusb_control_request_t const *request)
{
    if (stage != CONTROL_STAGE_SETUP) return true;

    if (request->bmRequestType_bit.recipient == TUSB_REQ_RCPT_INTERFACE &&
        tu_u16_low(request->wIndex) == HID_EXTRA_ITF_NUM) {
        if (request->bmRequestType_bit.type == TUSB_REQ_TYPE_STANDARD &&
            request->bRequest == TUSB_REQ_GET_DESCRIPTOR) {
            uint8_t desc_type = tu_u16_high(request->wValue);
            if (desc_type == HID_DESC_TYPE_REPORT) {
                return tud_control_xfer(rhport, request, (void *)(uintptr_t)s_hid_report_desc,
                                        sizeof(s_hid_report_desc));
            }
            if (desc_type == HID_DESC_TYPE_HID) {
                static const uint8_t hid_desc[] = {
                    0x09, 0x21, 0x11, 0x01, 0x00, 0x01, 0x22,
                    HID_EXTRA_REPORT_DESC_LEN & 0xff, HID_EXTRA_REPORT_DESC_LEN >> 8,
                };
                return tud_control_xfer(rhport, request, (void *)(uintptr_t)hid_desc, sizeof(hid_desc));
            }
        }

        if (request->bmRequestType_bit.type == TUSB_REQ_TYPE_CLASS) {
            if (request->bmRequestType_bit.direction == TUSB_DIR_IN) {
                static uint8_t zero[8] = {0};
                uint16_t len = request->wLength < sizeof(zero) ? request->wLength : sizeof(zero);
                return tud_control_xfer(rhport, request, zero, len);
            }
            return tud_control_status(rhport, request);
        }
    }

    return false;
}

static bool hid_extra_driver_xfer(uint8_t rhport, uint8_t ep_addr,
                                  xfer_result_t result, uint32_t xferred_bytes)
{
    (void)rhport;
    if (ep_addr == s_ep_in && result != XFER_RESULT_SUCCESS) {
        ESP_LOGW(TAG, "HID utility transfer failed: result=%d bytes=%lu",
                 result, (unsigned long)xferred_bytes);
    }
    return true;
}

static usbd_class_driver_t const s_hid_extra_driver = {
    .name = "HID_EXTRA",
    .init = hid_extra_driver_init,
    .deinit = hid_extra_driver_deinit,
    .reset = hid_extra_driver_reset,
    .open = hid_extra_driver_open,
    .control_xfer_cb = hid_extra_driver_control,
    .xfer_cb = hid_extra_driver_xfer,
    .xfer_isr = NULL,
    .sof = NULL,
};

usbd_class_driver_t const *hid_extra_class_driver(void)
{
    return &s_hid_extra_driver;
}
