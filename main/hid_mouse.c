#include "hid_mouse.h"

#include "dongle_config.h"

#include "class/hid/hid.h"
#include "device/usbd.h"
#include "device/usbd_pvt.h"
#include "esp_log.h"
#include "tusb.h"

static const char *TAG = "MOUSE";

static uint8_t s_rhport;
static uint8_t s_ep_in;
static bool s_open;
static uint8_t s_report_buf[DONGLE_MOUSE_EP_SIZE] TU_ATTR_ALIGNED(4);

static const uint8_t s_hid_report_desc[HID_MOUSE_REPORT_DESC_LEN] = {
    0x05, 0x01, 0x09, 0x02, 0xa1, 0x01,
    0x09, 0x01, 0xa1, 0x00,
    0x05, 0x09, 0x19, 0x01, 0x29, 0x03,
    0x15, 0x00, 0x25, 0x01, 0x95, 0x03, 0x75, 0x01,
    0x81, 0x02,
    0x95, 0x01, 0x75, 0x05, 0x81, 0x01,
    0x05, 0x01, 0x09, 0x30, 0x09, 0x31, 0x09, 0x38,
    0x15, 0x81, 0x25, 0x7f, 0x75, 0x08, 0x95, 0x03,
    0x81, 0x06,
    0xc0,
    0xc0
};

static const uint8_t s_hid_desc[] = {
    0x09, 0x21, 0x11, 0x01, 0x00, 0x01, 0x22,
    HID_MOUSE_REPORT_DESC_LEN & 0xff, HID_MOUSE_REPORT_DESC_LEN >> 8,
};

const uint8_t *hid_mouse_report_descriptor(void)
{
    return s_hid_report_desc;
}

void hid_mouse_init(void)
{
}

bool hid_mouse_send_report(uint8_t buttons, int8_t x, int8_t y, int8_t wheel)
{
    if (!tud_ready() || !s_open || !s_ep_in || usbd_edpt_busy(s_rhport, s_ep_in)) return false;

    s_report_buf[0] = buttons & 0x07;
    s_report_buf[1] = (uint8_t)(int8_t)x;
    s_report_buf[2] = (uint8_t)(int8_t)y;
    s_report_buf[3] = (uint8_t)(int8_t)wheel;

    if (!usbd_edpt_claim(s_rhport, s_ep_in)) return false;
    bool ok = usbd_edpt_xfer(s_rhport, s_ep_in, s_report_buf, 4);
    if (!ok) usbd_edpt_release(s_rhport, s_ep_in);
    return ok;
}

static void hid_mouse_driver_init(void)
{
    s_open = false;
    s_ep_in = 0;
}

static bool hid_mouse_driver_deinit(void)
{
    s_open = false;
    s_ep_in = 0;
    return true;
}

static void hid_mouse_driver_reset(uint8_t rhport)
{
    (void)rhport;
    s_open = false;
    s_ep_in = 0;
}

static uint16_t hid_mouse_driver_open(uint8_t rhport, tusb_desc_interface_t const *itf_desc,
                                      uint16_t max_len)
{
    if (itf_desc->bInterfaceClass != TUSB_CLASS_HID ||
        itf_desc->bInterfaceNumber != DONGLE_MOUSE_ITF_NUM) {
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
    ESP_LOGI(TAG, "HID mouse interface opened: itf=%u ep_in=0x%02x",
             itf_desc->bInterfaceNumber, s_ep_in);
    return drv_len;
}

static bool hid_mouse_driver_control(uint8_t rhport, uint8_t stage,
                                     tusb_control_request_t const *request)
{
    if (stage != CONTROL_STAGE_SETUP) return true;

    if (request->bmRequestType_bit.recipient == TUSB_REQ_RCPT_INTERFACE &&
        tu_u16_low(request->wIndex) == DONGLE_MOUSE_ITF_NUM) {
        if (request->bmRequestType_bit.type == TUSB_REQ_TYPE_STANDARD &&
            request->bRequest == TUSB_REQ_GET_DESCRIPTOR) {
            uint8_t desc_type = tu_u16_high(request->wValue);
            if (desc_type == HID_DESC_TYPE_REPORT) {
                return tud_control_xfer(rhport, request, (void *)(uintptr_t)s_hid_report_desc,
                                        sizeof(s_hid_report_desc));
            }
            if (desc_type == HID_DESC_TYPE_HID) {
                return tud_control_xfer(rhport, request, (void *)(uintptr_t)s_hid_desc,
                                        sizeof(s_hid_desc));
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

static bool hid_mouse_driver_xfer(uint8_t rhport, uint8_t ep_addr,
                                  xfer_result_t result, uint32_t xferred_bytes)
{
    (void)rhport;
    if (ep_addr == s_ep_in && result != XFER_RESULT_SUCCESS) {
        ESP_LOGW(TAG, "HID mouse transfer failed: result=%d bytes=%lu",
                 result, (unsigned long)xferred_bytes);
    }
    return true;
}

static usbd_class_driver_t const s_hid_mouse_driver = {
    .name = "HID_MOUSE",
    .init = hid_mouse_driver_init,
    .deinit = hid_mouse_driver_deinit,
    .reset = hid_mouse_driver_reset,
    .open = hid_mouse_driver_open,
    .control_xfer_cb = hid_mouse_driver_control,
    .xfer_cb = hid_mouse_driver_xfer,
    .xfer_isr = NULL,
    .sof = NULL,
};

usbd_class_driver_t const *hid_mouse_class_driver(void)
{
    return &s_hid_mouse_driver;
}
