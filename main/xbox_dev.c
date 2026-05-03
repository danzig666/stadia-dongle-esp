/*
 * xbox_dev.c - Custom TinyUSB class driver for one Xbox 360 endpoint pair per
 * virtual gamepad.
 */

#include "xbox_dev.h"

#include "bridge.h"
#include "dongle_config.h"
#include "hid_extra.h"
#include "hid_mouse.h"

#include "esp_log.h"
#include "tinyusb.h"
#include "device/usbd.h"
#include "device/usbd_pvt.h"

#include <string.h>

static const char *TAG = "XBOX_DEV";

#define XBOX_EP_SIZE 32
#define XBOX_INTERFACE_BASE 0

typedef struct {
    uint8_t index;
    uint8_t interface_number;
    uint8_t ep_in;
    uint8_t ep_out;
    bool open;
    tusb_desc_endpoint_t desc_in;
    tusb_desc_endpoint_t desc_out;
    uint8_t in_buf[XBOX_EP_SIZE] TU_ATTR_ALIGNED(4);
    uint8_t out_buf[XBOX_EP_SIZE] TU_ATTR_ALIGNED(4);
} xbox_gamepad_instance_t;

static xbox_gamepad_instance_t s_gamepads[DONGLE_MAX_CONTROLLERS] = {
    {
        .index = 0,
        .interface_number = 0,
        .ep_in = 0x81,
        .ep_out = 0x02,
    },
#if DONGLE_MAX_CONTROLLERS >= 2
    {
        .index = 1,
        .interface_number = 1,
        .ep_in = 0x84,
        .ep_out = 0x05,
    },
#endif
};

static volatile bool s_rhport_ready;
static uint8_t s_rhport;

static xbox_gamepad_instance_t *gamepad_by_interface(uint8_t itf)
{
    for (uint8_t i = 0; i < DONGLE_MAX_CONTROLLERS; i++) {
        if (s_gamepads[i].interface_number == itf) return &s_gamepads[i];
    }
    return NULL;
}

static xbox_gamepad_instance_t *gamepad_by_ep(uint8_t ep_addr)
{
    for (uint8_t i = 0; i < DONGLE_MAX_CONTROLLERS; i++) {
        if (s_gamepads[i].ep_in == ep_addr || s_gamepads[i].ep_out == ep_addr) {
            return &s_gamepads[i];
        }
    }
    return NULL;
}

static void arm_out(uint8_t rhport, xbox_gamepad_instance_t *gp)
{
    if (!gp || !gp->open) return;
    if (usbd_edpt_claim(rhport, gp->ep_out)) {
        if (!usbd_edpt_xfer(rhport, gp->ep_out, gp->out_buf, XBOX_EP_SIZE)) {
            usbd_edpt_release(rhport, gp->ep_out);
        }
    }
}

static void xbox_init(void)
{
    s_rhport_ready = false;
    for (uint8_t i = 0; i < DONGLE_MAX_CONTROLLERS; i++) s_gamepads[i].open = false;
}

static bool xbox_deinit(void)
{
    xbox_init();
    return true;
}

static void xbox_reset(uint8_t rhport)
{
    (void)rhport;
    xbox_init();
}

static uint16_t xbox_open(uint8_t rhport, tusb_desc_interface_t const *itf_desc,
                          uint16_t max_len)
{
    if (itf_desc->bInterfaceClass != 0xFF || itf_desc->bInterfaceSubClass != 0x5D) {
        return 0;
    }

    xbox_gamepad_instance_t *gp = gamepad_by_interface(itf_desc->bInterfaceNumber);
    if (!gp) return 0;

    uint16_t const drv_len = (uint16_t)(sizeof(tusb_desc_interface_t) +
                             itf_desc->bNumEndpoints * sizeof(tusb_desc_endpoint_t) + 16);
    TU_VERIFY(max_len >= drv_len, 0);

    s_rhport = rhport;
    uint8_t const *p_desc = tu_desc_next(itf_desc);
    uint8_t found = 0;

    while (found < itf_desc->bNumEndpoints) {
        if (tu_desc_type(p_desc) == TUSB_DESC_ENDPOINT) {
            tusb_desc_endpoint_t const *ep = (tusb_desc_endpoint_t const *)p_desc;
            TU_ASSERT(usbd_edpt_open(rhport, ep), 0);
            if ((ep->bEndpointAddress & 0x80) != 0) {
                gp->desc_in = *ep;
                gp->ep_in = ep->bEndpointAddress;
            } else {
                gp->desc_out = *ep;
                gp->ep_out = ep->bEndpointAddress;
            }
            ESP_LOGI(TAG, "usb %u opened EP 0x%02x", gp->index, ep->bEndpointAddress);
            found++;
        }
        p_desc = tu_desc_next(p_desc);
    }

    gp->open = true;
    s_rhport_ready = true;
    arm_out(rhport, gp);
    ESP_LOGI(TAG, "usb %u interface %u opened (%u bytes claimed)",
             gp->index, itf_desc->bInterfaceNumber, drv_len);
    return drv_len;
}

static bool xbox_control_xfer_cb(uint8_t rhport, uint8_t stage,
                                 tusb_control_request_t const *request)
{
    (void)rhport;
    (void)stage;
    (void)request;
    return true;
}

static bool xbox_xfer_cb(uint8_t rhport, uint8_t ep_addr,
                         xfer_result_t result, uint32_t xferred_bytes)
{
    xbox_gamepad_instance_t *gp = gamepad_by_ep(ep_addr);
    if (!gp) return true;
    if (result != XFER_RESULT_SUCCESS) {
        ESP_LOGW(TAG, "usb %u xfer failed ep=0x%02x result=%d", gp->index, ep_addr, result);
    }

    if (ep_addr == gp->ep_out) {
        if (xferred_bytes >= 5 && gp->out_buf[0] == 0x00 && gp->out_buf[1] == 0x08) {
            usb_to_ble_msg_t msg = {
                .gamepad_index = gp->index,
                .data = {0x00, gp->out_buf[3], 0x00, gp->out_buf[4]},
                .len = DONGLE_RUMBLE_REPORT_SIZE,
            };
            if (xQueueSendToBack(usb_to_ble_queue, &msg, 0) != pdTRUE) {
                usb_to_ble_msg_t dummy;
                xQueueReceive(usb_to_ble_queue, &dummy, 0);
                xQueueSendToBack(usb_to_ble_queue, &msg, 0);
            }
            ESP_LOGI(TAG, "usb %u rumble out large=%u small=%u", gp->index, gp->out_buf[3], gp->out_buf[4]);
        }
        arm_out(rhport, gp);
    }
    return true;
}

static usbd_class_driver_t const s_xbox_driver = {
    .name = "XBOX360",
    .init = xbox_init,
    .deinit = xbox_deinit,
    .reset = xbox_reset,
    .open = xbox_open,
    .control_xfer_cb = xbox_control_xfer_cb,
    .xfer_cb = xbox_xfer_cb,
    .xfer_isr = NULL,
    .sof = NULL,
};

usbd_class_driver_t const *usbd_app_driver_get_cb(uint8_t *driver_count)
{
    static usbd_class_driver_t drivers[3];
    uint8_t count = 0;
    drivers[count++] = s_xbox_driver;
#if DONGLE_ENABLE_UTILITY_HID
    drivers[count++] = *hid_extra_class_driver();
#endif
#if DONGLE_ENABLE_MOUSE_HID
    drivers[count++] = *hid_mouse_class_driver();
#endif
    *driver_count = count;
    return drivers;
}

bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage,
                                tusb_control_request_t const *request)
{
    if (stage != CONTROL_STAGE_SETUP) return true;

    if (request->bmRequestType_bit.type == TUSB_REQ_TYPE_VENDOR &&
        request->bRequest == 1 && request->wIndex == 7) {
        extern const uint8_t s_ms_os_20_desc[];
        uint16_t total_len;
        memcpy(&total_len, s_ms_os_20_desc + 8, 2);
        return tud_control_xfer(rhport, request, (void *)(uintptr_t)s_ms_os_20_desc, total_len);
    }

    if (request->bmRequestType_bit.direction == TUSB_DIR_IN) {
        static uint8_t buf[32] = {0};
        uint16_t len = request->wLength < sizeof(buf) ? request->wLength : sizeof(buf);
        return tud_control_xfer(rhport, request, buf, len);
    }
    return tud_control_status(rhport, request);
}

int xbox_send_report(uint8_t gamepad_index, const uint8_t *report, size_t len)
{
    if (gamepad_index >= DONGLE_MAX_CONTROLLERS || !report || len < DONGLE_XBOX_REPORT_SIZE) return -1;
    xbox_gamepad_instance_t *gp = &s_gamepads[gamepad_index];
    if (!tud_connected() || !tud_ready() || tud_suspended() || !s_rhport_ready || !gp->open) {
        return -1;
    }
    if (usbd_edpt_busy(s_rhport, gp->ep_in)) return 0;

    memcpy(gp->in_buf, report, DONGLE_XBOX_REPORT_SIZE);
    if (usbd_edpt_claim(s_rhport, gp->ep_in)) {
        bool ok = usbd_edpt_xfer(s_rhport, gp->ep_in, gp->in_buf, DONGLE_XBOX_REPORT_SIZE);
        if (!ok) usbd_edpt_release(s_rhport, gp->ep_in);
        return ok ? 1 : 0;
    }
    return 0;
}

void xbox_recover_after_resume(void)
{
    if (!tud_connected() || !tud_ready() || tud_suspended() || !s_rhport_ready) return;

    for (uint8_t i = 0; i < DONGLE_MAX_CONTROLLERS; i++) {
        xbox_gamepad_instance_t *gp = &s_gamepads[i];
        if (!gp->open) continue;

        if (usbd_edpt_stalled(s_rhport, gp->ep_in)) usbd_edpt_clear_stall(s_rhport, gp->ep_in);
        if (usbd_edpt_stalled(s_rhport, gp->ep_out)) usbd_edpt_clear_stall(s_rhport, gp->ep_out);
        memset(gp->out_buf, 0, sizeof(gp->out_buf));
        arm_out(s_rhport, gp);
        ESP_LOGW(TAG, "usb %u endpoints refreshed after resume", gp->index);
    }
}
