/*
 * xbox_dev.c — Custom TinyUSB class driver for Xbox 360 controller endpoints.
 *
 * The generic TinyUSB vendor class uses bulk-oriented FIFOs internally, which
 * don't drain correctly on interrupt endpoints.  This driver bypasses the
 * vendor class entirely and uses usbd_edpt_xfer() directly on EP1 IN
 * (input reports) and EP2 OUT (rumble commands).
 *
 * Registration: TinyUSB discovers this driver via usbd_app_driver_get_cb().
 */

#include "xbox_dev.h"
#include "bridge.h"

#include "esp_log.h"
#include "tinyusb.h"
#include "device/usbd.h"
#include "device/usbd_pvt.h"

#include <string.h>

static const char *TAG = "XBOX_DEV";

#define XBOX_EP_IN   0x81
#define XBOX_EP_OUT  0x02
#define XBOX_EP_SIZE 32

static uint8_t  s_in_buf[XBOX_EP_SIZE]  TU_ATTR_ALIGNED(4);
static uint8_t  s_out_buf[XBOX_EP_SIZE] TU_ATTR_ALIGNED(4);
static volatile bool s_rhport_ready = false;
static uint8_t  s_rhport = 0;

/* ---- Class driver callbacks --------------------------------------------- */

static void xbox_init(void) {
    s_rhport_ready = false;
}

static bool xbox_deinit(void) {
    s_rhport_ready = false;
    return true;
}

static void xbox_reset(uint8_t rhport) {
    (void)rhport;
    s_rhport_ready = false;
}

/*
 * Open Xbox 360 interface and its endpoints.
 *
 * Fixed-size layout (fluffymadness approach):
 *   interface(9) + Xbox_desc(16) + 2*endpoint(7) = 39 bytes
 */
static uint16_t xbox_open(uint8_t rhport, tusb_desc_interface_t const *itf_desc,
                           uint16_t max_len)
{
    if (itf_desc->bInterfaceClass    != 0xFF ||
        itf_desc->bInterfaceSubClass != 0x5D) {
        return 0; // not ours
    }

    // Fixed layout: interface(9) + Xbox_desc(16) + 2*endpoint(7) = 39
    uint16_t const drv_len = (uint16_t)(sizeof(tusb_desc_interface_t)
                             + itf_desc->bNumEndpoints * sizeof(tusb_desc_endpoint_t)
                             + 16);
    TU_VERIFY(max_len >= drv_len, 0);

    s_rhport = rhport;
    uint8_t const *p_desc = tu_desc_next(itf_desc); // skip interface descriptor
    uint8_t found = 0;

    while (found < itf_desc->bNumEndpoints) {
        if (tu_desc_type(p_desc) == TUSB_DESC_ENDPOINT) {
            tusb_desc_endpoint_t const *ep = (tusb_desc_endpoint_t const *)p_desc;
            TU_ASSERT(usbd_edpt_open(rhport, ep), 0);
            ESP_LOGI(TAG, "Opened EP 0x%02x", ep->bEndpointAddress);

            if (ep->bEndpointAddress == XBOX_EP_OUT) {
                if (usbd_edpt_claim(rhport, XBOX_EP_OUT)) {
                    if (!usbd_edpt_xfer(rhport, XBOX_EP_OUT, s_out_buf, XBOX_EP_SIZE)) {
                        usbd_edpt_release(rhport, XBOX_EP_OUT);
                    }
                }
            }
            found++;
        }
        p_desc = tu_desc_next(p_desc);
    }

    s_rhport_ready = true;
    ESP_LOGI(TAG, "Opened (%u bytes claimed)", drv_len);
    return drv_len;
}

static bool xbox_control_xfer_cb(uint8_t rhport, uint8_t stage,
                                  tusb_control_request_t const *request)
{
    (void)rhport; (void)stage; (void)request;
    return true; // ACK all class/standard requests (don't stall xusb22.sys)
}

/*
 * Transfer-complete callback (runs in TinyUSB task context).
 */
static bool xbox_xfer_cb(uint8_t rhport, uint8_t ep_addr,
                          xfer_result_t result, uint32_t xferred_bytes)
{
    #if DONGLE_DEBUG
    ESP_LOGI(TAG, "xfer_cb ep=0x%02x result=%d bytes=%lu",
             ep_addr, result, (unsigned long)xferred_bytes);
    #endif
    (void)result;

    if (ep_addr == XBOX_EP_IN) {
        // busy cleared automatically by TinyUSB; nothing to do

    } else if (ep_addr == XBOX_EP_OUT) {
        // Xbox 360 rumble: [0]=0x00 [1]=0x08 [2]=0x00 [3]=large [4]=small
        if (xferred_bytes >= 5 && s_out_buf[0] == 0x00 && s_out_buf[1] == 0x08) {
            uint8_t rumble[4] = { 0x00, s_out_buf[3], 0x00, s_out_buf[4] };
            if (xQueueSendToBack(usb_to_ble_queue, rumble, 0) != pdTRUE) {
                uint8_t dummy[4];
                xQueueReceive(usb_to_ble_queue, dummy, 0);
                xQueueSendToBack(usb_to_ble_queue, rumble, 0);
            }
        }
        // Re-arm OUT endpoint
        if (usbd_edpt_claim(rhport, XBOX_EP_OUT)) {
            if (!usbd_edpt_xfer(rhport, XBOX_EP_OUT, s_out_buf, XBOX_EP_SIZE)) {
                usbd_edpt_release(rhport, XBOX_EP_OUT);
            }
        }
    }
    return true;
}

/* ---- Driver registration ------------------------------------------------ */

static usbd_class_driver_t const s_xbox_driver = {
    .name             = "XBOX360",
    .init             = xbox_init,
    .deinit           = xbox_deinit,
    .reset            = xbox_reset,
    .open             = xbox_open,
    .control_xfer_cb  = xbox_control_xfer_cb,
    .xfer_cb          = xbox_xfer_cb,
    .xfer_isr         = NULL,
    .sof              = NULL,
};

usbd_class_driver_t const *usbd_app_driver_get_cb(uint8_t *driver_count) {
    *driver_count = 1;
    return &s_xbox_driver;
}

/* ---- Vendor control request handler (overrides TinyUSB weak default) ---- */

/*
 * xusb22.sys sends vendor-type control requests during initialisation.
 * The TinyUSB default (weak) returns false → STALL, which makes the Windows
 * XInput driver give up and stop polling EP1 IN.  ACK them instead.
 */
bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage,
                                 tusb_control_request_t const *request)
{
    #if DONGLE_DEBUG
    if (stage == CONTROL_STAGE_SETUP) {
        ESP_LOGI(TAG, "Vendor ctrl: bReq=0x%02x wVal=0x%04x wIdx=0x%04x wLen=%d dir=%d",
                 request->bRequest, request->wValue, request->wIndex,
                 request->wLength, request->bmRequestType_bit.direction);
    }
    #endif

    if (stage != CONTROL_STAGE_SETUP) return true;

    if (request->bmRequestType_bit.direction == TUSB_DIR_IN) {
        // Host wants data — send zeros (satisfies handshake without stalling)
        static uint8_t buf[32] = {0};
        uint16_t len = request->wLength < sizeof(buf) ? request->wLength : sizeof(buf);
        return tud_control_xfer(rhport, request, buf, len);
    }
    // Host is sending data or no-data — ACK
    return tud_control_status(rhport, request);
}

/* ---- Public API --------------------------------------------------------- */

int xbox_send_report(const uint8_t *report) {
    if (!tud_connected() || !s_rhport_ready) return -1;
    if (usbd_edpt_busy(s_rhport, XBOX_EP_IN)) return 0;

    memcpy(s_in_buf, report, 20);
    if (usbd_edpt_claim(s_rhport, XBOX_EP_IN)) {
        bool ok = usbd_edpt_xfer(s_rhport, XBOX_EP_IN, s_in_buf, 20);
        if (!ok) {
            usbd_edpt_release(s_rhport, XBOX_EP_IN);
        }
        return ok ? 1 : 0;
    }
    return 0;
}