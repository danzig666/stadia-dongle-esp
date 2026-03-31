/*
 * usb_xbox.c — Xbox 360 wired controller USB descriptors and input task.
 *
 * The Xbox 360 controller uses USB class 0xFF (vendor-specific), not HID.
 * Windows loads xusb22.sys natively when it sees VID=0x045E / PID=0x028E.
 *
 * USB descriptor layout (48 bytes total):
 *   Configuration (9) + Interface 0 (9) + Xbox vendor descriptor (16)
 *   + EP1 IN (7) + EP2 OUT (7)
 *
 * Interface 0 (class FF/5D/01) handles all data:
 *   EP1 IN  (0x81): interrupt, 32 bytes, 4 ms  → host reads Xbox input reports
 *   EP2 OUT (0x02): interrupt, 32 bytes, 8 ms  → host writes rumble commands
 *
 * Descriptor matches fluffymadness/tinyusb-xinput (verified working with
 * xusb22.sys). Single interface, 16-byte 0x21 Xbox vendor descriptor.
 *
 * Endpoint transfers are handled by the custom class driver in xbox_dev.c,
 * which uses direct usbd_edpt_xfer() calls instead of the TinyUSB vendor
 * class FIFO (the vendor class uses bulk semantics that stall on interrupt EPs).
 */

#include "usb_xbox.h"
#include "xbox_dev.h"
#include "bridge.h"

#include "esp_log.h"
#include "tinyusb.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "USB";

/* ---- USB descriptors ---------------------------------------------------- */

static const tusb_desc_device_t s_device_desc = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,
    .bDeviceClass       = 0xFF,
    .bDeviceSubClass    = 0xFF,
    .bDeviceProtocol    = 0xFF,
    .bMaxPacketSize0    = 64,
    .idVendor           = 0x045E, // Microsoft
    .idProduct          = 0x028E, // Xbox 360 wired controller
    .bcdDevice          = 0x0114,
    .iManufacturer      = 1,
    .iProduct           = 2,
    .iSerialNumber      = 3,
    .bNumConfigurations = 1,
};

// Total: 9+9+16+7+7 = 48 bytes (fluffymadness/tinyusb-xinput layout)
#define XBOX_CFG_LEN 48

static const uint8_t s_cfg_desc[XBOX_CFG_LEN] = {
    // ----- Configuration Descriptor (9) -----
    0x09, 0x02, 0x30, 0x00,
    0x01,       // bNumInterfaces = 1
    0x01,       // bConfigurationValue
    0x00,       // iConfiguration
    0xa0,       // bmAttributes: bus-powered, remote-wakeup
    0xfa,       // bMaxPower: 500 mA

    // ----- Interface 0: Xbox 360 data (9) -----
    0x09, 0x04,
    0x00,       // bInterfaceNumber
    0x00,       // bAlternateSetting
    0x02,       // bNumEndpoints
    0xff,       // bInterfaceClass
    0x5d,       // bInterfaceSubClass
    0x01,       // bInterfaceProtocol
    0x00,       // iInterface

    // ----- Xbox vendor descriptor (16, type 0x21) — fluffymadness bytes -----
    0x10, 0x21, 0x10, 0x01, 0x01, 0x24,
    0x81,                               // EP1 IN address
    0x14, 0x03, 0x00, 0x03, 0x13,
    0x02,                               // EP2 OUT address
    0x00, 0x03, 0x00,

    // ----- EP1 IN: Interrupt, 32 bytes, 4 ms (7) -----
    0x07, 0x05, 0x81, 0x03, 0x20, 0x00, 0x04,

    // ----- EP2 OUT: Interrupt, 32 bytes, 8 ms (7) -----
    0x07, 0x05, 0x02, 0x03, 0x20, 0x00, 0x08,
};

// String descriptors: [0] language, [1] manufacturer, [2] product, [3] serial
static const char *s_str_desc[] = {
    (char[]){0x09, 0x04}, // [0] Language: English (0x0409)
    "Microsoft",          // [1] iManufacturer
    "Controller",         // [2] iProduct
    "000000000001",       // [3] iSerialNumber
};

/* ---- USB Xbox task ------------------------------------------------------- */

void usb_xbox_task(void *arg)
{
    uint8_t report[20];
    while (1) {
        if (xQueueReceive(ble_to_usb_queue, report, pdMS_TO_TICKS(10))) {
            // Drain to latest report — for a gamepad only the newest state matters
            while (xQueueReceive(ble_to_usb_queue, report, 0)) {}

            bool sent = xbox_send_report(report);
            #if DONGLE_DEBUG
            if (sent) {
                ESP_LOG_BUFFER_HEX_LEVEL(TAG, report, 20, ESP_LOG_INFO);
            } else {
                ESP_LOGW(TAG, "EP busy or disconnected");
            }
            #endif
        }
    }
}

/* ---- Init ---------------------------------------------------------------- */

void usb_xbox_init(void)
{
    tinyusb_config_t cfg = {
        .port = TINYUSB_PORT_FULL_SPEED_0,
        .phy  = { .skip_setup = false, .self_powered = false },
        .task = { .size = 4096, .priority = 4, .xCoreID = 1 },
        .descriptor = {
            .device           = &s_device_desc,
            .qualifier        = NULL,
            .string           = s_str_desc,
            .string_count     = 4,
            .full_speed_config = s_cfg_desc,
            .high_speed_config = NULL,
        },
        .event_cb  = NULL,
        .event_arg = NULL,
    };
    ESP_ERROR_CHECK(tinyusb_driver_install(&cfg));
    ESP_LOGI(TAG, "Xbox 360 USB initialised (VID=045E PID=028E)");
}
