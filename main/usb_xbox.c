/*
 * usb_xbox.c — TinyUSB vendor-class driver presenting as an Xbox 360 wired controller.
 *
 * The Xbox 360 controller uses USB class 0xFF (vendor-specific), not HID.
 * Windows loads xusb22.sys natively when it sees VID=0x045E / PID=0x028E.
 *
 * USB descriptor layout (64 bytes total):
 *   Configuration (9) + Interface 0 (9) + Xbox vendor descriptor (17)
 *   + EP1 IN (7) + EP2 OUT (7)
 *   + Interface 1 / security descriptor (9) + security data (6)
 *
 * Interface 0 (class FF/5D/01) handles all data:
 *   EP1 IN  (0x81): interrupt, 32 bytes, 4 ms  → host reads Xbox input reports
 *   EP2 OUT (0x02): interrupt, 32 bytes, 8 ms  → host writes rumble commands
 *
 * Interface 1 (class FF/5D/03, 0 endpoints): the "security descriptor" that Windows
 * requires to recognise the device as an XInput controller.  The firmware does not
 * actively use this interface; it only needs to appear in the descriptor.
 */

#include "usb_xbox.h"
#include "bridge.h"

#include "esp_log.h"
#include "tinyusb.h"
#include "class/vendor/vendor_device.h"
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

// Total: 9+9+17+7+7+9+6 = 64 bytes
#define XBOX_CFG_LEN 64

static const uint8_t s_cfg_desc[XBOX_CFG_LEN] = {
    // ----- Configuration Descriptor (9) -----
    0x09, 0x02, XBOX_CFG_LEN, 0x00,
    0x02,       // bNumInterfaces = 2
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

    // ----- Xbox 360 proprietary descriptor (17, type 0x21) -----
    // Reverse-engineered from real hardware; references EP1 IN and EP2 OUT.
    0x11, 0x21, 0x00, 0x01, 0x01, 0x25,
    0x81,                               // EP1 IN address
    0x14, 0x03, 0x00, 0x03, 0x13,
    0x02,                               // EP2 OUT address
    0x00, 0x03, 0x00, 0x03,

    // ----- EP1 IN: Interrupt, 32 bytes, 4 ms (7) -----
    0x07, 0x05, 0x81, 0x03, 0x20, 0x00, 0x04,

    // ----- EP2 OUT: Interrupt, 32 bytes, 8 ms (7) -----
    0x07, 0x05, 0x02, 0x03, 0x20, 0x00, 0x08,

    // ----- Interface 1: Security Descriptor (9) -----
    // Windows requires this interface (class FF/5D/03) to load xusb22.sys.
    // No endpoints; firmware does not service this interface.
    0x09, 0x04,
    0x01,       // bInterfaceNumber
    0x00,       // bAlternateSetting
    0x00,       // bNumEndpoints
    0xff,       // bInterfaceClass
    0x5d,       // bInterfaceSubClass
    0x03,       // bInterfaceProtocol
    0x00,       // iInterface

    // ----- Security descriptor data (6, type 0x41) -----
    0x06, 0x41, 0x00, 0x01, 0x01, 0x03,
};

// String descriptors: [0] language, [1] manufacturer, [2] product, [3] serial
static const char *s_str_desc[] = {
    (char[]){0x09, 0x04}, // [0] Language: English (0x0409)
    "Microsoft",          // [1] iManufacturer
    "Controller",         // [2] iProduct
    "000000000001",       // [3] iSerialNumber
};

/* ---- Rumble receive (EP2 OUT → usb_to_ble queue) ------------------------ */

/*
 * Xbox 360 rumble output report (8 bytes):
 *   [0] 0x00
 *   [1] 0x08 (length)
 *   [2] 0x00
 *   [3] Large motor (0–255)
 *   [4] Small motor (0–255)
 *   [5–7] 0x00
 *
 * Mapped to 4-byte Stadia rumble payload (uint16 LE strong, uint16 LE weak):
 *   [0] 0x00  [1] large_motor   [2] 0x00  [3] small_motor
 */
void tud_vendor_rx_cb(uint8_t itf, uint8_t const *buffer, uint16_t bufsize)
{
    if (bufsize >= 5 && buffer[0] == 0x00 && buffer[1] == 0x08) {
        uint8_t rumble[4] = {0x00, buffer[3], 0x00, buffer[4]};
        xQueueSendToBack(usb_to_ble_queue, rumble, 0);
    }
}

/* ---- USB Xbox task ------------------------------------------------------- */

void usb_xbox_task(void *arg)
{
    uint8_t report[20];
    while (1) {
        if (xQueueReceive(ble_to_usb_queue, report, pdMS_TO_TICKS(10))) {
            if (tud_connected() && tud_vendor_n_write_available(0) >= 20) {
                tud_vendor_n_write(0, report, 20);
                tud_vendor_n_flush(0);
            }
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
