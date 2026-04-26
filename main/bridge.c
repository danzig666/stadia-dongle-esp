/*
 * bridge.c — Translation between Stadia BLE input reports and Xbox 360 USB reports.
 *
 * Stadia BLE notification payload (10 bytes, no Report ID; byte 9 unused):
 *   [0] D-pad hat (0=Up … 7=Up-L, >7=neutral)
 *   [1] bits 7=RS, 6=OPTIONS, 5=MENU, 4=STADIA_BTN
 *   [2] bits 6=A, 5=B, 4=X, 3=Y, 2=LB, 1=RB, 0=LS
 *   [3] Left stick X  (0–255)
 *   [4] Left stick Y  (0–255, 0=up)
 *   [5] Right stick X (0–255)
 *   [6] Right stick Y (0–255, 0=up)
 *   [7] Left trigger  (0–255)
 *   [8] Right trigger (0–255)
 *
 * Xbox 360 USB input report (20 bytes):
 *   [0]    0x00 (packet type)
 *   [1]    0x14 (packet length = 20)
 *   [2]    d-pad bits 0=Up,1=Down,2=Left,3=Right | btn bits 4=Start,5=Back,6=LS,7=RS
 *   [3]    btn bits 0=LB,1=RB,2=Guide,4=A,5=B,6=X,7=Y
 *   [4]    Left trigger  (0–255)
 *   [5]    Right trigger (0–255)
 *   [6–7]  Left stick X  (int16 LE, positive=right)
 *   [8–9]  Left stick Y  (int16 LE, positive=up)
 *   [10–11] Right stick X (int16 LE)
 *   [12–13] Right stick Y (int16 LE, positive=up)
 *   [14–19] 0x00 padding
 */

#include "bridge.h"
#include <string.h>

QueueHandle_t ble_to_usb_queue;
QueueHandle_t usb_to_ble_queue;

void bridge_init(void)
{
    ble_to_usb_queue = xQueueCreate(8, 20);
    usb_to_ble_queue = xQueueCreate(4, 4);
}

void bridge_send_neutral(void)
{
    uint8_t neutral[20] = {0x00, 0x14, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    xQueueReset(ble_to_usb_queue);
    xQueueSendToBack(ble_to_usb_queue, neutral, 0);
}

// Stadia hat-switch (0–7) → Xbox 360 d-pad bitmask (bits 0=Up,1=Down,2=Left,3=Right)
static const uint8_t dpad_map[8] = {
    0x01, // Up
    0x09, // Up+Right
    0x08, // Right
    0x0A, // Down+Right
    0x02, // Down
    0x06, // Down+Left
    0x04, // Left
    0x05, // Up+Left
};

// Map 0–255 unsigned stick value to -32767..+32767 signed, optionally inverting axis.
// Clamps center dead-zone: raw 128 → 0, raw 0/255 → ±32767.
static int16_t map_stick(uint8_t v, int invert)
{
    int c = (int)v - 128;
    if (c < -127) c = -127;
    if (invert) c = -c;
    return (int16_t)(32767 * c / 127);
}

void stadia_to_xbox360(const uint8_t *s, uint8_t *x)
{
    memset(x, 0, 20);
    x[0] = 0x00; // packet type
    x[1] = 0x14; // packet length = 20

    // Byte 2: d-pad + Start / Back / LS / RS
    uint8_t b2 = (s[0] < 8) ? dpad_map[s[0]] : 0;
    if (s[1] & (1 << 5)) b2 |= (1 << 4); // MENU    → Start
    if (s[1] & (1 << 6)) b2 |= (1 << 5); // OPTIONS → Back
    if (s[2] & (1 << 0)) b2 |= (1 << 6); // LS
    if (s[1] & (1 << 7)) b2 |= (1 << 7); // RS
    x[2] = b2;

    // Byte 3: LB / RB / Guide / A / B / X / Y
    uint8_t b3 = 0;
    if (s[2] & (1 << 2)) b3 |= (1 << 0); // LB
    if (s[2] & (1 << 1)) b3 |= (1 << 1); // RB
    if (s[1] & (1 << 4)) b3 |= (1 << 2); // STADIA_BTN → Guide
    if (s[2] & (1 << 6)) b3 |= (1 << 4); // A
    if (s[2] & (1 << 5)) b3 |= (1 << 5); // B
    if (s[2] & (1 << 4)) b3 |= (1 << 6); // X
    if (s[2] & (1 << 3)) b3 |= (1 << 7); // Y
    x[3] = b3;

    x[4] = s[7]; // left trigger
    x[5] = s[8]; // right trigger

    int16_t lx = map_stick(s[3], 0);
    int16_t ly = map_stick(s[4], 1); // Stadia Y=0 is up → invert for Xbox (positive=up)
    int16_t rx = map_stick(s[5], 0);
    int16_t ry = map_stick(s[6], 1);

    x[6]  = (uint8_t)(lx & 0xFF); x[7]  = (uint8_t)(lx >> 8);
    x[8]  = (uint8_t)(ly & 0xFF); x[9]  = (uint8_t)(ly >> 8);
    x[10] = (uint8_t)(rx & 0xFF); x[11] = (uint8_t)(rx >> 8);
    x[12] = (uint8_t)(ry & 0xFF); x[13] = (uint8_t)(ry >> 8);
    // bytes 14–19 already zero
}
