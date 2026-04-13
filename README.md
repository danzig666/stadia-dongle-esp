# Stadia Controller Dongle

Presents a Google Stadia controller as an Xbox 360 gamepad over USB. The ESP32-S3
connects to the controller via BLE and appears to the host PC as a wired Xbox 360
controller — no drivers required on Windows, Linux, or macOS.

Plug the dongle into your PC, hold the Stadia button on the controller to pair, and it works.

### Why a dongle?

After Google shut down Stadia, they released a firmware update enabling the controller's
Bluetooth mode. On Windows, input works fine but **rumble does not** — and it cannot be
fixed in software.

The root cause is a Windows kernel driver limitation: the Windows BLE HID driver holds
exclusive access to the controller's GATT service, blocking all output reports. Neither
`WriteFile` (returns error 87 — the BLE stack sends a write-without-response which the
Stadia firmware rejects) nor `HidD_SetOutputReport` (fails silently) can send the rumble
command. Direct GATT access via WinRT is also denied while the HID driver is active, and
disabling the driver drops the Bluetooth connection entirely.

The ESP32-S3 sidesteps all of this. Its BLE stack correctly issues a GATT
write-with-response for the rumble characteristic — exactly what the Stadia firmware
requires. The host PC only sees a standard wired Xbox 360 controller over USB, so no
special software or drivers are needed on the PC side at all.

### Hardware

Any ESP32-S3 board with native USB (USB OTG on GPIO19/20). The USB port connected
to the host must be the native USB port, not UART.

### Requirements

- [ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/get-started/) v5.x
- Target: `esp32s3`

### Build & flash

```sh
cd stadia-dongle
idf.py build
idf.py -p <PORT> flash
```

### Pairing

The dongle bonds to the first Stadia controller it sees. The bond is stored in NVS
(flash) and survives power cycles. To pair a different controller, erase NVS:

```sh
idf.py -p <PORT> erase-flash
```

### Button mapping

| Stadia          | Xbox 360     |
|-----------------|--------------|
| A / B / X / Y  | A / B / X / Y |
| LB / RB        | LB / RB      |
| LT / RT        | LT / RT      |
| LS / RS        | LS / RS      |
| Menu           | Start        |
| Options        | Back         |
| Stadia button  | Guide        |
| D-pad          | D-pad        |

Rumble is fully supported in both directions.

### Assistant button — hold 3 s

Hold the Assistant button for 3 seconds to toggle rumble on/off. A short haptic
pattern confirms the state change.

### Debug logging

In `bridge.h`, set `DONGLE_DEBUG 1` to enable raw HID report hex dumps over UART.
Set to `0` for production builds (the `sdkconfig.defaults` silences the log output
at the IDF level anyway).

---
