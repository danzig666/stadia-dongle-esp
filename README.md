# Stadia Controller Dongle

ESP32-S3 firmware that bridges Google Stadia controllers (Bluetooth LE) to a PC
as a wired **Xbox 360 controller** + **HID mouse** + **HID keyboard**. Wi-Fi
captive portal for pairing, button remapping, and OTA updates.

Forked from [Scalee/stadia-dongle](https://github.com/Scalee/stadia-dongle).

## Requirements

- **ESP32-S3** module with native USB OTG
- **4 MB** flash (default) or 16 MB - configurable via `sdkconfig.defaults`
- Optional: WS2812/SK6812 RGB LED on GPIO 48 (configurable in menuconfig)
- [ESP-IDF v6.x](https://github.com/espressif/esp-idf)

## Build

```sh
idf.py set-target esp32s3
idf.py build
```

`sdkconfig.defaults` pre-configures NimBLE central, TinyUSB, 240 MHz CPU, and
production logging. The default partition table fits 4 MB flash with dual OTA
slots.

### Flash

```sh
idf.py -p <PORT> flash
```

Merged binary for the [ESP Web Tools](https://esphome.github.io/esp-web-tools/)
installer:

```sh
esptool.py --chip esp32s3 merge_bin \
  -o docs/firmware-merged.bin \
  --flash_mode dio --flash_freq 80m --flash_size 4MB \
  0x0 build/bootloader/bootloader.bin \
  0x8000 build/partition_table/partition-table.bin \
  0x20000 build/stadia-dongle.bin
```

Host the `docs/` folder and open `docs/index.html` in Chrome/Edge.

## Setup

1. Flash the firmware.
2. On first boot, the dongle starts a setup AP automatically.
3. Join Wi-Fi **`StadiaDongle-XXXX`** (open, no password).
4. Open `http://192.168.4.1` (captive portal should redirect).
5. Click **Start Pairing**, then put the controller in Bluetooth mode
   (hold **Stadia + Y** until orange flash). This is not really needed, it auto pairs available controllers.
6. Once bonded, the dongle auto-connects on future boots.

## Controller Modes

### Gamepad Mode

The controller appears as an **Xbox 360 Controller for Windows** (VID 0x045E,
PID 0x0289). Button mapping:

| Stadia          | Xbox 360  |
|-----------------|-----------|
| A / B / X / Y   | A / B / X / Y |
| LB / RB         | LB / RB |
| LT / RT         | LT / RT (analog) |
| LS / RS click   | LS / RS |
| Left Stick      | Left Stick |
| Right Stick     | Right Stick |
| D-pad           | D-pad |
| Menu            | Start |
| Options         | Back |
| Stadia button   | Guide |

### Mouse Mode

Hold **Assistant + Capture** for 2 seconds to toggle. Rumble confirms: 2 pulses
= mouse ON, 3 pulses = controller mode restored. Always starts in controller
mode after power-on. While active, the Xbox gamepad interface sends neutral
reports (sticks centered, buttons released) and all input is routed to the HID
mouse.

| Stadia input   | Mouse action                                       |
|----------------|----------------------------------------------------|
| Left stick     | Cursor movement (deadzone, acceleration curve, smoothing) |
| A              | Left click                                         |
| B              | Right click                                        |
| X              | Middle click                                       |
| D-pad up/down  | Scroll wheel (±1 per press)                        |
| Right stick Y  | Analog scroll wheel                                |
| LT (hold)      | Precision mode - cursor speed ÷ 3                  |
| RT (hold)      | Fast mode - cursor speed × 2                       |
| Stadia / LB / RB / LS / RS / Menu / Options | Inactive (neutral Xbox reports) |

Mouse reports are driven by a 125 Hz hardware timer for smooth, jitter-free movement.

### Multi-Controller

Up to **2 controllers** can be paired and used concurrently. Each appears as a
separate Xbox 360 gamepad. Mouse mode and button mappings are per-controller.

BLE connection interval is negotiated for lowest latency:

| Controllers | Connection interval | Update rate |
|-------------|---------------------|-------------|
| 1           | 7.5 ms              | ~133 Hz     |
| 2           | 11 – 15 ms          | ~67 – 91 Hz |

When the Wi-Fi AP is active, BLE scanning pauses to reduce coexistence impact;
existing controller connections and notifications are unaffected.

### Extra Buttons

Assistant and Capture buttons are exposed as a **boot keyboard + consumer-control**
HID interface. Defaults:

| Button    | Short press | Long press (1 s) |
|-----------|-------------|------------------|
| Assistant | F14         | Start Web UI     |
| Capture   | PrintScreen | *(none)*         |

All actions are remappable in the Web UI. 38 actions available: F13–F24,
PrintScreen, Escape, Space, Enter, Tab, Backspace, Insert, Delete, Home, End,
Page Up/Down, arrow keys, volume/mute, media playback, remote wake only, and
Start Web UI.

## Web UI

Access at `http://192.168.4.1` when the AP is active. Provides:

- BLE and USB connection status, firmware version
- Per-controller battery, name, address, mouse mode status
- Live button state and raw report bytes
- Pairing controls and bonded controller list
- Button action remapping, long-press duration, Web UI timeout
- Firmware update upload (OTA) - untested
- Reboot and disable-Web-UI

The AP starts on fresh flash, when no bonds exist, or by holding
**Assistant** button. It auto-disables after a configurable
timeout (default 120 s) when a controller is connected.

## USB Remote Wake

When the host PC sleeps, pressing **Stadia, Assistant, Capture, or A** sends a
remote wake signal via `tud_remote_wakeup()`. Requires BIOS/OS USB wake support. Not always working. 

## LED Reference

| Pattern                        | Meaning |
|--------------------------------|---------|
| White pulse                    | Booting |
| Dim blue heartbeat             | Scanning, no controller connected |
| Blue-yellow alternating        | Scanning + Wi-Fi AP on |
| Green heartbeat                | 1 controller in gamepad mode |
| Faster green heartbeat         | 2 controllers in gamepad mode |
| Orange heartbeat               | At least 1 controller in mouse mode |
| Dim purple heartbeat           | PC sleeping (USB suspended) |
| Red pulsing                    | Error |
| Purple/yellow alternating      | OTA firmware update |
| Brief white flash              | Remote wake sent |

## Architecture

```
Stadia BLE ──▶ ble_central.c ──▶ bridge.c ──▶ usb_xbox.c ──▶ Host PC (Xbox 360)
                   │
                   ├──▶ button_actions.c ──▶ hid_extra.c ──▶ Host PC (keyboard)
                   │
                   └──▶ mouse_mode.c ──▶ hid_mouse.c ──▶ Host PC (mouse, 125 Hz)

Host PC (rumble) ──▶ xbox_dev.c ──▶ ble_central.c ──▶ Stadia BLE
```

## License

[GNU General Public License v3.0](LICENSE)
