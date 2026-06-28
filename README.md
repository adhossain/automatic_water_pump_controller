# Water Tank Level Controller

An IoT water tank monitoring and pump control system built on **ESP32** with **Tuya Cloud** integration. Automatically manages rooftop water tank levels with real-time monitoring via the Tuya Smart mobile app.

![Platform](https://img.shields.io/badge/Platform-ESP32-blue)
![Framework](https://img.shields.io/badge/Framework-TuyaOpen_SDK-orange)
![License](https://img.shields.io/badge/License-MIT-green)

## Overview

This project solves a common problem in buildings where the water tank is on the roof and the pump is on the ground floor. The controller:

- Measures water level using an ultrasonic sensor mounted above the tank
- Automatically turns the pump ON when water drops below a threshold
- Automatically turns the pump OFF when the tank is full
- Reports real-time data to a mobile app via Tuya Cloud (WiFi + BLE)
- Displays status on an ILI9341 TFT screen with physical button navigation
- Survives power failures with automatic pump state recovery

The sensor communicates over RS485, allowing reliable operation over 100+ feet of CAT5/6 cable between the rooftop sensor and ground-floor control box.

## Features

- **Auto/Manual pump control** with configurable ON/OFF thresholds
- **Real-time monitoring** via Tuya Smart app (iOS/Android)
- **ILI9341 2.8" TFT display** showing water level, distance, pump status
- **3-button menu system** for local settings adjustment
- **RS485 long-distance link** between sensor (roof) and controller (ground)
- **Power failure recovery** — remembers pump state across power cycles with configurable delay
- **Safety shutdown** — stops pump automatically if sensor fails for extended period
- **Persistent settings** — all configuration saved to flash (survives reboot)
- **OTA updates** via Tuya Cloud

## Hardware

### Components

| Component | Model | Purpose |
|---|---|---|
| Microcontroller | ESP32 DevKit V1 (30-pin, ESP-WROOM-32) | Main controller |
| Ultrasonic Sensor | A02YYUW | Waterproof distance measurement (UART, 3-450cm) |
| Display | ILI9341 2.8" TFT (240x320, SPI) | Local status display |
| Relay | 12V 30A optocoupler module | Pump switching (220V AC) |
| RS485 Modules | TTL-RS485 auto-flow (x2) | Long-distance sensor link |
| Power Supply | 220V AC to 12V DC | Main power |
| Buck Converter | LM2596 | 12V to 5V for ESP32 |
| Voltage Regulator | 7805 or LM2596 mini | 12V to 5V at rooftop |
| LEDs | Red 5mm + Green 5mm | Power and pump indicators |
| Buttons | Tactile switches (x4) | Pump toggle + menu navigation |
| Cable | CAT5/6 (~30m) | RS485 data + power to roof |

### Pin Assignments

```
ESP32 DevKit V1 (30-pin)
├── ILI9341 Display (SPI)
│   ├── GPIO 23  →  MOSI (SDA)
│   ├── GPIO 18  →  SCK
│   ├── GPIO  5  →  CS
│   ├── GPIO  2  →  DC (A0)
│   ├── GPIO  4  →  RST (RESET)
│   └── GPIO 15  →  LED (Backlight)
│
├── RS485 Sensor (UART1, remapped)
│   ├── GPIO 16  →  RS485 Module TXD (ESP32 RX)
│   └── GPIO 17  →  RS485 Module RXD (ESP32 TX)
│
├── Relay & LEDs
│   ├── GPIO 27  →  Relay IN (active LOW)
│   └── GPIO 13  →  Green LED via 220Ω
│
├── Buttons (all active LOW, internal pull-up)
│   ├── GPIO 14  →  PUMP button
│   ├── GPIO 25  →  UP button
│   ├── GPIO 26  →  DOWN button
│   └── GPIO 32  →  SELECT button
│
└── Power
    ├── VIN      ←  5V from LM2596 buck converter
    ├── 3.3V     →  Display VCC, RS485 #1 VCC, Red LED via 330Ω
    └── GND      →  Common ground
```

### System Architecture

```
 ROOFTOP (Sensor Box)              CAT5/6 Cable (~100ft)           GROUND FLOOR (Control Box)
┌─────────────────────┐          ┌─────────────────┐          ┌──────────────────────────────┐
│                     │          │                 │          │                              │
│  A02YYUW Sensor     │          │  Pair 1: A + B  │          │  ESP32 DevKit V1             │
│       │             │          │  (RS485 data)   │          │    ├── ILI9341 Display       │
│  RS485 Module #2    │◄────────►│                 │◄────────►│    ├── RS485 Module #1       │
│       │             │          │  Pair 2: +12V   │          │    ├── Relay → Pump (220V)   │
│  7805 (12V→5V)      │          │         + GND   │          │    ├── 4 Buttons             │
│       │             │          │  (power)        │          │    ├── 2 LEDs                │
│  12V + GND input    │          │                 │          │    └── LM2596 (12V→5V)       │
│                     │          │                 │          │                              │
└─────────────────────┘          └─────────────────┘          └──────────────────────────────┘
```

### Water Level Calculation

```
      Sensor ──┐
               │ sensor_offset (cm)
      100% ────┤─────────────────
               │
               │ ← distance (measured by sensor)
               │
               │ tank_height (cm)
               │
        0% ────┴─────────────────

water_height = (tank_height + sensor_offset) - distance
water_level% = (water_height / tank_height) × 100
```

## Tuya Cloud Integration

### Data Points (DPs)

| DP ID | Name | Type | Range | Mode | Description |
|---|---|---|---|---|---|
| 1 | Switch | Bool | — | RW | Master pump ON/OFF |
| 101 | Water Level | Value | 0-100 | RO | Water level percentage |
| 102 | Pump Running | Bool | — | RO | Pump is currently active |
| 103 | Pump Mode | Enum | auto/manual | RW | Operating mode |
| 104 | ON Threshold | Value | 5-59 | RW | Pump ON below this % |
| 105 | OFF Threshold | Value | 10-100 | RW | Pump OFF above this % |
| 106 | Tank Height | Value | 10-1000 | RW | Tank height in cm |
| 107 | Sensor Offset | Value | 0-100 | RW | Sensor-to-100% distance |
| 108 | Raw Distance | Value | 0-600 | RO | Raw sensor reading in cm |
| 109 | Fault | Fault | bitmap | RO | sensor_fault, pump_stuck, overflow_risk |

### Tuya Platform Setup

1. Create account at [platform.tuya.com](https://platform.tuya.com)
2. Create Product → Category: Water Pump → Protocol: WiFi+BLE
3. Add the DPs listed above under Function Definition
4. Get 2 free TuyaOpen licenses (Hardware Development → Get Free Licenses)
5. Download the license Excel file (contains UUID and AuthKey)
6. Update `include/tuya_config.h` with your PID, UUID, and AuthKey

## Project Structure

```
water_tank/
├── CMakeLists.txt          # TuyaOpen build configuration
├── app_default.config      # Board and feature config
├── include/
│   ├── tuya_config.h       # Product ID, pins, DPs, defaults
│   └── ili9341.h           # Display driver header
└── src/
    ├── tuya_main.c         # Main application
    └── ili9341.c           # ILI9341 display driver (ESP-IDF SPI)
```

## Building & Flashing

### Prerequisites

- [TuyaOpen SDK](https://github.com/tuya/TuyaOpen) cloned to a path without spaces
- Python 3.x with `esptool` installed (`pip install esptool`)
- Git, CMake, Ninja

### Setup

```bash
# Clone TuyaOpen SDK
git clone https://github.com/tuya/TuyaOpen.git
cd TuyaOpen
git submodule update --init --recursive

# Copy project
cp -r water_tank apps/tuya_cloud/water_tank

# Activate environment
./export.sh    # Linux/Mac
.\export.ps1   # Windows PowerShell

# Select board
cd apps/tuya_cloud/water_tank
tos.py config choice   # Select ESP32
```

### Configure

Edit `include/tuya_config.h` with your credentials:

```c
#define TUYA_PRODUCT_ID     "your_product_id_here"
#define TUYA_DEVICE_UUID    "your_uuid_here"
#define TUYA_DEVICE_AUTHKEY "your_authkey_here"
```

Disable AI components (if build fails on `tuya_ai_encoder.h`):

```bash
tos.py config menu
# Navigate: configure tuyaopen → configure tuya AI → disable
# Also disable: enable ai components
```

### Build

```bash
tos.py build
```

### Flash

TuyaOpen's flash tool may not work. Use `esptool` directly:

```bash
# Windows: deactivate TuyaOpen venv first
deactivate

cd .build/bin
python -m esptool --chip esp32 --port COM6 --baud 460800 write_flash \
    0x1000 bootloader.bin \
    0x8000 partition-table.bin \
    0xd000 ota_data_initial.bin \
    0x10000 water_tank.bin
```

Hold the **BOOT** button on the ESP32 if it doesn't connect.

### Monitor

```bash
tos.py monitor -p COM6
# or
python -m serial.tools.miniterm COM6 115200
```

## Pairing with Tuya Smart App

1. Install **Tuya Smart** app on your phone
2. Power on the ESP32 — it enters pairing mode on first boot
3. In the app: tap **+** → **Add Device**
4. Enable Bluetooth on your phone
5. The app should auto-discover the device via BLE
6. Follow the wizard to connect it to your WiFi
7. The device appears as "Water Tank Controller" in your app

If auto-discover fails, try AP mode: connect your phone to the `SmartLife-XXXX` WiFi network, then return to the app.

## Button Controls

| Button | Short Press | Long Press (3s) |
|---|---|---|
| PUMP | Toggle pump ON/OFF (manual mode only) | Toggle Auto/Manual mode |
| SELECT | Enter menu / Confirm edit / Save | — |
| UP | Navigate up / Increase value | — |
| DOWN | Navigate down / Decrease value | — |

### Display Screens

- **Home Screen**: Water level bar, percentage, distance, pump status, thresholds
- **Settings Menu**: Press SELECT to enter. Navigate with UP/DOWN, edit with SELECT
- **Edit Screen**: Change value with UP/DOWN, save with SELECT, cancel with PUMP

## Known Issues & Solutions

### UART Pin Remap Order

The UART1 pin remap functions **must** be called **before** `tal_uart_init`:

```c
// CORRECT — set pins first, then init
__tkl_uart1_set_txd_pin(17);
__tkl_uart1_set_rxd_pin(16);
tal_uart_init(TUYA_UART_NUM_1, &cfg);

// WRONG — init first, then set pins (won't work)
tal_uart_init(TUYA_UART_NUM_1, &cfg);
uart_set_pin(1, 17, 16, -1, -1);
```

### TuyaOpen SDK Only Supports UART 0 and 1

`MAX_UART_NUM = 2` in the platform driver. Use `TUYA_UART_NUM_1` (not `_2`). Default UART1 pins are GPIO9/10 — remap to GPIO16/17 as shown above.

### Sensor Buffer Overflow (Freeze Bug)

The A02YYUW sends data continuously every 100ms. With a 5-second read interval, the 256-byte UART buffer overflows and stalls. Fix: flush all stale data before each read:

```c
// Drain buffer
while (tal_uart_read(g_uart_num, flush, sizeof(flush)) > 0) { }
// Wait for fresh frame
tal_system_sleep(150);
// Then read
```

### AI Components Build Error

If build fails on `tuya_ai_encoder.h: No such file`, disable AI via `tos.py config menu` or set `CONFIG_ENABLE_AI=n` in `app_default.config`.

### DP 101 Minimum Value

Set DP 101 (Water Level) minimum to **0** on the Tuya platform (not 1), otherwise 0% readings get rejected by the schema validator.

### DP 109 Fault Bitmap

The Tuya "Fault" DP type uses named barriers (sensor_fault, pump_stuck, overflow_risk), not raw bitmap values. DP 109 reporting is currently disabled in firmware to avoid schema errors.

### Schema Sync After Platform Changes

After modifying DPs on platform.tuya.com, the device caches the old schema. To force resync: remove the device from the app, power cycle the ESP32, and re-pair.

## Offline Arduino Nano Version

An offline variant (no WiFi/cloud) is available for Arduino Nano in the `water_tank_nano/` directory. It provides the same core functionality with a local TFT display and EEPROM settings persistence, using PlatformIO + Arduino framework.

## License

MIT License — see [LICENSE](LICENSE) for details.

## Acknowledgments

- [TuyaOpen SDK](https://github.com/tuya/TuyaOpen) — ESP32 cloud connectivity framework
- [A02YYUW](https://wiki.dfrobot.com/A02YYUW_Waterproof_Ultrasonic_Sensor_SKU_SEN0311) — Waterproof ultrasonic sensor documentation
- [ESP-IDF](https://github.com/espressif/esp-idf) — Espressif IoT Development Framework
