# Setup guide

How to get the PooKooli Fountain running from a fresh clone.

---

## 1. What you need

**Board**
- ESP32-DevKitC (WROOM-32D) — selected in the IDE as **"ESP32 Dev Module"**

**Modules**

| Module | Part | Notes |
|---|---|---|
| Laser sensor ×2 | VL53L0X (GY-VL53L0X / CJVL53L0XV2) | 3.3 V logic |
| Water level | Analogue water-level sensor | Must be on an **ADC1** pin |
| Pump driver | MOSFET module (e.g. FR120N + PC817) | Not a relay — see [HARDWARE.md](HARDWARE.md) |
| Display | 0.96" SSD1306 OLED, I²C | Address `0x3C` |
| Touch | Capacitive touch module | Active-HIGH, driven output |
| Pump | 5 V/12 V DC submersible pump | Match your supply |
| BLE remote *(optional)* | Any generic "iTag" anti-lost keyfob | No wiring — pairs over Bluetooth |

**Software**
- [Arduino IDE](https://www.arduino.cc/en/software) 2.x
- ESP32 board package by **Espressif** (Boards Manager → "esp32")
- Libraries via **Library Manager**:
  - `Adafruit VL53L0X` (pulls in `Adafruit BusIO`)
  - `Adafruit SSD1306`
  - `Adafruit GFX`
  - `NimBLE-Arduino` (by h2zero) — only needed for the BLE remote, V2.5+

---

## 2. Wire it up

Follow [`assets/schematic.svg`](../assets/schematic.svg) — it is the authoritative
diagram. Summary:

```
VL53L0X #1   VCC→3.3V  GND→GND  SDA→GPIO21  SCL→GPIO22  XSHUT→GPIO32   → addr 0x30
VL53L0X #2   VCC→3.3V  GND→GND  SDA→GPIO21  SCL→GPIO22  XSHUT→GPIO33   → addr 0x31
Water level  GND→GND   +→3.3V   S→GPIO34   (ADC1 ch6, input-only pin)
OLED         VCC→3.3V  GND→GND  SDA→GPIO21  SCL→GPIO22  (shares the sensor bus)
Touch        VCC→3.3V  GND→GND  SIG→GPIO13 (active-HIGH — no pull-up needed)
Pump MOSFET  Control GND→GND    PWM→GPIO26
```

> **Both sensors boot at the same address (`0x29`).** `XSHUT` is what makes two on one bus
> possible: the firmware holds both in reset, releases one, reassigns it, then releases the
> other. Get these two pins right or you'll see a single sensor, or neither.

⚠️ **Do not skip the protection components.** A DC pump sharing the ESP32's rail will
brown out or crash the board. See [HARDWARE.md](HARDWARE.md) before powering on.

---

## 3. Choose a sketch

| I want to… | Open |
|---|---|
| Just run the fountain | `stable/V43AlfaX-V2.7-Debug-WaterLevelFix/` |
| Run it without the BLE remote / on the older, more-tested base | `stable/V35AlfaX-V2.4-Debug-LatestAllDoneFirstAllOk/` |
| Check one module in isolation | `3-laser/ModuleTest/` |
| Follow the build-up module by module | `3-laser/Beta/` |
| See the earlier hardware generations | `1-infrared/`, `2-ultrasonic/` |

Arduino requires the sketch folder name to match the `.ino` filename — every folder here
already satisfies that, so just open the `.ino` and the IDE handles the rest.

---

## 4. WiFi credentials

The sketches declare credentials near the top:

```cpp
const char* WIFI_SSID     = "YourNetwork";
const char* WIFI_PASSWORD = "YourPassword";
```

**If you plan to push your build anywhere public**, move them out of the sketch instead.
Create `secrets.h` next to the `.ino`:

```cpp
#pragma once
#define SECRET_WIFI_SSID      "YourNetwork"
#define SECRET_WIFI_PASSWORD  "YourPassword"
```

then in the sketch:

```cpp
#include "secrets.h"
const char* WIFI_SSID     = SECRET_WIFI_SSID;
const char* WIFI_PASSWORD = SECRET_WIFI_PASSWORD;
```

`secrets.h` is already listed in [`.gitignore`](../.gitignore), so it will never be
committed.

---

## 5. Timezone (Schedule feature only)

```cpp
const long TZ_OFFSET_SEC = 12600;   // UTC+3:30
```

Set this to your own offset in seconds — `UTC-5` is `-18000`, `UTC+1` is `3600`.
Everything else (triggers, status, logs) works regardless; only the Schedule page's clock
and trigger times depend on it.

---

## 6. Upload and first boot

1. Select **ESP32 Dev Module**, pick the COM port, upload.
2. Open **Serial Monitor at 115200 baud**.
3. You should see a boot report like:

```
=== PooKooli Fountain X V2.7 ===
[Boot] Reason: Power-on | Free heap: 271000 bytes | Was at: power-on
[WDT] Armed 10s
[VL53L0X] Sensor #1 OK at 0x30 (20ms timing budget)
[VL53L0X] Sensor #2 OK at 0x31 (20ms timing budget)
[OLED] OK at 0x3C
----------------------------------------
[BOOT CHECK] Sensor #1 (0x30): OK
[BOOT CHECK] Sensor #2 (0x31): OK
[BOOT CHECK] Water sensor: attached
[BOOT CHECK] OLED: attached
----------------------------------------
Connecting to YourNetwork....
✓ http://192.168.1.42
[Web] Server ready
[BLE] Scanning for iTag remote...
```

No iTag paired? `[BLE]` lines simply keep retrying every 5 s in the background — everything
else works normally without one.

4. Open that IP in a browser. On a phone, use **Add to Home Screen** for the PWA icon.

---

## 7. Calibrate the water sensor

Defaults are placeholders — your sensor will read differently. As of `stable/` V2.7 they are
even marked **INTERIM** in the sketch itself (back-calculated estimates, not measured on a
real sensor):

```cpp
#define WATER_ADC_DRY    244    // INTERIM — reading when completely dry
#define WATER_ADC_FULL  1596    // INTERIM — reading when fully submerged
#define WATER_LOW_PCT      5    // below this %, the pump is blocked
```

Watch the `[Water] raw=… (median of 7) pct=…%` line on Serial with the sensor dry, then
submerged, and put those two raw numbers in. Until you do, the percentage on the dashboard
is not meaningful.

Testing with an empty tank? Toggle **test mode** in the web app to force the level to 50 %
so the pump will run.

---

## 8. Troubleshooting

| Symptom | Likely cause |
|---|---|
| `Sensor #1/#2 FAILED` at boot | `XSHUT` wiring (GPIO32/33), or 5 V on a 3.3 V sensor |
| Only one sensor detected | Both stuck at `0x29` — the `XSHUT` reassignment didn't run |
| `[OLED] Not found` | Address is `0x3D` not `0x3C`, or SDA/SCL swapped |
| Board reboots when the pump starts | **Power.** Brown-out from motor inrush — see [HARDWARE.md](HARDWARE.md) |
| Sensors drop out only while pumping | Motor EMI on the I²C bus — route sensor wiring away from motor leads |
| Random reboots | Read **Device Info → Boot History** for the reset reason and crash checkpoint |
| Schedule never fires | Clock never synced — check the Schedule page clock and `TZ_OFFSET_SEC` |
| Tank blocked as "empty" when it isn't | Water ADC thresholds are still the **INTERIM** defaults — calibrate them (step 7) |
| BLE remote never connects | Its advertised name/UUIDs may not match `BLE_TAG_NAME`/`BLE_SVC_UUID`/`BLE_CHR_UUID` — reflash `3-laser/ModuleTest/iTag-BLE-Test-2` to discover your exact unit's values from Serial |

**The Boot History page is the single best diagnostic tool here.** It records the reset
reason, free heap, and a crash-location breadcrumb for the last 10 boots, and it survives
the crash itself. Start there before guessing.
