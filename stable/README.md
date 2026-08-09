# Stable — the build to flash

If you just want a working fountain, this is the folder.

All three sketches here are **exact byte-for-byte copies** of their counterparts in
[`../3-laser/Alfa-X/`](../3-laser/Alfa-X/) (verified with `cmp`/`diff`). They are promoted here
so that "the version that actually works" is never ambiguous.

---

## ⭐ `V43AlfaX-V2.7-Debug-WaterLevelFix/`

**PooKooli Fountain X V2.7 — the recommended build.**

Everything V2.4 had, plus a **BLE iTag remote** (V2.5 — a physical keyfob button as a
universal pump toggle, no phone needed), a **Sensor Test Mode** bench-debug toggle (V2.6),
and a median-filtered, recalibrated water-level reading (V2.6/V2.7) that stopped false
"tank empty" trips during pump-motor inrush.

The water ADC thresholds (`WATER_ADC_DRY`/`WATER_ADC_FULL`) are still marked **INTERIM** in
the sketch — back-calculated estimates, not a from-scratch calibration. Re-run the
calibration steps in [`../docs/SETUP.md`](../docs/SETUP.md) for your own tank/sensor before
trusting the percentage reading.

## `V35AlfaX-V2.4-Debug-LatestAllDoneFirstAllOk/`

**PooKooli Fountain X V2.4.** Everything working together: dual laser sensors, water-level
cutoff, MOSFET pump with ten power levels, OLED, touch override, the full sidebar web app
with Schedule and Activity log, and the boot/crash diagnostics — kept here as the last build
before the BLE remote and water-level recalibration work in V2.5–V2.7.

## `V34AlfaX-V2.4-Debug-LatestAllDoneEXSchedule/`

The same V2.4 firmware **without the Schedule subsystem**. Useful if you want time-of-day
triggering out of the picture — either because you have no reliable clock source, or to
narrow down a problem.

---

## Flashing it

1. Open the `.ino` in the Arduino IDE (the folder and file names already match, as Arduino
   requires).
2. Install **Adafruit VL53L0X**, **Adafruit SSD1306**, **Adafruit GFX**.
3. Select board **ESP32 Dev Module**.
4. Set your WiFi credentials and `TZ_OFFSET_SEC`.
5. Upload, then open Serial Monitor at **115200** to find the device IP.

Full walkthrough, including the `secrets.h` pattern and water-sensor calibration:
**[`../docs/SETUP.md`](../docs/SETUP.md)**

⚠️ Fit the pump protection components first — [`../docs/HARDWARE.md`](../docs/HARDWARE.md).

---

## First place to look when something goes wrong

Open the web app → **Device Info → Boot History**.

It records the **reset reason**, **free heap**, and a **crash-location breadcrumb** for the
last 10 boots, written to flash so it survives the very crash you are investigating. A
line like `PANIC @ i2cinit:sensor2` tells you precisely where the firmware died — which
beats guessing every time.
