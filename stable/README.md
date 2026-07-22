# Stable — the build to flash

If you just want a working fountain, this is the folder.

Both sketches here are **exact byte-for-byte copies** of their counterparts in
[`../3-laser/Alfa-X/`](../3-laser/Alfa-X/) (verified with `cmp`). They are promoted here so
that "the version that actually works" is never ambiguous.

---

## ⭐ `V35AlfaX-V2.4-Debug-LatestAllDoneFirstAllOk/`

**PooKooli Fountain X V2.4 — the recommended build.**

Everything working together: dual laser sensors, water-level cutoff, MOSFET pump with ten
power levels, OLED, touch override, the full sidebar web app with Schedule and Activity
log, and the boot/crash diagnostics.

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
