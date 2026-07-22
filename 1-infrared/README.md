# Generation V — Infrared / PIR

**18 sketches · header versions v1 → v6 · where the project began**

The first generation. It starts as a Bluetooth prototype with a relay and ends as a
WiFi-connected, MOSFET-driven fountain with an OLED — and then walks straight into the
wall that ended the generation.

> **Why this generation ended:** a PIR sensor detects **body heat in motion**. A cat that
> settles down to drink stops moving, so the trigger drops out mid-drink. Swapping PIR for
> a plain IR comparator traded one problem for another: presence, but no distance, and
> easy to fool with ambient light. Neither sensor could answer the actual question —
> *"is the cat at the bowl right now?"*

---

## Versions

| # | Folder | Header | What changed |
|---|---|---|---|
| 01 | `V01-Origin` | — | **BLE**, not WiFi. PIR→GPIO5, touch→GPIO4, water→GPIO34, pump MOSFET→GPIO16, LED→GPIO2. No web app, no display, no persistence. |
| 02 | `V02-Base` | v1 | First real build. ESP32 NodeMCU-32, touch + PIR + water + **relay**, three trigger sources (touch 1 min · PIR 1 min · WiFi 2 min). |
| 03 | `V03-Iter1` | v2 | All three timers configurable live from the web app, saved to flash so they survive power cuts. |
| 04 | `V04-Iter2` | v3 | 0.91" SSD1306 OLED for status/errors · empty-tank error on OLED *and* web · switched to digital `DO` pin with inverted logic · relay set `OUTPUT` + `pumpOFF()` at boot so the pump is off until triggered. |
| 05 | `V05-Iter3` | v4 | Renamed **"PooKooli Fountain"** everywhere · **continuous mode** (runs until stopped from app or button). |
| 06 | `V06-Iter4` | v4 | Iteration on v4. |
| 07 | `V07-WithPIC` | v4 | Embedded artwork added — at 384 KB, the largest sketch in the whole archive. |
| 08 | `V08-Iter5` | v4 | Iteration on v4. |
| 09 | `V09-WithPicLowHDD` | v4 | Same artwork, reduced size to fit flash comfortably. |
| 10 | `V10-FullWithTransistor` | v4 | 🔑 **The relay bug, solved** — see below. |
| 11 | `V11-FullNoHangEmoji` | v4 | Fixed a hang traced to emoji characters in the UI strings. |
| 12 | `V12-NewPicNotFit` | v4 | New artwork attempt that didn't fit — kept as the record of a dead end. |
| 13 | `V13-PicAppOK-IconNO` | v4 | Artwork renders in the app, but the favicon/icon still didn't. |
| 14 | `V14-ESP32-PirRelay` | v4 | Consolidated ESP32 + PIR + relay build. |
| 15 | `V15-ESP12EEsp8266-AfterBurn` | v5 | 🔀 **Hardware detour** — NodeMCU **ESP8266**, relay → **FR120N MOSFET + PC817 optocoupler** with PWM speed control, `Preferences` → `EEPROM`. |
| 16 | `V16-AllGood` | v6 | ↩️ **Back to ESP32-DevKitC.** LEDC PWM (10 power levels) · settings-save fix (no more reset every 2 s) · pump power in the web UI · water polarity flip as a `#define` · **IR (LM393, active-LOW) replaces PIR** · board-protection checklist added to the header. |
| 17 | `V17-TriggerProblemFix` | v6 | Remaining trigger problem fixed. |
| 18 | `V18-CorrectUseFirstTime-LATEST` | — | ➡️ **The pivot.** An HC-SR04 ultrasonic test sketch — the first step into [Generation VI](../2-ultrasonic/). |

---

## 🔑 The relay bug

The single most instructive failure in this generation, and worth reading the header of
`V10-FullWithTransistor` for in full.

**Symptom:** the pump ran constantly, no matter what the code did.

**Cause:** the blue Tongling relay module's coil needs **5 V**, but ESP32 GPIO only swings
**3.3 V**. The module also has an internal pull-up to 5 V on its `IN` pin — so at idle the
pin sat at 3.3 V and the coil was *already energised*. The "active-low trick" adopted to
work around it was exactly what kept the pump on.

**Fix:** drive the relay **active-HIGH through an NPN transistor**.

**Real lesson:** the relay was the wrong part. From `v5` onward it was replaced by a MOSFET
module — quieter, faster, no mechanical wear, and it enables PWM speed control. See
[`docs/HARDWARE.md`](../docs/HARDWARE.md).

---

## Note on `V18`

`V18-CorrectUseFirstTime-LATEST` is **third-party code** — an HC-SR04 example by
[Rui Santos / RandomNerdTutorials](https://RandomNerdTutorials.com/esp32-hc-sr04-ultrasonic-arduino/),
included under its own permissive notice (preserved in the file header, and recorded in
[LICENSE](../LICENSE)).

It lives here rather than in `2-ultrasonic/` because that is chronologically where it sits:
the last thing tried in the infrared generation was the sensor that replaced it.
