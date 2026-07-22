# Module tests

**5 sketches · each module proven alone before it was trusted in a build**

No pump, no other sensors, no web app — just one module and a Serial log. When something
fails in a full build, these are the sketches that tell you whether the module is broken or
the integration is.

This is the habit that made Generation X far more reliable than the two before it.

---

| Folder | Tests | What it establishes |
|---|---|---|
| `V03-VL53L0X-SensorTest` | One VL53L0X | Basic ranging on ESP32. `XSHUT`→GPIO32, 3.3 V logic. Includes the multi-sensor addressing example, commented out — the seed of `V07`. |
| `V07-DualSensor-Test` | Two VL53L0X, one bus | 🔑 The `XSHUT` address-reassignment dance. Both sensors boot at `0x29`, so they are brought up **one at a time** and each given a unique address before use. This procedure went straight into every later build. |
| `V08-OledTest` | SSD1306 alone | Runs a **full I²C scan first** (printing every address that responds), then tries both `0x3C` and `0x3D`, so a wrong-address module still shows up instead of silently failing. |
| `V09-OledTestRecovery` | SSD1306 + bus recovery | 🔑 Before touching `Wire.h`, manually bit-bangs SCL to force-release a stuck SDA line — the classic "bus jammed by a bad device" fix. Also retries on a **second bus** (GPIO18/19) to prove whether the fault is bus-specific or follows the OLED. |
| `V15-RfPt2272-Test` | 433 MHz YK04 + PT2272-M4 | ✗ **The test that killed the feature.** Serial-only, printing every state change *plus* a periodic raw snapshot — so "nothing is happening" (wiring/power) is distinguishable from "pins are stuck" (decoder/pairing). |

---

## What came out of these

**`V07` → the addressing scheme.** Every Generation X build reassigns the sensors to
`0x30`/`0x31` at boot exactly the way this test proved.

**`V09` → `recoverI2CBus()`.** The bit-bang SCL recovery proven here became a permanent
feature: it runs **preventively at every boot**, **automatically on sensor dropout**, and
**on demand** from a button in the web app. Two sensors and an OLED share one bus, so a
single jammed device would otherwise take down everything.

**`V15` → a documented "no".** The RF remote suffered real, reproducible electrical noise
that survived a proper ~17 cm antenna, a hold filter in software, and dropping the noisiest
button entirely. Motor EMI beat it. WiFi control — already present and error-checked —
proved far more robust than continuing to fight RF next to a running pump.

The remote is listed as **NOT included** in every later header. A negative result, recorded
deliberately.
