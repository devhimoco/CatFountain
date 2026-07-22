# Hardware notes

The electrical lessons from this project, most of them learned the hard way.
If you build one of these, read this before you power anything on.

---

## ⚠️ Protect the board — this is not optional

A DC pump is an inductive, current-hungry load. Sharing a rail with an ESP32 without
protection produces exactly the symptoms that cost the most debugging time here: random
reboots, brown-outs, and sensors dropping off the I²C bus.

Three components, all cheap:

| Component | Placement | Job |
|---|---|---|
| **1N4007 / Schottky diode** | Cathode → Pump(+), anode → Pump(−) | Catches the back-EMF spike when the motor switches off |
| **1000 µF electrolytic** | Across the supply, close to the pump | Local energy reservoir for the inrush the supply can't deliver fast enough |
| **100 nF ceramic** | 3.3 V → GND at the ESP32 | Decoupling |

This checklist appears verbatim in the `v6` sketch headers under **"BOARD PROTECTION (read
before wiring!)"** because it was written after the failures, not before.

**Better still:** give the motor its own supply rail and share only ground with the ESP32.

---

## Pump driving: relay → MOSFET

**Generation V started with a relay, and it was a mistake.**

The blue Tongling relay module coil needs **5 V** to trigger, but ESP32 GPIO only swings
**3.3 V**. Worse, the module has an internal pull-up to 5 V on its `IN` pin — so at idle
the pin sits at 3.3 V and *the coil is already energised*. The pump ran constantly, and
the "active-low trick" used to work around it was precisely what kept it on.

The fix in `V10-FullWithTransistor` was to drive the relay **active-HIGH through an NPN
transistor**. From `v5` onward the relay was dropped entirely for a **MOSFET module**
(FR120N with a PC817 optocoupler), which brought two real advantages:

- **PWM speed control** — ten pump power levels instead of on/off
- **Silence and speed** — no mechanical contacts, no clack, no wear

Current firmware drives the MOSFET with ESP32 **LEDC PWM** at 1 kHz, 10-bit resolution.

### Soft-start

The pump ramps up over ~300 ms rather than jumping to full duty. Instant full duty pulls a
large inrush spike, which is the classic brown-out trigger the moment the pump engages.
Ramping spreads that inrush and lowers `di/dt`, which also reduces the EMI coupled onto
the shared I²C bus.

---

## The I²C bus is the fragile part

Two VL53L0X sensors **and** the OLED share one bus on GPIO21/22. That is convenient — no
extra pins — but it concentrates risk.

### Two sensors, one bus

Every VL53L0X boots at address `0x29`. Two on one bus therefore collide. `XSHUT` solves it:

1. Hold **both** sensors in reset (`XSHUT` LOW) — before anything else in `setup()`
2. Release sensor #1, reassign it to `0x30`
3. Release sensor #2, reassign it to `0x31`

The firmware claims both `XSHUT` pins and forces them LOW as the **very first thing** in
`setup()` — before `Serial.begin()`, before the pump. Every millisecond of delay is a
millisecond in which a floating pin lets a sensor wake early and grab `0x29`, which
produced random boot failures that varied between resets.

### Bus jamming and recovery

If a device is interrupted mid-transaction — noise, a power glitch, a sensor unplugged
while live — it can leave **SDA stuck LOW**, jamming the bus for every device on it.

Standard recovery, implemented in `recoverI2CBus()`:

1. Release the Wire driver
2. Manually pulse SCL up to 9 times to clock the stuck device out of its transaction
3. Issue a manual STOP condition
4. Re-init the bus and the devices

This runs **preventively at every boot**, automatically on sensor dropout, and on demand
from a button in the web app. `3-laser/ModuleTest/V09-OledTestRecovery` is where the
technique was first proven in isolation.

### Motor EMI

Sensors dropping off the bus **only while the pump runs** is EMI, not a wiring fault. Keep
motor leads physically away from SDA/SCL, keep the I²C runs short, and make sure the bus
has proper pull-ups. Soft-start helps here too.

---

## Sensor generations, and why each was replaced

| Sensor | Fatal limitation |
|---|---|
| **PIR (HC-SR501)** | Detects body heat *in motion*. A cat that settles down to drink stops moving, and the trigger drops. Fundamentally wrong sensor for the job. |
| **IR (LM393 comparator)** | Simple presence, no distance, and easily fooled by ambient light and surface colour. |
| **Ultrasonic (HC-SR04)** | Genuine distance, but a **~15° beam**. Miss the cone by a few centimetres and the cat is invisible. A median filter fixes noise, not geometry. |
| **Laser ToF (VL53L0X)** | ✅ Millimetre accuracy, fast, narrow and predictable. Two of them cover the bowl properly. No dependency on body heat or motion. |

---

## Pin choices worth knowing

| Pin | Used for | Why |
|---|---|---|
| `GPIO34` | Water level (analogue) | **ADC1.** ADC2 pins cannot be read while WiFi is active — a classic ESP32 trap. `GPIO34` is also input-only, which suits a sensor perfectly. |
| `GPIO21` / `GPIO22` | I²C SDA / SCL | ESP32 defaults; every I²C device shares them |
| `GPIO32` / `GPIO33` | Sensor `XSHUT` | Ordinary GPIOs, safe at boot |
| `GPIO26` | Pump PWM | LEDC-capable, no boot-strapping role |
| `GPIO13` | Touch `SIG` | Module drives it push-pull, so no pull-up and no floating-pin noise |

---

## The 433 MHz remote: a documented failure

A YK04 transmitter with a PT2272-M4 receiver was tested (`3-laser/ModuleTest/V15-RfPt2272-Test`)
and **deliberately dropped**.

It suffered real, reproducible electrical noise that survived:
- adding a proper ~17 cm antenna
- a hold/debounce filter in software
- dropping the noisiest button entirely

The motor EMI simply beat it. WiFi control — error-checked, and already present — proved
far more robust than continuing to fight RF near a running pump. Recorded here because a
negative result is still a result.
