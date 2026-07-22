# Generation VI — Ultrasonic

**3 sketches · header version v7 · the shortest generation**

The HC-SR04 finally delivered what infrared never could: an actual **distance in
centimetres**, so the fountain could tell "something is 12 cm away" from "something is
across the room". A distance threshold became a real, tunable setting.

It lasted three versions.

> **Why this generation ended:** the HC-SR04 emits a **~15° cone**. Inside it, readings are
> good. A few centimetres outside it, the cat may as well not exist. The `v7.1` header is
> refreshingly blunt about this — the median filter it added fights *noise*, and explicitly
> **"does NOT fix the sensor's narrow ~15° beam angle."** No amount of filtering fixes
> geometry, so the next generation changed the sensor.

---

## Versions

| # | Folder | Header | What changed |
|---|---|---|---|
| 01 | `V01-UltraSound` | v7 | **IR replaced with HC-SR04.** Distance threshold 2–100 cm from the web UI · master ON/OFF switch that kills the whole mechanism · independent ON/OFF for the ultrasonic and touch sensors · all switches + distance persisted to NVS. |
| 02 | `V02-SolvedBug` | v7.1 | Multi-sample **median filter** on readings (kills noise-driven false triggers) · **hardware watchdog** that resets on a genuine hang rather than on a timer · pump forced OFF on every boot. |
| 03 | `V03-RestartButton-LATEST` | v7.3 | Restart button added to the web app. Final build of the generation. |

---

## What carried forward

Short as it was, this generation established patterns the current firmware still uses:

- **Per-sensor enable switches** plus a master ON/OFF — the same shape as the toggles in
  today's web app
- **Distance as a stored, user-tunable setting** rather than a compile-time constant
- **A real hardware watchdog**, deliberately distinguished from a timed reboot
- **Pump OFF on every boot**, a safety default that is still there
- **Filtering sensor readings before trusting them**, which became the debounce and
  fail-count logic in Generation X

The move to [Generation X](../3-laser/) kept every one of these ideas and changed only the
thing that was actually broken: the sensor.
