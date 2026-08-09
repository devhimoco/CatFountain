# Changelog

Every version of the PooKooli Fountain, newest first, with what changed and *why*.

Version numbers follow the scheme declared in the sketch headers:

- **BETA Vx.y** — test track: `+0.1` for each module proven and added
- **X Vx.y** — complete edition: `+0.1` small change, `+1.0` big change

Folder prefixes (`V01`, `V02`, …) are the archive's chronological ordering, applied
during the 2026-07-17 reorganisation — see [VERSION-MANIFEST.md](VERSION-MANIFEST.md).
Within Generation X, `ModuleTest/` and `Beta/` share one numbering sequence because they
were interleaved in real time: a module was tested in isolation first, then folded into
the next Beta build.

---

# Generation X — Laser time-of-flight  ·  `3-laser/`

Two VL53L0X sensors on a shared I²C bus, addresses reassigned via `XSHUT` at boot.
This is the current generation.

## Complete editions — `3-laser/Alfa-X/`

### X V2.7 — `V43-…-WaterLevelFix` ⭐ current
Second recalibration of the water ADC thresholds — the V2.6 numbers were themselves
provisional. `WATER_ADC_DRY`/`WATER_ADC_FULL` are back-calculated estimates and still marked
**INTERIM** in the sketch header; `WATER_LOW_PCT` raised `1% → 5%` for a saner cutoff margin.
`V43` is the build promoted to [`stable/`](stable/).

### X V2.6 — `V42-…-ERRDntShow`
The water-level reading was tripping a false "tank empty" block during pump-motor inrush —
same class of problem as the ToF sensors' own noise-rejection fix.

- ADC now reads a **median of 7 samples** instead of one, rejecting a single noisy reading
- `WATER_ADC_DRY`/`WATER_ADC_FULL`/`WATER_LOW_PCT` recalibrated

### X V2.5 — `V41-…-AddBLRemoteLatest`
**BLE iTag remote.** The ESP32 connects as a BLE *client* directly to a cheap iTag keyfob
(the kind sold for anti-lost apps) — no phone or app required once paired in firmware.
Protocol confirmed first in [`3-laser/ModuleTest/iTag-BLE-Test-2`](3-laser/ModuleTest/).

- Targets the common clone GATT layout (service `FFE0`, notifying characteristic `FFE1`);
  any notification counts as a press
- Button toggles the pump **continuous** (like the web app's ON button), with the same
  override precedence as Touch — stops any run, or starts one
- New **Sensor Test Mode** toggle on the Debug page: quiets the repeating I²C-recovery
  retry chain and `[Dist]` print for bench-testing with no sensors connected
- Adds a third radio subsystem (WiFi + WebServer + now BLE central) on a board with a
  documented watchdog-crash history — flagged in the header as worth watching

### X V2.4 (re-verified) — `V36`–`V39`
After `V35` bundled several overnight-crash fixes together, each was re-verified
independently rather than trusted as a package: `V36` restarted from the pre-fix `V34`
baseline, then `V37`–`V39` re-added the laser-confirm/soft-start, I²C-recovery-defer/
soft-stop, and per-day Schedule/browser-clock fixes one at a time. `V39` lands
**byte-identical to `V35`**, closing the re-verification.

This exact build is also packaged as its own standalone repository,
**[PooKooli-Fountain-X-V2.4](https://github.com/devhimoco/PooKooli-Fountain-X-V2.4)** — a
quick-start README and `secrets.h.example` instead of the full archive here — which is the
firmware actually flashed on the physical fountain as of this writing, predating the
V2.5–V2.7 work above.

### X V2.4 — `V34-…-Debug`, `V35-…-LatestAllDoneFirstAllOk`
A full night of data came back **10/10 PANIC, every entry still reading checkpoint
`"boot"`** — implying the crash happened in `setup()` before `loop()` ever ran. V2.3's
breadcrumbs only covered `loop()`, so that region was genuinely invisible.

- Checkpoints through **every phase of `setup()`**: I²C recovery, sensor/OLED init, NVS
  load, WiFi connect, NTP, web routes, watchdog config
- Watchdog now fed during the WiFi connect wait, which can block up to 15 s and
  previously had zero resets across the whole window
- `V35` was the build promoted to [`stable/`](stable/), superseded by `V43` (X V2.7) above

### X V2.3 — `V33-…-RestartOverNight2`
Reset *reason* told us when and why, never **where**. So: a crash-location breadcrumb in
RTC memory, updated at every major loop section and densely through I²C recovery, read
back on the next boot and stored alongside each Boot History entry.

### X V2.2 — `V32-…-RestartOverNight`
Fixed a **confirmed** crash cause found via V2.1's own boot log: `esp_task_wdt_init()`
was failing silently ("already initialized" — newer ESP32 cores auto-start a default
watchdog), so the intended 10 s timeout never actually took effect and a shorter hidden
default stayed active, firing mid-I²C-recovery.

### X V2.1 — `V31-…-DebugRestart`
Added persistent boot/crash diagnostics, because the RAM activity log is wiped by the very
reset being investigated and Serial can't be watched overnight.

- Reset reason + free heap written to **flash on every boot**
- Minimum-free-heap tracker
- Both surfaced on the Device Info page

### X V2.0 — `V30-…-FullResponsiveDesign`
A ground-up rewrite. Earned `+1.0` rather than `+0.1` under the project's own rule, since
it was a full UI overhaul *plus* two new subsystems.

- Sidebar dashboard web app, collapsing to a drawer on phones
- **Schedule** — NTP-synced daily trigger times, persisted to flash, routed through the
  same `startPump()` / `triggerBlocked()` path as every other trigger source
- **Recent Activity** log — 15-entry RAM ring buffer

### X V1.1 — `V25` → `V29`
The design exploration that became V2.0.
`V25` added the sensor nav layout · `V26`–`V28` three rounds of design iteration ·
`V29` full responsive pass.

### X V1.0 — `V22-1`, `V23`, `V24`
**First complete edition**, graduated from the BETA test track after every module was
verified working individually *and* together. `V24`/`V22-1` are the pre-delay-fix builds
kept for comparison.

## Test track — `3-laser/Beta/`

| Version | Folder | What it added |
|---|---|---|
| **BETA V1.6** | `V22-V1.6` | **Final beta.** I²C bus recovery — automatic on sensor dropout plus a manual button. 433 MHz remote removed for good. |
| **BETA V1.5** | `V14`, `V16`–`V21` | Stabilisation series 1.5.0 → 1.5.5, then `V21-V1.5-Final`. |
| **BETA V1.4** | `V13-V1.4` | Water sensor **attached** — real ADC readings; test mode became optional. |
| **BETA V1.3** | `V11`, `V12` | Touch sensor (capacitive, active-HIGH) as a universal manual override. |
| **BETA V1.2** | `V10-V1.2-Sensor2LCD` | 0.96" SSD1306 OLED — status + live cm readout. |
| **BETA V1.1** | `V04`, `V05`, `V06` | Both laser sensors live at `0x30`/`0x31`, 5–70 cm. `V05` raised sample speed; `V06` added **timer renewal** so the pump keeps running while the cat stays in range instead of stopping mid-drink. |
| **BETA V1.0** | `V02-V1.0` | Deliberately stripped to **one** sensor with water detached — the "add one module at a time" discipline starts here. |
| **BETA V0** | `V01-Base` | The original all-in base sketch, before the decision to strip back. |

## Module tests — `3-laser/ModuleTest/`

Each module proven in isolation before being trusted in a build.

| Folder | Target |
|---|---|
| `V03-VL53L0X-SensorTest` | Single VL53L0X on ESP32 |
| `V07-DualSensor-Test` | Two sensors on one bus — the `XSHUT` address-reassignment dance |
| `V08-OledTest` | OLED alone, with a full I²C scan and both common addresses |
| `V09-OledTestRecovery` | Same, plus bit-banged SCL to force-release a stuck SDA line, and a second bus on GPIO18/19 to isolate the fault |
| `V15-RfPt2272-Test` | 433 MHz YK04 + PT2272-M4 receiver — **the test that killed the feature** |
| `iTag-BLE-Test-2` | Standalone BLE client — discovers a real iTag's advertised name and GATT layout, the protocol `V41` (X V2.5) then wired into the fountain |

---

# Generation VI — Ultrasonic  ·  `2-ultrasonic/`

Headers carry version **v7**. Real distance measurement at last, but the HC-SR04's ~15°
beam is too narrow to cover a drinking bowl reliably — which is what drove the move to
laser.

### v7.3 — `V03-RestartButton-LATEST`
Restart button added to the web app.

### v7.1 — `V02-SolvedBug`
- Multi-sample **median filter** on ultrasonic readings, fighting noise-driven false
  triggers. Explicitly *not* a fix for the narrow beam angle — the header says so plainly.
- **Hardware watchdog** that resets on a genuine hang (not a timed reboot)
- Pump forced OFF on every boot

### v7 — `V01-UltraSound`
- IR sensor **replaced** with HC-SR04
- Distance threshold setting, 2–100 cm, from the web UI
- Master ON/OFF switch, plus independent per-sensor switches for ultrasonic and touch
- All switches + distance persisted to NVS

---

# Generation V — Infrared / PIR  ·  `1-infrared/`

Where it began. Header versions run v1 → v6 across 18 folders.

### `V18-CorrectUseFirstTime-LATEST` — the pivot
An HC-SR04 test sketch from **Rui Santos / RandomNerdTutorials** (third-party, see
[LICENSE](LICENSE)). Chronologically the last thing in Gen V and the first step toward
Gen VI.

### v6 — `V16-AllGood`, `V17-TriggerProblemFix`
- Back to **ESP32-DevKitC** (and `Preferences`) after the ESP8266 detour
- LEDC PWM for the MOSFET pump, 1–10 power levels
- Web settings save fixed — no more reset every 2 s
- Pump power exposed in the web UI; water-sensor polarity flip as a `#define`
- **IR sensor (LM393, active-LOW) replaces PIR**
- Header gains a hard board-protection checklist: 1N4007 flyback diode, 1000 µF bulk
  capacitor, 100 nF decoupling. `V17` fixed a remaining trigger problem.

### v5 — `V15-ESP12EEsp8266-AfterBurn`
A hardware detour: NodeMCU **ESP8266**, relay swapped for a **FR120N MOSFET module with
PC817 optocoupler** and PWM speed control, `Preferences` swapped for `EEPROM`.

### v4 + transistor — `V10-FullWithTransistor` → `V14-ESP32-PirRelay`
**The relay bug, solved.** The blue Tongling relay coil needs 5 V, but ESP32 GPIO only
swings 3.3 V — and the module's internal pull-up to 5 V meant idle = coil ON. The pump ran
constantly, and the "active-low trick" was what kept it that way. Fixed by driving the
relay **active-HIGH through an NPN transistor**. `V11`–`V14` iterate on embedded artwork
and icons.

### v4 — `V05-Iter3` → `V09-WithPicLowHDD`
- Renamed **"PooKooli Fountain"** everywhere
- **Continuous mode** — pump runs until stopped from the app or the physical button

### v3 — `V04-Iter2`
- 0.91" SSD1306 OLED for status and errors
- Empty-tank error shown on OLED *and* web, with no timer
- Switched to the digital `DO` pin, logic inverted to match real sensor behaviour
- Relay set to `OUTPUT` with `pumpOFF()` at boot, so the pump is always off until triggered

### v2 — `V03-Iter1`
All three pump timers made configurable live from the web app, saved to flash so they
survive power cuts.

### v1 — `V02-Base`
The first real build: ESP32 NodeMCU-32, touch + PIR + water sensor + relay, with three
trigger sources (touch 1 min, PIR 1 min, WiFi 2 min).

### origin — `V01-Origin`
Where everything started, and nothing like where it ended: **BLE** instead of WiFi
(`BLEDevice`/`BLEServer`), PIR on GPIO5, touch on GPIO4, water on GPIO34, pump MOSFET on
GPIO16, status LED on GPIO2. No web app, no display, no persistence.
