# Alfa-X — complete editions

**21 sketches · `X V1.0` → `X V2.7`**

Where the test track graduates. Every module verified individually *and* together, then
the versioning switches:

> **X Vx.y — complete edition: `+0.1` small change, `+1.0` big change.**

`V31`–`V39` are not a feature run. They are a **crash investigation** — first solved in
`V35`, then independently re-verified fix-by-fix in `V36`–`V39` until the result matched
`V35` exactly. `V41`–`V43` pick up from there with a physical remote and a water-sensor
recalibration.

---

## Versions

| # | Folder | Version | What changed |
|---|---|---|---|
| 22-1 | `V22-1-AlfaX-V1.0-BeforeFixDelay-…-OldDesign` | **X V1.0** | First complete edition, old UI. Kept for comparison. |
| 23 | `V23-AlfaX-V1.0` | X V1.0 | 🎓 Graduated from the BETA track (V1.0 → V1.6). |
| 24 | `V24-AlfaX-V1.0-BeforeFixDelay` | X V1.0 | Pre-delay-fix build, kept as a reference point. |
| 25 | `V25-AlfaX-V1.1.0-NewDesign-AddSensorNavDesign` | **X V1.1** | New sidebar navigation, sensors page. |
| 26 | `V26-…-BestChangesDesign1` | X V1.1 | Design iteration 1. |
| 27 | `V27-…-BestChangesDesign2` | X V1.1 | Design iteration 2. |
| 28 | `V28-…-BestChangesDesign3` | X V1.1 | Design iteration 3. |
| 29 | `V29-AlfaX-V1.1-FullResponsiveDesign` | X V1.1 | Full responsive pass — the layout that became V2.0. |
| 30 | `V30AlfaX-V2.0-FullResponsiveDesign` | **X V2.0** | 🎨 **Ground-up rewrite.** Sidebar dashboard collapsing to a drawer on phones · **Schedule** (NTP daily triggers, persisted, routed through the same `startPump()`/`triggerBlocked()` path as every other source — no separate safety logic) · **Recent Activity** log (15-entry RAM ring buffer). Bumped `+1.0`, not `+0.1`, by the project's own rule. |
| 31 | `V31AlfaX-V2.1-DebugRestart` | **X V2.1** | 🔍 The hunt begins — see below. |
| 32 | `V32AlfaX-V2.2-Debug-RestartOverNight` | **X V2.2** | 🔍 A confirmed root cause, fixed. |
| 33 | `V33AlfaX-V2.3-Debug-RestartOverNight2` | **X V2.3** | 🔍 "When and why" wasn't enough — needed "**where**". |
| 34 | `V34AlfaX-V2.4-Debug` | **X V2.4** | 🔍 Blind spot closed. Also mirrored to [`../../stable/`](../../stable/) as the no-Schedule variant. |
| 35 | `V35AlfaX-V2.4-…-FirstAllOk` | **X V2.4** | Everything working. Mirrored to [`../../stable/`](../../stable/). |
| 36 | `V36Alfa-X-V2.4.1-FRestartDuring` | X V2.4.1 | 🔁 Restarted from `V34`'s baseline to re-verify the V2.4 fixes one at a time instead of trusting them bundled. |
| 37 | `V37AlfaX-V2.4.2-FFalseTrigger` | X V2.4.2 | Laser confirm window raised `150ms → 600ms` (rejects brief false triggers) · pump **soft-start** ramp added — the inrush current spike from slamming the MOSFET straight to full duty was a real brown-out-reset candidate. |
| 38 | `V38AlfaX-V2.4.3-FirstTrySchedule` | X V2.4.3 | I²C bus recovery now **deferred while the pump is running** — `sensor.begin()` mid-recovery on a bus the motor is actively flooding with EMI was panicking. Pump **soft-stop** ramp added. NTP is now (re)requested on every WiFi reconnect, not just once in `setup()` — on a slow router, WiFi coming up after the 15 s boot window meant NTP never started and Schedule silently never fired. |
| 39 | `V39AlfaX-V2.4.4-AllFuncFix` | **X V2.4** | ✅ Re-added per-day/one-shot Schedule entries and a browser-clock fallback (`/settime`, for networks that block outbound NTP) — landing **byte-identical to `V35`**. The re-verification closes here. |
| 41 | `V41AlfaX-V2.5-Debug-AddBLRemoteLatest` | **X V2.5** | 📻 **BLE iTag remote.** ESP32 connects as a BLE client straight to a cheap iTag keyfob (service `FFE0`/char `FFE1`) — no phone app needed. Button press toggles the pump **continuous**, same override precedence as Touch. Also adds a **Sensor Test Mode** bench toggle that quiets I²C-recovery retries and debug prints when sensors are unplugged. See [`../ModuleTest/`](../ModuleTest/) for the standalone protocol-discovery sketch that preceded this. |
| 42 | `V42AlfaX-V2.6-Debug-ERRDntShow` | X V2.6 | Water-level ADC now reads a **median of 7 samples** instead of one — a single noisy reading during pump inrush was enough to trip a false "tank empty" block. Thresholds recalibrated (`DRY`/`FULL`/`LOW_PCT`). |
| 43 | `V43AlfaX-V2.7-Debug-WaterLevelFix` | ⭐ **X V2.7** | Water ADC thresholds recalibrated again with better (though still marked **INTERIM/estimated**) numbers. **Promoted to [`../../stable/`](../../stable/)** as the current recommended build. |

`V35`/`V39`'s exact build is also maintained as its own standalone repository —
**[PooKooli-Fountain-X-V2.4](https://github.com/devhimoco/PooKooli-Fountain-X-V2.4)**, a
quick-start README and `secrets.h.example` in place of the full archive here — and is the
firmware actually flashed on the physical fountain right now, predating the V2.5–V2.7 work
above. It isn't copied into this repo (see [`VERSION-MANIFEST.md`](../../VERSION-MANIFEST.md)
for why).

---

## 🔍 The crash hunt (V2.1 → V2.4)

The board was rebooting overnight. No one is watching Serial at 3 a.m., and the RAM
activity log is wiped by the very reset being investigated. Four versions of chasing it:

### V2.1 — *make the crash leave evidence*
Reset reason + free heap written to **flash on every boot**, surviving even a brownout or
watchdog reset, plus a minimum-free-heap tracker. Both surfaced on the Device Info page.
For the first time the fountain could report *why* it had restarted.

### V2.2 — *a confirmed cause, found by V2.1's own log*
`esp_task_wdt_init()` was failing **silently**. Newer ESP32 cores auto-start a default task
watchdog, so the call returned "already initialized" and the intended 10 s timeout never
took effect — a shorter hidden default stayed active and fired mid-I²C-recovery. Fixed by
deinitialising the existing watchdog first.

### V2.3 — *"why" still wasn't "where"*
A full overnight Boot History (10/10 slots: PANIC + Task watchdog + Power-on) showed V2.2
helped but wasn't enough. Reset reason tells you *when* and *why*, never **where**. So: a
crash-location breadcrumb in **RTC memory** — which survives panics, watchdog resets and
software resets — updated at every major loop section and densely through I²C recovery,
read back on the next boot and stored alongside each Boot History entry.

### V2.4 — *the blind spot*
A full night came back **10/10 PANIC, every entry still reading checkpoint `"boot"`**.
V2.3's breadcrumbs only covered `loop()` and the recovery path, so a whole region was
genuinely invisible. V2.4 added checkpoints through **every phase of `setup()`** — I²C
recovery, sensor/OLED init, NVS load, WiFi connect, NTP, web routes, watchdog config — and
fed the watchdog during the WiFi connect wait, a loop that can block up to 15 s and
previously had zero resets across the whole window.

---

## Why this matters more than a feature

The diagnostics built here turn "it randomly reboots" into a specific, actionable line:

```
PANIC (crash) @ i2cinit:sensor2    heap 271 KB
```

That is a reset reason, a **crash location**, and a heap figure proving it wasn't memory
exhaustion — recorded to flash, surviving the crash itself, readable from a web page the
next morning. On a headless embedded device that runs unattended, that capability is worth
more than any single feature on the dashboard.

---

## Archive note

Two sketches here had `.ino` filenames that didn't match their folder, which stops the
Arduino IDE opening them cleanly. Both were corrected (the `V32` folder's file was named
`V30…`, a copy-paste typo). This matches the filename-typo fixes already recorded in
[`VERSION-MANIFEST.md`](../../VERSION-MANIFEST.md); no code was touched.

A folder numbered `V40` also exists locally but isn't tracked here: it's a byte-for-byte
duplicate of `V35` (its `.ino` is even still named after `V35`'s folder), saved as a
checkpoint right before the V2.5 branch started. Excluded via `.gitignore`, same as the
nested Beta duplicates — see [`VERSION-MANIFEST.md`](../../VERSION-MANIFEST.md).
