# Alfa-X — complete editions

**14 sketches · `X V1.0` → `X V2.4`**

Where the test track graduates. Every module verified individually *and* together, then
the versioning switches:

> **X Vx.y — complete edition: `+0.1` small change, `+1.0` big change.**

The last four versions are not a feature run. They are a **crash investigation**, and they
produced the most useful subsystem in the project.

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
| 35 | `V35AlfaX-V2.4-…-FirstAllOk` | **X V2.4** | ⭐ Everything working. Promoted to [`../../stable/`](../../stable/). |

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
