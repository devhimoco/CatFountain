# Beta — the test track

**17 sketches · `BETA V1.0` → `BETA V1.6`**

The rule for this track, stated in every header:

> **BETA Vx.y — test track: `+0.1` per module added.**
> Modules included in this build, **ONLY**.

Each header carries an explicit ✓/✗ checklist of what is in the build and what is
deliberately still left out. That discipline is why the complete editions in
[`../Alfa-X/`](../Alfa-X/) came together as smoothly as they did.

---

## Versions

| # | Folder | Version | What this build added |
|---|---|---|---|
| 01 | `V01-Base` | BETA V0 | The original all-in base sketch — two sensors *and* water sensor, before the decision to strip back and add one thing at a time. |
| 02 | `V02-V1.0` | **V1.0** | 🔑 Deliberately **stripped down**: one sensor only, sensor #2 detached, water detached (test mode locked at 50 %). The discipline starts here. |
| 04 | `V04-V1.1-WorkGood` | **V1.1** | Sensor #2 attached. Both lasers live at `0x30`/`0x31`, 5–70 cm range. |
| 05 | `V05-V1.1-MoreSenseSpeed` | V1.1 | Faster sampling — reduced trigger latency. |
| 06 | `V06-V1.1.0` | V1.1.0 | 🔑 **Timer renewal**: a laser-triggered run renews itself while the cat stays in range, instead of cutting out mid-drink. Directly answers item 4 of the original [`Notes-MustDo.txt`](../../Notes-MustDo.txt). |
| 10 | `V10-V1.2-Sensor2LCD` | **V1.2** | 0.96" SSD1306 **OLED** — status plus live cm readout. |
| 11 | `V11-V1.3.1` | **V1.3.1** | **Touch sensor** (capacitive, active-HIGH) as a universal manual override. |
| 12 | `V12-V1.3.2` | V1.3.2 | Touch refinements. |
| 13 | `V13-V1.4` | **V1.4** | **Water sensor attached** — real ADC readings at last; test mode becomes optional rather than mandatory. |
| 14 | `V14-V1.5.0` | **V1.5.0** | Start of the stabilisation series. |
| 16 | `V16-V1.5.1` | V1.5.1 | Stabilisation. |
| 17 | `V17-V1.5.2` | V1.5.2 | Stabilisation. |
| 18 | `V18-V1.5.3` | V1.5.3 | Stabilisation. |
| 19 | `V19-V1.5.4` | V1.5.4 | Stabilisation. |
| 20 | `V20-V1.5.5` | V1.5.5 | Stabilisation. |
| 21 | `V21-V1.5-Final` | V1.5 final | Last of the 1.5.x line. |
| 22 | `V22-V1.6` | **V1.6** | 🏁 **Final beta.** I²C bus recovery — automatic on sensor dropout plus a manual "Recover I2C Bus" button. 433 MHz remote formally removed. Test track complete. |

Gaps in the numbering (`03`, `07`, `08`, `09`, `15`) are the
[module tests](../ModuleTest/) — they share one chronological sequence with this folder
because that is genuinely when they happened.

---

## Reading the headers

Every sketch header states exactly what is present and what is not, for example
`V02-V1.0`:

```
✓ 1× VL53L0X laser sensor #1 (XSHUT, addr 0x30)
✗ Laser sensor #2 DETACHED in this build
✗ Water sensor DETACHED — test mode locked ON (50%)
✓ MOSFET pump (LEDC PWM, 10 power levels)
✓ Hardware watchdog + NVS-saved settings

NOT included yet (add later, one at a time, +0.1):
✗ Laser sensor #2 (set SENSOR2_ATTACHED to 1)
✗ Water sensor (set WATER_ATTACHED to 1)
✗ OLED display
```

Note that detaching is done with **compile-time flags** (`SENSOR2_ATTACHED`,
`WATER_ATTACHED`, `OLED_ATTACHED`), not by deleting code. The same sketch can be built with
a module in or out, which makes bisecting a fault genuinely quick — flip one flag, reflash,
see if the symptom moves.

## A note on `V12-V1.3.2/`

That folder contains nested copies of several earlier version folders. They are
**byte-identical duplicates** of the siblings one level up (verified with `cmp`) — an
accidental nesting during the original archiving, not distinct versions. They are excluded
from the repository via [`.gitignore`](../../.gitignore) so the history reads cleanly; only
`V12-V1.3.2.ino` itself is tracked here.
