# Generation X — Laser time-of-flight

**44 sketches · the current generation · VL53L0X**

Two VL53L0X laser time-of-flight sensors sharing one I²C bus, each reassigned its own
address at boot via `XSHUT`. This is the generation that finally answered the question the
previous two couldn't: *is the cat at the bowl right now?*

- **Millimetre accuracy**, not a vague presence signal
- **No dependency on body heat or motion** — a cat sitting perfectly still is still seen
- **Narrow, predictable beams** — two of them cover the bowl properly, where one 15°
  ultrasonic cone did not

---

## How this generation is organised

Generation X was built with a deliberate discipline that the earlier ones lacked:

> **Prove each module alone, then add exactly one module per version.**

That produces three parallel tracks, all sharing one chronological numbering sequence
(which is why the numbers interleave rather than running straight through each folder):

| Folder | What it holds | Count |
|---|---|---|
| [`ModuleTest/`](ModuleTest/) | Each module tested **in isolation** — no pump, no other sensors | 6 |
| [`Beta/`](Beta/) | The test track, `BETA V1.0` → `V1.6`, one module added at a time | 17 |
| [`Alfa-X/`](Alfa-X/) | Complete editions, `X V1.0` → `X V2.7` | 21 |

So `V03` (a module test) sits chronologically between `V02` and `V04` (Beta builds),
because that is genuinely when it happened: the sensor was proven alone, then folded into
the next build.

### Versioning

- **BETA Vx.y** — test track: `+0.1` per module proven and added
- **X Vx.y** — complete edition: `+0.1` small change, `+1.0` big change

`BETA V1.6` is the last test build. `X V1.0` is the first complete edition. `X V2.0` earned
a full point for a ground-up UI rewrite plus two new subsystems.

---

## The arc

```
ModuleTest         Beta (test track)              Alfa-X (complete editions)
───────────        ─────────────────              ──────────────────────────
V03 sensor  ──┐
V07 dual    ──┼──► V01 base → V1.0 → V1.1 ──┐
V08 oled    ──┤     ↓ +OLED (V1.2)          │
V09 recovery──┤     ↓ +touch (V1.3)         │
V15 rf ✗    ──┘     ↓ +water (V1.4)         │
                    ↓ stabilise (V1.5.x)    │
                    ↓ +I²C recovery (V1.6) ─┴──► X V1.0 ─► X V1.1 ─► X V2.0
                                                                        ↓
                                              X V2.4 ◄─ V2.3 ◄─ V2.2 ◄─ V2.1
                                              (crash hunt)
                                                │
                                                ▼
                                    X V2.5 (BLE remote) ─► V2.6 ─► V2.7
                                                            (water-level fixes)
```

`X V2.1` through `X V2.4` is **not** a feature run. It is a sustained hunt for a crash that
only appeared overnight, and it produced the diagnostics subsystem that is arguably the most
useful thing in the whole project. `X V2.5`–`X V2.7` build on that stable base: a physical
BLE remote, then two rounds of fixing false "tank empty" trips in the water-level reading.
See [`Alfa-X/`](Alfa-X/) for both stories.

---

## Hardware

| Module | Connections |
|---|---|
| VL53L0X #1 | `SDA`→GPIO21 · `SCL`→GPIO22 · `XSHUT`→GPIO32 → addr `0x30` |
| VL53L0X #2 | `SDA`→GPIO21 · `SCL`→GPIO22 · `XSHUT`→GPIO33 → addr `0x31` |
| Water level | `S`→GPIO34 (ADC1 — must not be ADC2, which WiFi blocks) |
| OLED SSD1306 | `SDA`→GPIO21 · `SCL`→GPIO22 · addr `0x3C` |
| Touch | `SIG`→GPIO13 (active-HIGH) |
| Pump MOSFET | `PWM`→GPIO26 |

Full schematic: [`../assets/schematic.svg`](../assets/schematic.svg) ·
Electrical notes: [`../docs/HARDWARE.md`](../docs/HARDWARE.md)

> **Both sensors boot at `0x29`.** `XSHUT` is the only reason two can share one bus. The
> firmware claims both pins and forces them LOW as the *very first statement* in `setup()`
> — before `Serial.begin()`, before the pump — because any delay lets a floating pin wake a
> sensor early and grab `0x29`, which caused random, varying boot failures.

---

## Where to start

- **Just want it running?** → [`../stable/`](../stable/)
- **Debugging one module?** → [`ModuleTest/`](ModuleTest/)
- **Want to understand the build-up?** → [`Beta/`](Beta/), in order
- **Want the crash-hunt story?** → [`Alfa-X/`](Alfa-X/), `V31` onward
