/*
 ╔══════════════════════════════════════════════════════════╗
 ║     PooKooli Fountain — ESP32 NodeMCU-32  v4             ║
 ║                                                          ║
 ║  Changes from v3:                                        ║
 ║   ✓ Renamed to "PooKooli Fountain" everywhere            ║
 ║   ✓ Continuous mode: pump runs forever until stopped     ║
 ║     via app Stop button or physical touch button         ║
 ║   ✓ Relay logic fixed for ACTIVE HIGH via NPN transistor ║
 ╚══════════════════════════════════════════════════════════╝

 ── INSTALL THESE LIBRARIES (Arduino Library Manager) ──────
   • Adafruit SSD1306   (by Adafruit)
   • Adafruit GFX       (by Adafruit)

 ── RELAY PROBLEM & SOLUTION ────────────────────────────────
  Problem:
    The blue Tongling relay module coil needs 5V to trigger.
    ESP32 GPIO outputs only 3.3V.  The module has an internal
    pull-up to 5V on the IN pin, so idle = 3.3V (coil ON!) and
    driven LOW = 0V (coil OFF).  This is why the pump was
    always running — the "active-low" trick kept it on.

  Best fix — use the relay in ACTIVE HIGH mode with a simple
  NPN transistor level-shifter (e.g. 2N2222, BC547, S8050):

      GPIO 26 ──[1kΩ]── Base  (NPN transistor)
      GND     ───────── Emitter
      Relay IN ──────── Collector
      Relay VCC → 5V (Vin), Relay GND → GND

  How it works:
    GPIO LOW  (0V)  → transistor OFF → relay IN floats HIGH
                       via internal pull-up → coil OFF → pump OFF ✓
    GPIO HIGH (3.3V) → transistor ON  → relay IN pulled to GND
                       (0V) → coil ON → pump ON ✓

  Parts needed:  1× NPN transistor + 1× 1kΩ resistor (cheap!)

  Alternative (no transistor):
    Some relay modules have a jumper to disconnect the internal
    pull-up (labelled "JD-VCC" or "VCC-JD").  If yours has it,
    separate JD-VCC from VCC, power JD-VCC from 5V and VCC from
    3.3V.  Then set RELAY_ACTIVE_LOW false below and it works
    directly without a transistor.

 ── WIRING ─────────────────────────────────────────────────
  OLED 0.91" SSD1306 (I2C)
    VCC → 3.3V
    GND → GND
    SCL → GPIO 22
    SDA → GPIO 21

  Touch Sensor (red module)
    VCC → 3.3V  |  GND → GND  |  OUT → GPIO 4

  PIR Sensor HC-SR501
    VCC → 5V    |  GND → GND  |  OUT → GPIO 14

  Water Level Sensor HW-101
    VCC → 3.3V  |  GND → GND  |  DO  → GPIO 34
    (DO = digital output; LOW when submerged = water OK)

  Relay Module (with NPN transistor fix)
    Relay VCC  → Vin (5V from USB)
    Relay GND  → GND
    Relay IN   → Collector of NPN transistor
    NPN Base   → GPIO 26 via 1kΩ resistor
    NPN Emitter → GND
    Relay COM  → pump positive wire
    Relay NO   → power-supply positive
    (pump negative → power-supply negative)
 ───────────────────────────────────────────────────────────
*/

#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ── USER CONFIG ────────────────────────────────────────────
const char* WIFI_SSID     = "HiMo 2G";
const char* WIFI_PASSWORD = "@HiMo9226#";

// GPIO
#define PIN_TOUCH        4
#define PIN_PIR         14
#define PIN_WATER_DO    34   // DO pin — INPUT_PULLUP; LOW = water present
#define PIN_RELAY       26

// Relay: with NPN transistor fix → ACTIVE HIGH
//   GPIO HIGH (3.3V) → transistor ON → relay IN = 0V → coil ON → pump ON
//   GPIO LOW  (0V)   → transistor OFF → relay IN = 5V → coil OFF → pump OFF
// If you use the JD-VCC jumper method instead, same setting works.
// Only set to true if you have a relay module that works directly
// with 3.3V logic without any transistor (rare).
#define RELAY_ACTIVE_LOW  false

// Durations (seconds) — editable from web UI, saved to flash
#define DEFAULT_TOUCH_SEC   60
#define DEFAULT_PIR_SEC     60
#define DEFAULT_WIFI_SEC   120
#define MIN_SEC   5
#define MAX_SEC   600

// Debounce / lockout
#define TOUCH_DEBOUNCE_MS   500
#define PIR_LOCKOUT_MS     5000
#define EMPTY_BLINK_MS     3000   // how long to show "tank empty" error on OLED

// OLED
#define OLED_WIDTH  128
#define OLED_HEIGHT  32
#define OLED_ADDR   0x3C
#define OLED_RESET   -1

// ── GLOBALS ────────────────────────────────────────────────
WebServer    server(80);
Preferences  prefs;
Adafruit_SSD1306 oled(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RESET);

unsigned long touchDurationMs = DEFAULT_TOUCH_SEC * 1000UL;
unsigned long pirDurationMs   = DEFAULT_PIR_SEC   * 1000UL;
unsigned long wifiDurationMs  = DEFAULT_WIFI_SEC  * 1000UL;

enum PumpSource { SRC_NONE, SRC_TOUCH, SRC_PIR, SRC_WIFI };

bool          pumpRunning    = false;
bool          pumpContinuous = false;   // true = run forever (no timer)
PumpSource    pumpSource     = SRC_NONE;
unsigned long pumpStartTime  = 0;
unsigned long pumpDuration   = 0;
bool          tankEmptyError = false;   // set when trigger fires but tank is empty
unsigned long emptyErrorTime = 0;       // when the error was set

unsigned long lastTouchTime  = 0;
unsigned long lastPirEndTime = 0;
unsigned long lastOledUpdate = 0;

// ── FLASH ──────────────────────────────────────────────────
void loadSettings() {
  prefs.begin("fountain", true);
  touchDurationMs = prefs.getULong("touch_ms", DEFAULT_TOUCH_SEC * 1000UL);
  pirDurationMs   = prefs.getULong("pir_ms",   DEFAULT_PIR_SEC   * 1000UL);
  wifiDurationMs  = prefs.getULong("wifi_ms",  DEFAULT_WIFI_SEC  * 1000UL);
  prefs.end();
}
void saveSettings() {
  prefs.begin("fountain", false);
  prefs.putULong("touch_ms", touchDurationMs);
  prefs.putULong("pir_ms",   pirDurationMs);
  prefs.putULong("wifi_ms",  wifiDurationMs);
  prefs.end();
}

// ── WATER SENSOR ───────────────────────────────────────────
// HW-101 DO: LOW when probes submerged (water present)
//            HIGH when dry
// Your sensor is inverted vs the label, so:
//   digitalRead LOW  → has water   ✓
//   digitalRead HIGH → empty       ✓
bool hasWater() {
  // Use INPUT_PULLUP so floating pin reads HIGH (safe = empty)
  return digitalRead(PIN_WATER_DO) == LOW;
}

// ── RELAY / PUMP ───────────────────────────────────────────
void pumpON() {
  digitalWrite(PIN_RELAY, RELAY_ACTIVE_LOW ? LOW : HIGH);
}
void pumpOFF() {
  digitalWrite(PIN_RELAY, RELAY_ACTIVE_LOW ? HIGH : LOW);
  pumpRunning    = false;
  pumpContinuous = false;
  pumpSource     = SRC_NONE;
}
void startPump(PumpSource src, unsigned long dur) {
  tankEmptyError = false;
  pumpRunning    = true;
  pumpContinuous = false;
  pumpSource     = src;
  pumpStartTime  = millis();
  pumpDuration   = dur;
  pumpON();
  Serial.printf("[Pump] ON  src=%d  dur=%lus\n", src, dur / 1000);
}
void startPumpForever(PumpSource src) {
  tankEmptyError = false;
  pumpRunning    = true;
  pumpContinuous = true;
  pumpSource     = src;
  pumpStartTime  = millis();
  pumpDuration   = 0;
  pumpON();
  Serial.printf("[Pump] ON  src=%d  CONTINUOUS\n", src);
}
void triggerBlocked(PumpSource src) {
  tankEmptyError = true;
  emptyErrorTime = millis();
  if (src == SRC_NONE)
    Serial.println("[Pump] STOPPED — tank ran empty during run");
  else
    Serial.printf("[Pump] BLOCKED src=%d — tank empty\n", src);
}

// ── OLED DISPLAY ───────────────────────────────────────────
// Called regularly from loop(); keeps display in sync with state.
void updateOled() {
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextWrap(false);   // ← disables auto-wrap; text is clipped instead of wrapping

  // ── ERROR: tank empty ──────────────────────────────────
  if (tankEmptyError) {
    oled.setTextSize(1);
    oled.setCursor(0,  0); oled.print(F("!! TANK EMPTY !!"));
    oled.setCursor(0, 12); oled.print(F("Please refill"));
    oled.setCursor(0, 24); oled.print(F("water reservoir"));
    oled.display();
    return;
  }

  // ── PUMP RUNNING ───────────────────────────────────────
  if (pumpRunning) {
    oled.setTextSize(1);

    // Row 0 — source label  (max 21 chars)
    const char* srcLabel =
      (pumpSource == SRC_TOUCH) ? "Pump ON-Touch" :
      (pumpSource == SRC_PIR)   ? "Pump ON-PIR"   :
                                  "Pump ON-App";
    oled.setCursor(0, 0);
    oled.print(srcLabel);

    if (pumpContinuous) {
      // Row 1 — elapsed  e.g. "~~ 04:32 ~~"
      unsigned long elSec = (millis() - pumpStartTime) / 1000;
      char buf[14];
      snprintf(buf, sizeof(buf), "~~ %02lu:%02lu ~~",
               elSec / 60, elSec % 60);
      oled.setCursor(16, 13);
      oled.print(buf);
      // Row 2
      oled.setCursor(8, 24);
      oled.print(F("CONTINUOUS MODE"));

    } else {
      // Row 1+2 — big countdown centred
      unsigned long elapsed = millis() - pumpStartTime;
      unsigned long remain  = (elapsed >= pumpDuration) ? 0
                                                        : (pumpDuration - elapsed);
      unsigned int m = (remain / 1000) / 60;
      unsigned int s = (remain / 1000) % 60;
      char buf[6];
      snprintf(buf, sizeof(buf), "%02d:%02d", m, s);
      oled.setTextSize(2);
      // Each char is 12 px wide at size 2; centre in 128 px
      int x = (OLED_WIDTH - (int)strlen(buf) * 12) / 2;
      oled.setCursor(max(0, x), 12);
      oled.print(buf);
    }

    oled.display();
    return;
  }

  // ── IDLE ───────────────────────────────────────────────
  bool water = hasWater();
  oled.setTextSize(1);

  // Row 0 — device name  (17 chars, fits fine)
  oled.setCursor(0, 0);
  oled.print(F("PooKooli Fountain"));

  // Row 1 — IP address or short error  (max 21 chars)
  oled.setCursor(0, 11);
  if (WiFi.status() == WL_CONNECTED) {
    // IP like "192.168.xxx.xxx" = 15 chars max
    oled.print(WiFi.localIP().toString());
  } else {
    oled.print(F("No WiFi"));
  }

  // Row 2 — water status  (max 21 chars)
  oled.setCursor(0, 22);
  if (water) {
    oled.print(F("Water OK - Ready"));
  } else {
    if ((millis() / 600) % 2 == 0)
      oled.print(F("!! TANK EMPTY !!"));
    else
      oled.print(F("Add water please"));
  }

  oled.display();
}

// ── JSON ───────────────────────────────────────────────────
String srcStr() {
  if (pumpSource == SRC_TOUCH) return "TOUCH";
  if (pumpSource == SRC_PIR)   return "PIR";
  if (pumpSource == SRC_WIFI)  return "WIFI";
  return "NONE";
}

String buildStatusJson() {
  unsigned long elapsed = pumpRunning ? (millis() - pumpStartTime) : 0;
  bool water = hasWater();
  String j = "{";
  j += "\"running\":"       + String(pumpRunning     ? "true" : "false") + ",";
  j += "\"continuous\":"    + String(pumpContinuous  ? "true" : "false") + ",";
  j += "\"empty_error\":"   + String(tankEmptyError  ? "true" : "false") + ",";
  j += "\"source\":\""      + srcStr() + "\",";
  j += "\"locked\":"        + String((pumpRunning && pumpSource != SRC_WIFI) ? "true" : "false") + ",";
  j += "\"water\":"         + String(water           ? "true" : "false") + ",";
  j += "\"elapsed_ms\":"    + String(elapsed) + ",";
  j += "\"duration_ms\":"   + String(pumpDuration) + ",";
  j += "\"touch_sec\":"     + String(touchDurationMs / 1000) + ",";
  j += "\"pir_sec\":"       + String(pirDurationMs   / 1000) + ",";
  j += "\"wifi_sec\":"      + String(wifiDurationMs  / 1000);
  j += "}";
  return j;
}

// ── HTML ───────────────────────────────────────────────────
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1">
<title>PooKooli Fountain</title>
<meta name="mobile-web-app-capable" content="yes">
<meta name="apple-mobile-web-app-capable" content="yes">
<meta name="apple-mobile-web-app-status-bar-style" content="black-translucent">
<meta name="apple-mobile-web-app-title" content="PooKooli">
<meta name="theme-color" content="#07090f">
<link rel="apple-touch-icon" href="data:image/png;base64,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">
<link rel="icon" type="image/png" sizes="192x192" href="data:image/png;base64,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">
<link rel="manifest" href="/manifest.json">
<style>
@import url('https://fonts.googleapis.com/css2?family=Syne:wght@400;700;800&family=DM+Sans:wght@400;500&display=swap');
*{margin:0;padding:0;box-sizing:border-box;-webkit-tap-highlight-color:transparent}
:root{
  --bg:#07090f;--card:#0f1520;--card2:#131b28;
  --accent:#38bdf8;--on:#4ade80;--off:#f87171;--warn:#fbbf24;--inf:#a78bfa;
  --text:#e8f0fe;--muted:#4a5a72;--border:#1a2540;--radius:18px;
}
body{
  font-family:'DM Sans',sans-serif;background:var(--bg);color:var(--text);
  min-height:100vh;display:flex;flex-direction:column;align-items:center;
  padding:0 14px 56px;
  background-image:
    radial-gradient(ellipse 70% 35% at 50% -5%,rgba(56,189,248,.13) 0%,transparent 65%),
    radial-gradient(ellipse 40% 25% at 80% 90%,rgba(74,222,128,.07) 0%,transparent 60%);
}
.hdr{text-align:center;margin-top:0;width:100%;max-width:380px}
.cat-banner{
  width:100%;height:210px;object-fit:cover;object-position:center 15%;
  border-radius:20px 20px 0 0;display:block;
}
.hdr-text{
  background:var(--card);
  border:1px solid var(--border);border-top:none;
  border-radius:0 0 20px 20px;
  padding:14px 16px 18px;
}
h1{font-family:'Syne',sans-serif;font-size:1.7rem;font-weight:800;letter-spacing:-.03em;
   background:linear-gradient(120deg,#e0f2fe,#38bdf8 50%,#818cf8);
   -webkit-background-clip:text;-webkit-text-fill-color:transparent;
   margin:0}
.sub{font-size:.78rem;color:var(--muted);margin-top:3px}

.card{background:var(--card);border:1px solid var(--border);border-radius:var(--radius);
      padding:20px;width:100%;max-width:380px;margin-top:18px}
.card-title{font-family:'Syne',sans-serif;font-size:.65rem;font-weight:700;
            letter-spacing:.15em;text-transform:uppercase;color:var(--muted);margin-bottom:16px}

/* ── Error banner ── */
.error-banner{
  display:none;
  background:rgba(248,113,113,.12);border:1px solid rgba(248,113,113,.4);
  border-radius:12px;padding:14px 16px;margin-bottom:14px;
}
.error-banner.show{display:flex;align-items:center;gap:12px}
.error-ico{font-size:1.8rem;flex-shrink:0}
.error-title{font-family:'Syne',sans-serif;font-size:.95rem;font-weight:700;color:var(--off)}
.error-msg{font-size:.78rem;color:#fca5a5;margin-top:2px}

/* ── Status ── */
.status-row{display:flex;align-items:center;gap:12px}
.dot{width:12px;height:12px;border-radius:50%;flex-shrink:0;transition:all .4s}
.dot.on{background:var(--on);box-shadow:0 0 0 4px rgba(74,222,128,.2),0 0 12px rgba(74,222,128,.5)}
.dot.off{background:var(--muted)}
.dot.err{background:var(--off);box-shadow:0 0 0 4px rgba(248,113,113,.2),0 0 12px rgba(248,113,113,.5)}
.dot.inf{background:var(--inf);box-shadow:0 0 0 4px rgba(167,139,250,.2),0 0 12px rgba(167,139,250,.6);
         animation:pulse 1.4s ease-in-out infinite}
@keyframes pulse{0%,100%{opacity:1}50%{opacity:.4}}
.s-label{font-family:'Syne',sans-serif;font-size:1rem;font-weight:700}
.s-sub{font-size:.78rem;color:var(--muted);margin-top:2px}
.s-sub.red{color:#fca5a5}
.s-sub.inf{color:var(--inf)}

/* ── Timer bar ── */
.timer-wrap{margin-top:18px;display:none}
.timer-wrap.active{display:block}
.t-row{display:flex;justify-content:space-between;font-size:.75rem;color:var(--muted);margin-bottom:7px}
.t-time{font-family:'Syne',sans-serif;font-weight:700;font-size:.85rem;color:var(--accent)}
.t-time.inf{color:var(--inf)}
.bar-bg{height:5px;background:var(--border);border-radius:99px;overflow:hidden}
.bar-fill{height:100%;width:0%;border-radius:99px;transition:width .8s linear;
          background:linear-gradient(90deg,var(--accent),var(--on))}
.bar-fill.inf{width:100%!important;background:linear-gradient(90deg,var(--inf),#c084fc);
              animation:shimmer 2s linear infinite;background-size:200% 100%}
@keyframes shimmer{0%{background-position:100% 0}100%{background-position:-100% 0}}

/* ── Chips ── */
.chips{display:flex;gap:7px;flex-wrap:wrap;margin-top:18px}
.chip{font-size:.7rem;font-weight:500;padding:4px 11px;border-radius:99px;
      background:var(--border);color:var(--muted);transition:all .3s;border:1px solid transparent}
.chip.active{background:rgba(56,189,248,.15);color:var(--accent);border-color:rgba(56,189,248,.3)}
.chip.water-ok{background:rgba(74,222,128,.12);color:var(--on);border-color:rgba(74,222,128,.3)}
.chip.water-no{background:rgba(248,113,113,.1);color:var(--off);border-color:rgba(248,113,113,.25)}
.chip.inf-chip{background:rgba(167,139,250,.15);color:var(--inf);border-color:rgba(167,139,250,.35)}

/* ── Buttons ── */
.btn-grid{display:grid;grid-template-columns:1fr 1fr;gap:10px;margin-top:16px}
.btn-grid-3{display:grid;grid-template-columns:1fr 1fr 1fr;gap:8px;margin-top:16px}
.btn{display:flex;flex-direction:column;align-items:center;justify-content:center;
     gap:4px;padding:13px 6px;border:none;border-radius:14px;
     font-family:'Syne',sans-serif;font-size:.82rem;font-weight:700;
     cursor:pointer;transition:transform .12s,opacity .15s}
.btn:active{transform:scale(.95);opacity:.8}
.btn .ico{font-size:1.35rem}
.btn-on {background:linear-gradient(145deg,#166534,#22c55e);color:#fff}
.btn-locked{opacity:.35;pointer-events:none;filter:grayscale(.7)}
.btn-locked::after{content:'🔒';position:absolute;top:6px;right:8px;font-size:.7rem}
.btn-inf{background:linear-gradient(145deg,#4c1d95,#a78bfa);color:#fff}
.btn-off{background:linear-gradient(145deg,#991b1b,#ef4444);color:#fff}
.btn-sub{font-size:.6rem;font-weight:400;opacity:.8;font-family:'DM Sans',sans-serif}

/* ── Settings ── */
.settings-card{background:var(--card2)}
.timer-setting{display:flex;align-items:center;justify-content:space-between;
               padding:13px 0;border-bottom:1px solid var(--border)}
.timer-setting:last-child{border-bottom:none}
.ts-label{font-size:.88rem;font-weight:500}
.ts-label small{display:block;font-size:.72rem;color:var(--muted);margin-top:1px}
.ts-controls{display:flex;align-items:center}
.ts-btn{width:34px;height:34px;border:1px solid var(--border);background:var(--bg);
        color:var(--text);font-size:1.1rem;font-weight:700;border-radius:8px;
        cursor:pointer;display:flex;align-items:center;justify-content:center;transition:background .15s}
.ts-btn:active{background:var(--border)}
.ts-val{min-width:54px;text-align:center;font-family:'Syne',sans-serif;
        font-size:.95rem;font-weight:700;color:var(--accent);padding:0 4px}
.save-btn{width:100%;margin-top:16px;padding:14px;
          background:linear-gradient(135deg,#1d4ed8,#38bdf8);color:#fff;
          border:none;border-radius:12px;font-family:'Syne',sans-serif;
          font-size:.95rem;font-weight:700;cursor:pointer;
          display:flex;align-items:center;justify-content:center;gap:8px;
          transition:opacity .15s,transform .1s}
.save-btn:active{transform:scale(.97);opacity:.85}

/* ── Toast ── */
.toast{position:fixed;bottom:24px;left:50%;
       transform:translateX(-50%) translateY(80px);
       background:#1e293b;color:var(--on);padding:10px 22px;
       border-radius:99px;font-size:.85rem;font-weight:600;
       border:1px solid rgba(74,222,128,.3);
       transition:transform .35s cubic-bezier(.34,1.56,.64,1),opacity .35s;
       opacity:0;white-space:nowrap;z-index:999}
.toast.show{transform:translateX(-50%) translateY(0);opacity:1}
.toast.err{color:var(--off);border-color:rgba(248,113,113,.4)}

.note{font-size:.7rem;color:var(--muted);margin-top:24px;text-align:center;opacity:.55;line-height:1.6}
</style>
</head>
<body>

<div class="hdr">
  <img class="cat-banner" src="data:image/jpeg;base64,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" alt="PooKooli">
  <div class="hdr-text">
    <h1>PooKooli Fountain</h1>
    <p class="sub">Smart water controller</p>
  </div>
</div>

<!-- STATUS CARD -->
<div class="card">
  <div class="card-title">Live Status</div>

  <!-- Empty tank error banner -->
  <div class="error-banner" id="errorBanner">
    <div class="error-ico">🪣</div>
    <div>
      <div class="error-title">Water tank is empty!</div>
      <div class="error-msg">Please refill the reservoir before using the pump.</div>
    </div>
  </div>

  <div class="status-row">
    <div class="dot" id="pumpDot"></div>
    <div>
      <div class="s-label" id="pumpLabel">Loading…</div>
      <div class="s-sub"   id="pumpSub"></div>
    </div>
  </div>

  <!-- Timer (only when pump running, not shown when empty error) -->
  <div class="timer-wrap" id="timerWrap">
    <div class="t-row">
      <span id="timerLabel">Pump running</span>
      <span class="t-time" id="timerText">--:--</span>
    </div>
    <div class="bar-bg"><div class="bar-fill" id="barFill"></div></div>
  </div>

  <div class="chips">
    <span class="chip" id="chipWater">💧 Water</span>
    <span class="chip" id="chipTouch">👆 Touch</span>
    <span class="chip" id="chipPir">🐾 PIR</span>
    <span class="chip" id="chipWifi">📱 App</span>
    <span class="chip" id="chipInf">∞ Continuous</span>
  </div>
</div>

<!-- CONTROL CARD -->
<div class="card" style="margin-top:12px">
  <div class="card-title">Control</div>
  <div class="btn-grid-3">
    <button class="btn btn-on" id="btnOn" onclick="sendCmd('on')">
      <span class="ico">▶</span>
      Start
      <span class="btn-sub" id="wifiDurLabel">2 min</span>
    </button>
    <button class="btn btn-inf" id="btnInf" onclick="sendCmd('continuous')">
      <span class="ico">∞</span>
      Infinite
      <span class="btn-sub">until stopped</span>
    </button>
    <button class="btn btn-off" onclick="sendCmd('off')">
      <span class="ico">⏹</span>
      Stop
      <span class="btn-sub">immediately</span>
    </button>
  </div>
</div>

<!-- SETTINGS CARD -->
<div class="card settings-card" style="margin-top:12px">
  <div class="card-title">⏱ Timer Settings</div>

  <div class="timer-setting">
    <div class="ts-label">👆 Touch Button<small>Physical touch sensor</small></div>
    <div class="ts-controls">
      <button class="ts-btn" onclick="adj('touch',-10)">−</button>
      <div class="ts-val" id="val-touch">60s</div>
      <button class="ts-btn" onclick="adj('touch',+10)">+</button>
    </div>
  </div>

  <div class="timer-setting">
    <div class="ts-label">🐾 PIR Sensor<small>Cat motion detected</small></div>
    <div class="ts-controls">
      <button class="ts-btn" onclick="adj('pir',-10)">−</button>
      <div class="ts-val" id="val-pir">60s</div>
      <button class="ts-btn" onclick="adj('pir',+10)">+</button>
    </div>
  </div>

  <div class="timer-setting">
    <div class="ts-label">📱 App Start Button<small>Started from this page</small></div>
    <div class="ts-controls">
      <button class="ts-btn" onclick="adj('wifi',-10)">−</button>
      <div class="ts-val" id="val-wifi">120s</div>
      <button class="ts-btn" onclick="adj('wifi',+10)">+</button>
    </div>
  </div>

  <button class="save-btn" onclick="saveSettings()">
    💾 Save Settings to Device
  </button>
</div>

<p class="note">Timers adjust in 10-second steps (5 s min · 600 s max)<br>
Settings saved to device flash — survive power cuts</p>

<div class="toast" id="toast"></div>

<script>
const timers = { touch: 60, pir: 60, wifi: 120 };
const MIN = 5, MAX = 600;
let pumpEndTime = 0, timerTick = null, lastDurationMs = 0;
let elapsedTick = null, elapsedStart = 0;

// ── Bug 1 fix: track when user is actively editing ──
let editingKey = null;       // which timer is being edited right now
let editIdleTimer = null;    // clears editingKey after 5 s of no taps

function adj(key, delta) {
  // Mark this key as being edited; restart the idle timeout
  editingKey = key;
  clearTimeout(editIdleTimer);
  editIdleTimer = setTimeout(() => { editingKey = null; }, 5000);

  timers[key] = Math.min(MAX, Math.max(MIN, timers[key] + delta));
  render(key);
}
function render(key) {
  const v = timers[key];
  const s = v >= 60 ? (Math.floor(v/60)+'m'+(v%60?v%60+'s':'')) : v+'s';
  document.getElementById('val-'+key).textContent = s;
  if (key==='wifi') document.getElementById('wifiDurLabel').textContent = s;
}

async function saveSettings() {
  editingKey = null;   // done editing, allow sync again
  clearTimeout(editIdleTimer);
  try {
    const r = await fetch(`/settings?touch=${timers.touch}&pir=${timers.pir}&wifi=${timers.wifi}`);
    const j = await r.json();
    showToast(j.ok ? '✓ Settings saved!' : '⚠ Save failed', !j.ok);
  } catch(e) { showToast('⚠ No response', true); }
}

async function sendCmd(cmd) {
  if (cmd !== 'off' && window._pumpLocked) return;
  try { applyStatus(await (await fetch('/pump?cmd='+cmd)).json()); } catch(e) {}
}

async function fetchStatus() {
  try { applyStatus(await (await fetch('/status')).json()); } catch(e) {}
}

function stopTimerTick()   { if(timerTick)  {clearInterval(timerTick);  timerTick=null;} }
function stopElapsedTick() { if(elapsedTick){clearInterval(elapsedTick);elapsedTick=null;} }
function stopTicks()       { stopTimerTick(); stopElapsedTick(); }

// ── Bug 3 fix: fully reset the timer bar ──
function resetTimerUI() {
  const wrap  = document.getElementById('timerWrap');
  const bar   = document.getElementById('barFill');
  const tt    = document.getElementById('timerText');
  wrap.classList.remove('active');
  bar.className  = 'bar-fill';
  bar.style.width = '0%';
  tt.className   = 't-time';
  tt.textContent = '0:00';
}

function applyStatus(d) {
  // ── Bug 1 fix: only sync timer values when user isn't editing that key ──
  if (d.touch_sec && editingKey !== 'touch') { timers.touch = d.touch_sec; render('touch'); }
  if (d.pir_sec   && editingKey !== 'pir')   { timers.pir   = d.pir_sec;   render('pir');   }
  if (d.wifi_sec  && editingKey !== 'wifi')  { timers.wifi  = d.wifi_sec;  render('wifi');  }

  const src = d.source;
  ['Touch','Pir','Wifi'].forEach(s =>
    document.getElementById('chip'+s).classList.toggle('active', src===s.toUpperCase()));

  document.getElementById('chipInf').className =
    'chip' + (d.continuous ? ' inf-chip' : '');

  const wc = document.getElementById('chipWater');
  wc.className   = 'chip '+(d.water?'water-ok':'water-no');
  wc.textContent = d.water ? '💧 Water OK' : '💧 Tank Empty';

  const dot    = document.getElementById('pumpDot');
  const label  = document.getElementById('pumpLabel');
  const sub    = document.getElementById('pumpSub');
  const tlabel = document.getElementById('timerLabel');
  const bar    = document.getElementById('barFill');
  const tt     = document.getElementById('timerText');
  const banner = document.getElementById('errorBanner');

  // ── Empty tank error ──
  if (d.empty_error) {
    banner.classList.add('show');
    dot.className     = 'dot err';
    label.textContent = 'Water tank is empty!';
    sub.className     = 's-sub red';
    sub.textContent   = 'Refill the reservoir to use the pump';
    stopTicks();
    resetTimerUI();   // Bug 3: hide+reset bar
    return;
  }
  banner.classList.remove('show');

  // ── Continuous mode ──
  if (d.running && d.continuous) {
    window._pumpLocked = d.locked;
    document.getElementById('btnOn').classList.toggle('btn-locked', d.locked);
    document.getElementById('btnInf').classList.toggle('btn-locked', d.locked);
    dot.className      = 'dot inf';
    label.textContent  = 'Running continuously';
    sub.className      = 's-sub inf';
    const map = {TOUCH:'Touch sensor',PIR:'PIR sensor',WIFI:'App — infinite'};
    sub.textContent    = (map[src]||src)+' · Stop via app or touch';
    const wrap         = document.getElementById('timerWrap');
    wrap.classList.add('active');
    tlabel.textContent = 'Elapsed';
    bar.className      = 'bar-fill inf';
    tt.className       = 't-time inf';
    elapsedStart       = Date.now() - d.elapsed_ms;
    stopTimerTick();
    if (!elapsedTick) {
      elapsedTick = setInterval(() => {
        tt.textContent = fmtMs(Date.now() - elapsedStart);
      }, 500);
    }
    return;
  }

  // ── Timed pump running ──
  if (d.running) {
    window._pumpLocked = d.locked;
    document.getElementById('btnOn').classList.toggle('btn-locked', d.locked);
    document.getElementById('btnInf').classList.toggle('btn-locked', d.locked);
    dot.className      = 'dot on';
    label.textContent  = 'Pump is running';
    sub.className      = 's-sub';
    const map = {TOUCH:'Touch sensor',PIR:'PIR sensor',WIFI:'App control'};
    sub.textContent    = 'Triggered by: '+(map[src]||src);
    const wrap         = document.getElementById('timerWrap');
    wrap.classList.add('active');
    tlabel.textContent = 'Pump running';
    bar.className      = 'bar-fill';
    tt.className       = 't-time';
    lastDurationMs     = d.duration_ms;
    const remain       = Math.max(0, d.duration_ms - d.elapsed_ms);
    pumpEndTime        = Date.now() + remain;
    bar.style.width    = Math.min(100, d.elapsed_ms/d.duration_ms*100)+'%';
    tt.textContent     = fmtMs(remain);
    stopElapsedTick();
    if (!timerTick) {
      timerTick = setInterval(() => {
        const r2 = Math.max(0, pumpEndTime - Date.now());
        bar.style.width = Math.min(100,(lastDurationMs-r2)/lastDurationMs*100)+'%';
        tt.textContent  = fmtMs(r2);
        if (r2<=0) stopTimerTick();
      }, 400);
    }
    return;
  }

  // ── Idle / stopped ──
  window._pumpLocked = false;
  document.getElementById('btnOn').classList.remove('btn-locked');
  document.getElementById('btnInf').classList.remove('btn-locked');
  dot.className     = 'dot off';
  label.textContent = 'Pump is OFF';
  sub.className     = 's-sub';
  sub.textContent   = 'Waiting for trigger…';
  stopTicks();
  resetTimerUI();   // Bug 3: always hide+reset bar when pump is off
}

function fmtMs(ms) {
  const s = Math.ceil(ms/1000), m = Math.floor(s/60);
  return m+':'+String(s%60).padStart(2,'0');
}
function showToast(msg, isErr=false) {
  const t = document.getElementById('toast');
  t.textContent = msg;
  t.className   = 'toast show'+(isErr?' err':'');
  setTimeout(()=>t.classList.remove('show'), 2800);
}

fetchStatus();
setInterval(fetchStatus, 2000);
</script>
</body>
</html>
)rawliteral";

// ── WEB HANDLERS ───────────────────────────────────────────
void handleRoot()   { server.send_P(200, "text/html", INDEX_HTML); }

const char MANIFEST_JSON[] PROGMEM = R"manifest({"name":"PooKooli Fountain","short_name":"PooKooli","start_url":"/","display":"standalone","background_color":"#07090f","theme_color":"#07090f","icons":[{"src":"data:image/png;base64,/9j/4AAQSkZJRgABAQAAAQABAAD/2wBDAAwICQsJCAwLCgsODQwOEh4UEhEREiUbHBYeLCcuLisnKyoxN0Y7MTRCNCorPVM+QkhKTk9OLztWXFVMW0ZNTkv/2wBDAQ0ODhIQEiQUFCRLMisyS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0v/wAARCADAAMADASIAAhEBAxEB/8QAHAAAAgMBAQEBAAAAAAAAAAAABAUAAwYHAgEI/8QAPxAAAgEDAwEGBAMGBAYCAwAAAQIDAAQRBRIhMQYTQVFhcRQigZEyocEHFSNCsdFSYuHwFjNDgqLxJHJTY5L/xAAaAQACAwEBAAAAAAAAAAAAAAACAwABBAUG/8QALBEAAgICAgEEAAYBBQAAAAAAAQIAAxEhEjFBBBMiUQUjYXGRsTJCUsHR8P/aAAwDAQACEQMRAD8A5VUqVKkklSpUqSSVKlMNG0S/1y57iwgMhH43PCIPMnwqSiQNmL6aaP2d1XWjmws5JI84Mp+VB/3HiumdnP2dabpwSbUcX9z1wwxEp9F/m+v2raLEqoqqAqqMBQMAD0FNFf3ENf8A7ZzDTf2XHAbU9QA847Zc/wDk39q0dl2E7P2oGbEzsP5p5GbP0GB+VawxV4kCRKWchVAySfAU1VT6mVrLD2YFaaPp1sB3Gn2kWP8ADAo/SjVjVRhVAHoMV9iZZEV42VkYZDKcg1aBVnUEZMH3xPIYsqzAZI6+lUXGkaddA/EafaS//eBT+lXI8IuSFUB3JBPmR/6/KittRsS1z4mWvewPZ68B/wDgm3Y/zQSMuPocj8qzOp/sqIBbS9RB8o7lcf8Akv8AaunFa+baAqpjBY6+Z+ftY7N6tohzf2UkcecCVfmQ/wDcOKVV+lSmVIIBBGCCODWN7R/s80vUw0tkBp9z/wDrX+G3uvh9PsaA1/Uet4/1TjlSmWuaBqGhT91fwFVb8Eq8o/sf060tpRGI8EEZElSpUqS5KlSpUkkqVKlSSSpUrY9jeyovGS/1FM2/WKE/9T1b/L6ePt1NELnAi7LFrXk0p7Jdi5tZKXV7ugseq44eX/6+Q9ftnw6vp1jb6fbJb2kKwwJ+FFH5nzPqa+Wy8DjAAwBTCILkAkZPQZ61r4CsanOaxrjk9T1F0ohADS3UYL5twtmxGyAYUfOrA5yD4Z6V7t76RZu4u4HRgOG/x+fH9jQlcjIhK2DgxkAGHBB9qU6ssq3sWx5xFLE0bCJN3iCc/Sm0KxRwho9ixAZ+Uce9eoZY51LRtuAJXIpasUOY1kDjEVabY/BidVLd08m6MN1AwP1ott/CR471+Ez4ep9qJmKRRtJIQqKMkmq7BTte6kGHk4VT/IvgP1NWXJ+RlLWAeMTarE9tte3BZoCGQeLbev35+9ObeWO4gjmiOY5FDKfQ0NegMc+VDaM/wtw9kx/hSEyQeniy/r96Y3yTPkRa4D4+40G1phEPxlS2PSg7eedpSs0a7CflZeo9686pb3iXUV7ZZYqhjljHVlzkEeor3pzNdQt31vNBtIxvG0mhAAXlCOS2Ov8AmFMmKHlXijGFBXNxDCjl5ogVBOC4BJ8h60KZJluABuLNStYbuB4LmJJoXHzI4yDXKu1XY+TTN93p+6W0HLIeWiH6j18PHzrrFvdR38DSIrKVOGVuo4zQlzHitJqVxxbuZBc1R5L1OD1K2XbDsuIA+oafHiMczQqOF/zL6eY8Pbpjaw2VtW2DOtVatq8lkqVKlLjZKlSitMsJdSvorWHhpDyx6KPEn2FWAScCUSAMmOOyGgfvS5+JuFzaQtjB/wCo3l7ef+tdStIsYoLTLOGztora3XbHGNqjxPqfUmidN+P1BrtLQCOB8xF5FwQB1x5E8/lXVSsVJj+ZxLLTdZnx4EcwpgDigo9Ou2crcObmJJGeEGTaoBOSD4+NHWyGOJIzI0hVQC7dSaLTpSS5XqNCAxRd2t+ZVeKNYkAwRBOxPvg17F3PAvc36C6tujOv409Tjr/WmU88cGN7YJoe/tA6/EIMNwH2/wAwP9qgfOAwkKkZKncL08x7MxzCeNzz5e/v51dYwR2am1Q9CzjjoCeBSyNfhmTaxRuQUPQmnNtMJ4ww9iPI0i0EfsZopYNryIBfZvLsQIw7uBlMi+LORkD2A5+ooxyUG0jGKQ9mrkTXl3Ox5kmkb88D8hThnmvFMkMiRx5IUMmd2DjPWisQqQp6EpHDgsO5TL82aW38MogM8JAkhcMno3h9D0pqIncsAArLgEsDt+nnQVyHUzQzKodVDKV6MPA+nQ0ys7xE2A4yY2srlLy0huY/wSoGHp6VYGDDKkEdOKT9lHzp00X8sVzIq+x5/Wib66NsVihAUEbQB/L5n9KQ1f5hQTQLR7YcykXpv7yW0XdGiZ3+ZHT6c14n0exf5khYMpyCjkYq6O/t44i5iCStxsjGSQOmTVZuL82sk6W8R28pFzuYeVO+QPx0In4kb2ZPh1hUhFxuOWPix8zQk8WaB7+/vb2G4hHdZdVkhXI2r4lgR5fpTW4khhI76VYlIJ3N5DrTMFDEHD9RVLDXMe2nZ7913PxdqmLOY42j/pt5ex8PtXUILtLvcqqwZQC3yELySOCfahNUs4by1e1uQDFcKRjIycHGR6g/0o7E9xcHuDTYaX5DrzOKVKK1XT5dLv5rSf8AFGeG8GHgR7iha5ZGDgzuAgjIkrc9ibAW1qbp1/jXHC8chP8AU8/QVj9NtDfX0NuOO8bBPkPE/autaFBIkjTwkww26HdIBxGpBA/KtvpKxuw+P7nO9fadVDz3+0e6TJYnuI4kd7kL3hnzhUYr0PsD9/WqbRpBCkcrhypLZA4yepoZXWW2iljhkhtxuWLc+TKfFm+nSroW2DJIGPE1p49mY+egIZLdx2q5l3bipZVVSS2McenXxpjC4eNWXlWAI9qWmeG3RZZZEjVeVZjivq63YoFzK20jhhExB+uKy2Mo7mmpXbYEaGNHkRmUMwzjIq10JjKoQCeORS611zTZm2JeRh/8LnafscUTHqFjPKFjuoZJBnAVwaRyH3NHtsPEGvIpLVhJK/fWrfKSR80frnxFEWE/cTJBKeZOAfM9R+VFsFljZGAZGGCKSSLIkRU/8y1cbT5jqKev5i8TM7fltyWI45W0nWLu2bhRMxU+jHI/I1p9MvY+5EbMBg/KSfM9KA7Q6Z+8ljvbZQ0yryv/AOROuPceFJLKeVSYwS23gqeGHuK18Vvr/XzMZdqLT9GbvePFgPHk44pDqV13s7kY5G0Ef4R40LEJ5T8qhfVzjFMPhYJItiSd6z8SyDoB5CkrWtRydxz2tcuAMT32eU22kmUjmZ2lx79P6CqxmeQuRvx8oHmf/dXXtwLa2Yjqq/Ko+wodoGC2lpGxDMMyEeGf9mqGyWPmWTgBB4nzurhGZrVBcXIbDuR/DRSDwnIyRTSximitEjuZDLIM5Y9SM8Z9auiiSCFYYgFRBhRXvpSHs5ampK+MpkU0s1WyS8jUMBvjbfGxHRqaydKXSrcd7I3eKUH4F24GPU+YNXX3Asi6V4Vu0LwpBcSRBGVHJXgnGM4zn7jGKHkiiiZ2WBTJKwJfJyp5yfr5fWj541Z1kKKZUBCsQRjP+uD9KWuzx4SdkQsP4ZcdVxjPqc5rYkyOSTMl290sXViL2Nf41tw2PGM/2PP1Nc9rslzsljZXAaNwQwPiCOa5JqVobC/ntm57tyAfMeB+2Ky+srwQ48zZ6C7kDWfEfdibTdLcXTD8IEa+55P5D866lo1q02lJbMCIrmcs5A6ouM8+XFYjsrbCDQ4DjDyFpD9TgfkBW902UroXfKyt3aSx7GOAOc/rWgKUoUCZWYWepcnxA7mcXdzIYsiGMfw0P8q8Dj+tWwc4z9KEsESR+6d3R+671FEZO9c4J/I0UXklullM8jJ3YUxt0yOhH08sdTTWwPiIpd/JvMs1LTrLVLRodQgWaMfMM5BU+YI5Brm2pzzKy21rFhYRsLHIyPIA9K6esvdxs54Cgk846VxrU9Sn1CeRgzLEzsdqnAGTmuZ6oDInX9CTgwiS+bu0inkMkanIjbkZ8fY+1UfvZlVUhUqq9PMHzph2Z7MS61NtaQKM/MDkEZ6H1Hr7VupuxFtaRkRxZcxH/uYc1znsVddzqKhPnETdktdv7dooMkwyEkCQn8RHH0zRV12zdZ5IyIllGUkkPIYjwArLtqDaZdrvXc9qzNjkYPP+lKI9029yflRS7sT1Y801GYdGLdEbsZm7sv2gSoBvtomhj/EA+049KK/4v03Utzz2UPy9G747m9AQv9TXN5LV1h3x5O8AkjjFeLZ5Vwu4x9ecnmmC112DEt6epxgrOwaZfaPdd2UjdO8OIzPko58lbJBPp1ovVNah00MmxmdBzwFRR6kkflXOdDtF1iJIrRE/eBfYTnKbccsw9B49cgY8a18j97o4g1Axy3FqSjyE9cEjOT1PrRm+xl3FL6SlG1/EDuu0st4xWNY0PBB27gfIdTQD9rb6CbKz75M7d23PND3jrKSLdQkIGFRF5PFAx2pWeSTZ/wAlgrNjhc/7/pQB2PZj/aRegJqP+Mr5JUSR497DG3bxnPApRf8AavUbq5ZPiGj5HHQUocd7cO4JGzjp0xmq3ysqxyqC7DcB4nONo+/FTJl8QPE0dh2kvolLG+KQqOe9XcB69f8A3Tq27U9+mfiYZCxwO8t2iAPllSf6VjJrkQtHAqq8KfjUj8fmab6BaWi3CtHLsjnG9VzkZHNByKnuEUVhsTR2WsrfyyRfDzRunVtpaNgRkENjHNEwQw3FwyTsQu3K4bByPAe+Tx416OO74xySTjxPnQ2cTxtgnawOF68V2KQTXknc89eVW4gDWYDfK8bMZpY5GYkho+PHjIwMH0rB9uLJo57a72kd6pQ5GOnQ/UH8q6U94Y7qOG3MHxJb5nkj/iw+Hsfasl27t+90+4HetMbZwQ54zg7Tj05o7cvUVx1uDTiu8Pns4/mHafD3VrBCo/AioB9AK0mm9zpclxZX022S4X5o1RmKYB544ORg5HTxpHEF7kMG+bdjb5Dzplpli8pF8A0y2yOsimQl2G35Sc9fEfQU24fHHiK9Pts+YNbLbvLLdWiywIXaNUJyWX/N489aYQUKYzAiqwwXZnAZcHacYOfaiITUOxKHc+a3CJ9MaJtxWXjAON2Pbn6CsR+5ILaZ1mZRAzE7x8oGPL/WmOt608V3P3d4u1CAISCMjxIIoW3wlw01zH8ZaP8APJsXdsXx+1ee9Q7M5M9R6atUrAjrT+1+jaPDHb6dYX10CwXcNoDOfAc8mtPpPa/StdQSQM6vCGMkUq4eMdMkeXt51hdW0IarYAaZdw8TF4334EisOjeKsOeo5r5AkfZKSO6uS0rxWrxMwYbpnfAAUHnaAOWxjoOtZQ6t8R3n+o8rvJ6iztqbWTVpTY7CsyYBHTJPJ+gFLUhEYGnqrGedwvHJ68/kKpkjluLx7kDYqkFIx0Az4D/fWmNjpsFzPZ91cM0twZUkxn5XAyBu9QT9jTyRWmT4gj5tqaBezc2qBktGi7mHgIOXfFZnUtIkW4MKoQ4HOfDFeNGs5JbyUR28lrJaoxklVyChA4PvnNaxHvGWT94vKZVTBeSLLE46A+NA1gU4G4SpzGTqK+wmorps+ozPCpeGAD8OMPuAAH50TZtPdDY0jusrlio5ySf6eFfb6C2fs8dRhUW8yuEdQfxL5/nVPZh7q5vA1jEVMaO7SuvBAUnAHieKfy5KBE8QrExzpVndRa5aC5tmNscrJgZ2epPvQ9pYloe1FvOu2SO5Cpz+LHK4/L70JJb3G7RZpNYv3vL8mRpUkYLEAm7gDjqfsDTeeCfUNMuXd8X1tdrFMUGBNuVSCR54IyPDBpa2KxwIRUjZmQgUNcyhmAEkqhvHAyCTT21t/i/2gW6BSLeGHvWAH8g5H54pVe2zQ3twqjacZUYyetP1ElithZWswhvbu1+Iubtly0UeeFX1zRswUZMoLy0INe6de3V7NItmUtWc7FaPadvn/wC6TajbyKJIoJMeQB5UDxoi2a+OlT6lb6neLcQyHcjuzbwGAwQeM8+HjVV98RYahKupxsoZ/lkQYVj9uPahDhiceIXHjNT2YvjcaWkMpPxEWd/+bnqKOkDFWOScHwUcA+Z96zWg3gXUo0UBVcFcCtOVLNjBJcbRgZJ56D6jwrtekflX+08966vhbrzBpGtwLlmHeXEwVVcjOzI+b7ZFL9Zto5NOlhTAZ4GVlAOFOMDk9c4z9aYXYuVcW93YLalkEhIk3Eg/KBjw4Bz60MqM1tCZPxGMA/Qlf0rWoDDvuYnypxjY/wC8z3YuvcOUhiabIYPLLtwviVHmOvjkcYqyyvDBIxaTbHMDHIc9AfH6H9aVaPdFrS0nUgP3aOCRnkAfrWkgt01iOQRIqM+FkaVAWOSOQ2QM+oGaFyAMnow6wSeK9ifIbWYLJahQk8MpYK3SQEdQx/pUP/OY5BQ4AATbtwMEdefei49WcC3tZopI5IpAJG7wglBkZOBnpzSq7kW3eZY5O9RCQjAYyKVyIyX1He2CQqbzOda3LJbX1x86tKJGU5PGM1LLXXXlhKjYwRGQAw+tW9pCUvpHP4ZMFt5HLfSk8MKt80ZAIPVCxxXFIDT0QJWM9S1uV0LY7iU/gUZJHqD4UDLFeX7SXTuzlQWcp820euKfwX+lwmNXtYu7XHeBZWDk+OX2HPsOBWh0PWOzFrMHt0NvLIcM7XTExjw2/L1P69aADiNCESWOzMZBdRW+hP8Aw2LyuNtwW46HKkeGT/Sr+zWvnSLqZbhRJa3AXcJFJTg8MB/Q9RiiO2MlnD2lmvbSzQ20wXadvyb8fMy+HXz9aGtzBqm43UyQN4hyVLA9dpwV8uDV6Ik2DOm6XfWupIrFLYuvJ5OGI8cZ5pV2k1JCgldmdIyQ2xAFQeYyeT9KQaxpcnZ22guLaWa4tyf4EmOmecEj5fqD9BSO6v7q9KvOqKRyWwMGsqenCnXUcbNfrCrqYXVgbZX3iQ8LnG0g+I9qNsLi/sHie0/htDgphcg0lieWaUPH/DO7OF54HiK2ujTwXFmlvMsbuPwl9yH2NahoRB2ZTba0kRHf2jQxhtzLBIpSMk5O1XVivtnHoKrtdWNtf3UViGNrcS94nenLF+hYk9Tg1fqmh3iEi2tXnPA/hp8q55wB40107srLqOmY7yTT7mMnYZIgctg5JGfM9fSqVEBJA2ZCxxgzN69eRC5M4GZUUAt4HHNJ9L1AahrkNxf3M8CxQmONoFBYYzgYPDDJ5FMtO7P6nrWqzaT3rQrZMRdyuCUXyAB67uSPTmre0HZZtJmRLGOe5aMbklABz0zkDkfarwCMGX0dGXNq1pA6lIprt1YSKsqJHGGHRiqZ3Eepx6Ulvru61QzJOu8udwI5w2aLstJub6TeLd4yozKkmU2+x8KOlshYrG1o6M4ILAkux9M4oQFQYUYlnLbME7M2csGrxpcDDopbk+GP9a39vEsQhulkzKVYxqBn7euM9eOazehRSW+pS39yO9Yrzx4npx44pozT30uRIu0MT3YQthQMe2OcfWur6Qcq8Ti+uPG0N+mp9uLq3mUT3c8jtEQmxIy5l6nAb8vuaBZGh394EVslyqNuUZ5wPSn19aSWFnbSBU3Ird58q4UnGBg+h8OfpWb1WT4fS7uXpsgcj7cVsRhgsOphsQ5CnsxL2HmW4sIojEs0ilokEj7V3ZyM/fz61utOsGNrNZXDNDcSDMceTtU9eD7+Ncq7EXjQ3skCsVZsSRkeDKfD6H8q6giRxX1pLctlJ9zs4b/mFl5IGfOkKxepd/8AhNLIEubI8/3E+n3klxPI853Mx5LeIHH6V71Qs9uXDKuDjGec+AoCVvgZLh2GO7dk49+Kv0+5jlPdyIZC3Vi2MemPGk/iFgACLNH4ZUSTY0yN/BLcTOCHdSMtkdD/AE/rSK6U2shRjkjoCD+v9q7UdFtrix225aOQDK4jJKn1HWsHquhSfEyAQmcfzd2RlfdDz+dcb3+LYPU7ftchruYvvZA24lufAHrRtlZNJcJ3jfjIGTztU8Fj7ZqX1p3DYt1bHirAg/QGjdF1a2t7qJriLaQcOSucjwFP5ZGRE8cHBnQ7/R7DVOzz29tsd4UKLIo4JX1Nc7gsJLV3RwRLEcHa+A2fI1u7PtDZzJ3VvKiD/CV25+lLruGxmuhcPs4Pytu4pAbjqOILbmWvhP8AJFIyiNiWQBQFHuR196ommz8ojVSPxEePFaDV1tLlAiOdo5wgyaUPbAmNFfeXO1V8R6n1o1cHUBkI3GvZqwMwErjk9M+Vb23tHaFQn4x04A/p/el3Z/Tu7jQbfAVs7K1VVHGKaRkRYOIuSCXH8ZFf/tBH55q4zfCxFbW0bcevdqqj702e2UjgChJbWaMZhwcfyk8GqGu5CQZndPk1Kyv7y4miDC6ZSyxnlCBtHXrxVuoQx35WSSCFj5yQrkflmjma5kdk7naTjkiiINOlYA3D7m9qrOeoWMbMV29ugiZI9y7upUgfTpj7ivdvpECtvZA7nxZAD9xin8dnGn8oz7V7aNFHCioE+5Rf6iOXTYjbyRhMZBNZ/RtUUXRsJz3RJIJA+dgBwoPhk1rrp1CnJwMVyTVWxqtwi5xvO3PWtFdhTqZ7KhZ3Nzd3STRMzOttA+Zooy+9mYYU9OetZTtndiLQJQCc3DKikjGRnJP/AI/nTXQ7z4Wzt7mY5mDloUIBBA4JPp5YrI/tDvmmv4bYvuMamRyOhLdPyA+9dB3xUSOjOWlWbgG7EzWnXklhfQ3UTFXjbOQAePEc+ma63Z3TLLDHOqmO4iSW2kROCNuTjOcAgk4HQ/WuOVsuy+rR3GmxWFw1x8TaTB7aRDnu06nA9D9Oay0PxJBm6+vlgjxCtfuzPftEikq3Iw2STjrmjuzctjZzAaq+1cZVckgnwobXrexktTeaYDFLESXhfcXHmSTwSTkjHhSiaQ6hGkMYVGRfxMeBWS4MWPLzNlJUKAviarUNWmsNUmuLKMSxn5l+fcR9qDuu1kurOIZbYQMDkiVVDH2Jwc1lRFdWzLFGw77JOQwIb2NERavcFxBeMQ+MF3IbA8MeZrMaRiaRZuWXsPfTFd5kDN8oVuT48Hpn0r1ZaY0mWhkid8cLIuWVvXxOKG1G/wBhxGMxFiCVYYf+2KoTtFIqESQRSMDkNjBFDwsx8YXNAflNKtlHEyvM294hgkpgewPFU3cttKwVLchj/nXH5Gpaa/aT2pSc4LDBEgzg+ftWP1CZWvneDaoB47vOPpVJWzaMp7FXYj6XUY7VztKbj0VPmP1NXdnXk1LXEMqnbGpYDOR5Vk0Yh92SD1zW2/Z8xuNRmO0cIBx41pSsJENYWE6jplsoVT6eVOoU2jFBWCHYMjFMUGBTYmfcV5ZcjGTXqvh54xmqMglYjAI2qBjwr3wK+cMWHhXkttJLH5fboaHqF3PZJxwKRdodQ+GtjiXYenHH502nlVELM+1cVzntNqBubgoJNsRyFYYOR55qEwlEU3esXEjuvevKfDc2KUwqb/WLaBmfdK6q5BBIHj4+VXPbqRnvJCF6A8AD14ph2Z7mOe+NyCAWTDIo3EYxgMeVHJ6deKOtcnEGxsDIni9ubfTNcmAVZII8xYL/AIQByQcHpz+dYXULt769muZPxSNn2HgPtTLV5DbwCDayvJyc5/Dng/X9KTUZZsYMXxXPISVdZ3L2lzHPHyUYHBJww8QceB6VTUoZc6bo80Wp2UrbmS2lhCtuIUEqw25OOCpLDI8DSDULeDTJZoDIySxMRvDcFfAjHUUr7Ma0NMuGguixsLn5ZgBkp4bl9R+Y+ldAvtHsLmxtL6C53qU/iIG2lxnIIOOh6fXNaGHvJruIVvZff+M5xILeTn4huPw5Jz+de7G0nuJUjhga7djtRVUlvYCmvabS7W1vSkasxaMSNsJkVAecBsc8Y8Ku7NRJafGs0nw7CEGKdn292wIY4bzxx7sKzBDy4maS448xF+o6Re2USmSORWYkmKRfmBxnofSlUsJCpcZDo55x0H9q0Wu62bm6gFrM9xBaxLEHlQLI4J5zj7Um75ll4AG5eSo4cA85H9aWw4k4jFPJRnuU3QePu3RvlXgHxX0oAnJyaO1Fo2dYoCQo42kdPT2/vVml6ety7CZLjBiZkMUe/wCYdM+nnRoIDmBR4bChOfAjrW7/AGaHbeXIfgkKT61iO8kgYofkGemOlaf9ndx3etMpYfxFOB1JqDuQ9TtlqSUGOBRYKgdfzoCzl3RqR81Fd7sG5kwB4g5qQZcSB1wM14dzt+TB60F+8oW7zCshA43r1HnVLXMcqLJC28YypyQCPEehFCTDCw1HbvSM5GABXm7nWC3aVzhEG7mq4cAsVHReKC1qTNi/gioSCenpVdCX5mR1e/1HU7oB1b4RWI2Jxj1z40oumtosiWQxh/mMZJ3N9KoudcnJ7zu544O82m5f5gAD4qOo9aCvLvuHX4O4+JmkPOJAyHPkOSPaqAJhkgQ+O1S6jZ4pLaIBgFEqt3khOeQAPSvmq6lb6fbxRWzSL3K5n3dZCfDHgc449ATRem3lxp1tOLq2he8gidR8g2xBuSSceHmelYPV9Tl1CQhnzGHZ+OAzHq2PsPYVsAFYB8zGSbCQeoJc3D3U7SyHLN+Q8B9BVVSpSYySpUqVJJK0fZTtIulk2t6He0ZtyMp+aF/Bh5jzH1HrnKlWrFTkSmUMMGdVu9RjvWDajbw3CTx7YpVbaGx0LAA5OOnmPak+rM//AAmLKS3htoYZTiVYj/EAwcA+ByQT6Y8BWS0zVGtGWOdO/tc/NET056r5VrrfWgtnLb2UvxFnIQRvX50I6ZB8RyPUVq5K4mcKyGYYqyDkMoPOD0IpmkTSWvfCNmCAlmHQ/XzFP9USG8S2Sym7yCMOTbtGFeHJ5GPH3HHXpR2iwRx6REr3BhhiJYxFt0c7NxggcZKnB5yOOKzikscR5uCDM5+O9uJwqK0kj8AKMlvYCtN2S1SDTbe+t2vJLG7uCsYlC5XZ/Op8VPQg+lJQ8ukX5kjEXfxllZMMdmQQevv1HNU3d29/dyXMu1XkbOFzgDwAzk4GBUHw35kPz1NB20+Dm1XdaTR3MndorTqD/HIH4uc8+H0qnsnHLFrUP8Fiz/gVSOT59Ofaq9DtGvpY7dNzyMwCxsuN/OSc+GOvpWnSSPszKktpOtzqWWWRjkxxoQOV6Z/X26jtmzC0q4nQEjkD2r2s6SxqxSds4Yjk9MefH60e2GXDS8ddvnSnSpC0EbswYuu4k8E+p8qagpjIxnzqOcyIMQK9Eyr3wlUlfmcbf5T/AC15TIlL7V2ygFWQ/K/+tGPFHnIA/vQqptRAiHAz8o8SKTjcbnULMqogUn8fU+g8KSaxci8gBE8RtX3KyE7S7A4C5PgeftXq70qa9uVN7P3cYOREPxOMc4Hh5ZPFUanNdNFLYQo9wzsTIVj3HGeFzjwHj409EGMtEO5zhZndUtreOIytfWyMw+VRyUHjgdPyoKzitbDdNpyd08cZZry4AXaMZO0YxnyJ86I1TS9N0e1+M1GVIgp+WCHDSSn/AA56e+M4rD61r9zqx7sgQ2wYssKnP3Pif94o8KniDlm8y7W9dNzF8DZs4swdzM3DSt6+gPQfU+iSpUpZJJyYYAHUlSpUqpclSpUqSSVKlSpJJXuCaS3kDxOUYeIrxUqSTT6brmmXKiPV7d4pg2Uu7fqPdfetENYNlA0IWO/tHYHvLcgNkHIZT44JJwfyrm1e4ZpYG3RSMh9DTVt+4o1/U2/caZJIs6vDeBB8vfja581bOQcZ6k+1fdM0LvrB4QsRjkuBJvg2NLGBjIX5ufIZOBz1rO2HaS6tWJeOOYNw2VALf79qdDtHotxGFkt7m3kH86txn6dftWkPW3czlLF6mh06P9yJexwaZKGkTbHJcIGdmPRCQcY4zgdcVnfgp9QeVRIZJVG9pHbaWJ6jB6nPh6Hyo627RpDC0MWoQ3ELkFknYjP3AIPqCKvXXbYkywOkMhAGyG4GGHPJJyc8nnPNC1aNjBhLY65yJpOxtwz6XF3qlGB24bOSRWillkVInC53SbCoxwCeCeeOAaw1jd20J7u2uLC3jAOJLq9UFTjghRnofTmmM3aHs7DZhJtbDzEIZO5JaPcM8gbTnOeeOaD2lHZhm1m6EfatqY02DvIkM0rnbFGATub3HSvnLW8QmvljuVGHMcZQnP4snqCfSsbdftD0iGR2giurmZBsSUIsalfT/CPZRWa1Pt9qN2jx2kcdkrdXQlpP/wCj0+gzQgog13IQ7nfU6beaxFpEDOoBAAXvXHcxj03N8zH2rD61+0NyrRWR74/4tuyEey9W92+xrAyyyTOXlkaR2OSzHJP1rzQGz6jBX9y++vbnULgz3kzzSnjcx6DyHkPQVRUqUuMkqVKlSSSpUqVJJ//Z","sizes":"192x192","type":"image/png"},{"src":"data:image/png;base64,/9j/4AAQSkZJRgABAQAAAQABAAD/2wBDAAwICQsJCAwLCgsODQwOEh4UEhEREiUbHBYeLCcuLisnKyoxN0Y7MTRCNCorPVM+QkhKTk9OLztWXFVMW0ZNTkv/2wBDAQ0ODhIQEiQUFCRLMisyS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0v/wAARCADAAMADASIAAhEBAxEB/8QAHAAAAgMBAQEBAAAAAAAAAAAABAUAAwYHAgEI/8QAPxAAAgEDAwEGBAMGBAYCAwAAAQIDAAQRBRIhMQYTQVFhcRQigZEyocEHFSNCsdFSYuHwFjNDgqLxJHJTY5L/xAAaAQACAwEBAAAAAAAAAAAAAAACAwABBAUG/8QALBEAAgICAgEEAAYBBQAAAAAAAQIAAxEhEjFBBBMiUQUjYXGRsTJCUsHR8P/aAAwDAQACEQMRAD8A5VUqVKkklSpUqSSVKlMNG0S/1y57iwgMhH43PCIPMnwqSiQNmL6aaP2d1XWjmws5JI84Mp+VB/3HiumdnP2dabpwSbUcX9z1wwxEp9F/m+v2raLEqoqqAqqMBQMAD0FNFf3ENf8A7ZzDTf2XHAbU9QA847Zc/wDk39q0dl2E7P2oGbEzsP5p5GbP0GB+VawxV4kCRKWchVAySfAU1VT6mVrLD2YFaaPp1sB3Gn2kWP8ADAo/SjVjVRhVAHoMV9iZZEV42VkYZDKcg1aBVnUEZMH3xPIYsqzAZI6+lUXGkaddA/EafaS//eBT+lXI8IuSFUB3JBPmR/6/KittRsS1z4mWvewPZ68B/wDgm3Y/zQSMuPocj8qzOp/sqIBbS9RB8o7lcf8Akv8AaunFa+baAqpjBY6+Z+ftY7N6tohzf2UkcecCVfmQ/wDcOKVV+lSmVIIBBGCCODWN7R/s80vUw0tkBp9z/wDrX+G3uvh9PsaA1/Uet4/1TjlSmWuaBqGhT91fwFVb8Eq8o/sf060tpRGI8EEZElSpUqS5KlSpUkkqVKlSSSpUrY9jeyovGS/1FM2/WKE/9T1b/L6ePt1NELnAi7LFrXk0p7Jdi5tZKXV7ugseq44eX/6+Q9ftnw6vp1jb6fbJb2kKwwJ+FFH5nzPqa+Wy8DjAAwBTCILkAkZPQZ61r4CsanOaxrjk9T1F0ohADS3UYL5twtmxGyAYUfOrA5yD4Z6V7t76RZu4u4HRgOG/x+fH9jQlcjIhK2DgxkAGHBB9qU6ssq3sWx5xFLE0bCJN3iCc/Sm0KxRwho9ixAZ+Uce9eoZY51LRtuAJXIpasUOY1kDjEVabY/BidVLd08m6MN1AwP1ott/CR471+Ez4ep9qJmKRRtJIQqKMkmq7BTte6kGHk4VT/IvgP1NWXJ+RlLWAeMTarE9tte3BZoCGQeLbev35+9ObeWO4gjmiOY5FDKfQ0NegMc+VDaM/wtw9kx/hSEyQeniy/r96Y3yTPkRa4D4+40G1phEPxlS2PSg7eedpSs0a7CflZeo9686pb3iXUV7ZZYqhjljHVlzkEeor3pzNdQt31vNBtIxvG0mhAAXlCOS2Ov8AmFMmKHlXijGFBXNxDCjl5ogVBOC4BJ8h60KZJluABuLNStYbuB4LmJJoXHzI4yDXKu1XY+TTN93p+6W0HLIeWiH6j18PHzrrFvdR38DSIrKVOGVuo4zQlzHitJqVxxbuZBc1R5L1OD1K2XbDsuIA+oafHiMczQqOF/zL6eY8Pbpjaw2VtW2DOtVatq8lkqVKlLjZKlSitMsJdSvorWHhpDyx6KPEn2FWAScCUSAMmOOyGgfvS5+JuFzaQtjB/wCo3l7ef+tdStIsYoLTLOGztora3XbHGNqjxPqfUmidN+P1BrtLQCOB8xF5FwQB1x5E8/lXVSsVJj+ZxLLTdZnx4EcwpgDigo9Ou2crcObmJJGeEGTaoBOSD4+NHWyGOJIzI0hVQC7dSaLTpSS5XqNCAxRd2t+ZVeKNYkAwRBOxPvg17F3PAvc36C6tujOv409Tjr/WmU88cGN7YJoe/tA6/EIMNwH2/wAwP9qgfOAwkKkZKncL08x7MxzCeNzz5e/v51dYwR2am1Q9CzjjoCeBSyNfhmTaxRuQUPQmnNtMJ4ww9iPI0i0EfsZopYNryIBfZvLsQIw7uBlMi+LORkD2A5+ooxyUG0jGKQ9mrkTXl3Ox5kmkb88D8hThnmvFMkMiRx5IUMmd2DjPWisQqQp6EpHDgsO5TL82aW38MogM8JAkhcMno3h9D0pqIncsAArLgEsDt+nnQVyHUzQzKodVDKV6MPA+nQ0ys7xE2A4yY2srlLy0huY/wSoGHp6VYGDDKkEdOKT9lHzp00X8sVzIq+x5/Wib66NsVihAUEbQB/L5n9KQ1f5hQTQLR7YcykXpv7yW0XdGiZ3+ZHT6c14n0exf5khYMpyCjkYq6O/t44i5iCStxsjGSQOmTVZuL82sk6W8R28pFzuYeVO+QPx0In4kb2ZPh1hUhFxuOWPix8zQk8WaB7+/vb2G4hHdZdVkhXI2r4lgR5fpTW4khhI76VYlIJ3N5DrTMFDEHD9RVLDXMe2nZ7913PxdqmLOY42j/pt5ex8PtXUILtLvcqqwZQC3yELySOCfahNUs4by1e1uQDFcKRjIycHGR6g/0o7E9xcHuDTYaX5DrzOKVKK1XT5dLv5rSf8AFGeG8GHgR7iha5ZGDgzuAgjIkrc9ibAW1qbp1/jXHC8chP8AU8/QVj9NtDfX0NuOO8bBPkPE/autaFBIkjTwkww26HdIBxGpBA/KtvpKxuw+P7nO9fadVDz3+0e6TJYnuI4kd7kL3hnzhUYr0PsD9/WqbRpBCkcrhypLZA4yepoZXWW2iljhkhtxuWLc+TKfFm+nSroW2DJIGPE1p49mY+egIZLdx2q5l3bipZVVSS2McenXxpjC4eNWXlWAI9qWmeG3RZZZEjVeVZjivq63YoFzK20jhhExB+uKy2Mo7mmpXbYEaGNHkRmUMwzjIq10JjKoQCeORS611zTZm2JeRh/8LnafscUTHqFjPKFjuoZJBnAVwaRyH3NHtsPEGvIpLVhJK/fWrfKSR80frnxFEWE/cTJBKeZOAfM9R+VFsFljZGAZGGCKSSLIkRU/8y1cbT5jqKev5i8TM7fltyWI45W0nWLu2bhRMxU+jHI/I1p9MvY+5EbMBg/KSfM9KA7Q6Z+8ljvbZQ0yryv/AOROuPceFJLKeVSYwS23gqeGHuK18Vvr/XzMZdqLT9GbvePFgPHk44pDqV13s7kY5G0Ef4R40LEJ5T8qhfVzjFMPhYJItiSd6z8SyDoB5CkrWtRydxz2tcuAMT32eU22kmUjmZ2lx79P6CqxmeQuRvx8oHmf/dXXtwLa2Yjqq/Ko+wodoGC2lpGxDMMyEeGf9mqGyWPmWTgBB4nzurhGZrVBcXIbDuR/DRSDwnIyRTSximitEjuZDLIM5Y9SM8Z9auiiSCFYYgFRBhRXvpSHs5ampK+MpkU0s1WyS8jUMBvjbfGxHRqaydKXSrcd7I3eKUH4F24GPU+YNXX3Asi6V4Vu0LwpBcSRBGVHJXgnGM4zn7jGKHkiiiZ2WBTJKwJfJyp5yfr5fWj541Z1kKKZUBCsQRjP+uD9KWuzx4SdkQsP4ZcdVxjPqc5rYkyOSTMl290sXViL2Nf41tw2PGM/2PP1Nc9rslzsljZXAaNwQwPiCOa5JqVobC/ntm57tyAfMeB+2Ky+srwQ48zZ6C7kDWfEfdibTdLcXTD8IEa+55P5D866lo1q02lJbMCIrmcs5A6ouM8+XFYjsrbCDQ4DjDyFpD9TgfkBW902UroXfKyt3aSx7GOAOc/rWgKUoUCZWYWepcnxA7mcXdzIYsiGMfw0P8q8Dj+tWwc4z9KEsESR+6d3R+671FEZO9c4J/I0UXklullM8jJ3YUxt0yOhH08sdTTWwPiIpd/JvMs1LTrLVLRodQgWaMfMM5BU+YI5Brm2pzzKy21rFhYRsLHIyPIA9K6esvdxs54Cgk846VxrU9Sn1CeRgzLEzsdqnAGTmuZ6oDInX9CTgwiS+bu0inkMkanIjbkZ8fY+1UfvZlVUhUqq9PMHzph2Z7MS61NtaQKM/MDkEZ6H1Hr7VupuxFtaRkRxZcxH/uYc1znsVddzqKhPnETdktdv7dooMkwyEkCQn8RHH0zRV12zdZ5IyIllGUkkPIYjwArLtqDaZdrvXc9qzNjkYPP+lKI9029yflRS7sT1Y801GYdGLdEbsZm7sv2gSoBvtomhj/EA+049KK/4v03Utzz2UPy9G747m9AQv9TXN5LV1h3x5O8AkjjFeLZ5Vwu4x9ecnmmC112DEt6epxgrOwaZfaPdd2UjdO8OIzPko58lbJBPp1ovVNah00MmxmdBzwFRR6kkflXOdDtF1iJIrRE/eBfYTnKbccsw9B49cgY8a18j97o4g1Axy3FqSjyE9cEjOT1PrRm+xl3FL6SlG1/EDuu0st4xWNY0PBB27gfIdTQD9rb6CbKz75M7d23PND3jrKSLdQkIGFRF5PFAx2pWeSTZ/wAlgrNjhc/7/pQB2PZj/aRegJqP+Mr5JUSR497DG3bxnPApRf8AavUbq5ZPiGj5HHQUocd7cO4JGzjp0xmq3ysqxyqC7DcB4nONo+/FTJl8QPE0dh2kvolLG+KQqOe9XcB69f8A3Tq27U9+mfiYZCxwO8t2iAPllSf6VjJrkQtHAqq8KfjUj8fmab6BaWi3CtHLsjnG9VzkZHNByKnuEUVhsTR2WsrfyyRfDzRunVtpaNgRkENjHNEwQw3FwyTsQu3K4bByPAe+Tx416OO74xySTjxPnQ2cTxtgnawOF68V2KQTXknc89eVW4gDWYDfK8bMZpY5GYkho+PHjIwMH0rB9uLJo57a72kd6pQ5GOnQ/UH8q6U94Y7qOG3MHxJb5nkj/iw+Hsfasl27t+90+4HetMbZwQ54zg7Tj05o7cvUVx1uDTiu8Pns4/mHafD3VrBCo/AioB9AK0mm9zpclxZX022S4X5o1RmKYB544ORg5HTxpHEF7kMG+bdjb5Dzplpli8pF8A0y2yOsimQl2G35Sc9fEfQU24fHHiK9Pts+YNbLbvLLdWiywIXaNUJyWX/N489aYQUKYzAiqwwXZnAZcHacYOfaiITUOxKHc+a3CJ9MaJtxWXjAON2Pbn6CsR+5ILaZ1mZRAzE7x8oGPL/WmOt608V3P3d4u1CAISCMjxIIoW3wlw01zH8ZaP8APJsXdsXx+1ee9Q7M5M9R6atUrAjrT+1+jaPDHb6dYX10CwXcNoDOfAc8mtPpPa/StdQSQM6vCGMkUq4eMdMkeXt51hdW0IarYAaZdw8TF4334EisOjeKsOeo5r5AkfZKSO6uS0rxWrxMwYbpnfAAUHnaAOWxjoOtZQ6t8R3n+o8rvJ6iztqbWTVpTY7CsyYBHTJPJ+gFLUhEYGnqrGedwvHJ68/kKpkjluLx7kDYqkFIx0Az4D/fWmNjpsFzPZ91cM0twZUkxn5XAyBu9QT9jTyRWmT4gj5tqaBezc2qBktGi7mHgIOXfFZnUtIkW4MKoQ4HOfDFeNGs5JbyUR28lrJaoxklVyChA4PvnNaxHvGWT94vKZVTBeSLLE46A+NA1gU4G4SpzGTqK+wmorps+ozPCpeGAD8OMPuAAH50TZtPdDY0jusrlio5ySf6eFfb6C2fs8dRhUW8yuEdQfxL5/nVPZh7q5vA1jEVMaO7SuvBAUnAHieKfy5KBE8QrExzpVndRa5aC5tmNscrJgZ2epPvQ9pYloe1FvOu2SO5Cpz+LHK4/L70JJb3G7RZpNYv3vL8mRpUkYLEAm7gDjqfsDTeeCfUNMuXd8X1tdrFMUGBNuVSCR54IyPDBpa2KxwIRUjZmQgUNcyhmAEkqhvHAyCTT21t/i/2gW6BSLeGHvWAH8g5H54pVe2zQ3twqjacZUYyetP1ElithZWswhvbu1+Iubtly0UeeFX1zRswUZMoLy0INe6de3V7NItmUtWc7FaPadvn/wC6TajbyKJIoJMeQB5UDxoi2a+OlT6lb6neLcQyHcjuzbwGAwQeM8+HjVV98RYahKupxsoZ/lkQYVj9uPahDhiceIXHjNT2YvjcaWkMpPxEWd/+bnqKOkDFWOScHwUcA+Z96zWg3gXUo0UBVcFcCtOVLNjBJcbRgZJ56D6jwrtekflX+08966vhbrzBpGtwLlmHeXEwVVcjOzI+b7ZFL9Zto5NOlhTAZ4GVlAOFOMDk9c4z9aYXYuVcW93YLalkEhIk3Eg/KBjw4Bz60MqM1tCZPxGMA/Qlf0rWoDDvuYnypxjY/wC8z3YuvcOUhiabIYPLLtwviVHmOvjkcYqyyvDBIxaTbHMDHIc9AfH6H9aVaPdFrS0nUgP3aOCRnkAfrWkgt01iOQRIqM+FkaVAWOSOQ2QM+oGaFyAMnow6wSeK9ifIbWYLJahQk8MpYK3SQEdQx/pUP/OY5BQ4AATbtwMEdefei49WcC3tZopI5IpAJG7wglBkZOBnpzSq7kW3eZY5O9RCQjAYyKVyIyX1He2CQqbzOda3LJbX1x86tKJGU5PGM1LLXXXlhKjYwRGQAw+tW9pCUvpHP4ZMFt5HLfSk8MKt80ZAIPVCxxXFIDT0QJWM9S1uV0LY7iU/gUZJHqD4UDLFeX7SXTuzlQWcp820euKfwX+lwmNXtYu7XHeBZWDk+OX2HPsOBWh0PWOzFrMHt0NvLIcM7XTExjw2/L1P69aADiNCESWOzMZBdRW+hP8Aw2LyuNtwW46HKkeGT/Sr+zWvnSLqZbhRJa3AXcJFJTg8MB/Q9RiiO2MlnD2lmvbSzQ20wXadvyb8fMy+HXz9aGtzBqm43UyQN4hyVLA9dpwV8uDV6Ik2DOm6XfWupIrFLYuvJ5OGI8cZ5pV2k1JCgldmdIyQ2xAFQeYyeT9KQaxpcnZ22guLaWa4tyf4EmOmecEj5fqD9BSO6v7q9KvOqKRyWwMGsqenCnXUcbNfrCrqYXVgbZX3iQ8LnG0g+I9qNsLi/sHie0/htDgphcg0lieWaUPH/DO7OF54HiK2ujTwXFmlvMsbuPwl9yH2NahoRB2ZTba0kRHf2jQxhtzLBIpSMk5O1XVivtnHoKrtdWNtf3UViGNrcS94nenLF+hYk9Tg1fqmh3iEi2tXnPA/hp8q55wB40107srLqOmY7yTT7mMnYZIgctg5JGfM9fSqVEBJA2ZCxxgzN69eRC5M4GZUUAt4HHNJ9L1AahrkNxf3M8CxQmONoFBYYzgYPDDJ5FMtO7P6nrWqzaT3rQrZMRdyuCUXyAB67uSPTmre0HZZtJmRLGOe5aMbklABz0zkDkfarwCMGX0dGXNq1pA6lIprt1YSKsqJHGGHRiqZ3Eepx6Ulvru61QzJOu8udwI5w2aLstJub6TeLd4yozKkmU2+x8KOlshYrG1o6M4ILAkux9M4oQFQYUYlnLbME7M2csGrxpcDDopbk+GP9a39vEsQhulkzKVYxqBn7euM9eOazehRSW+pS39yO9Yrzx4npx44pozT30uRIu0MT3YQthQMe2OcfWur6Qcq8Ti+uPG0N+mp9uLq3mUT3c8jtEQmxIy5l6nAb8vuaBZGh394EVslyqNuUZ5wPSn19aSWFnbSBU3Ird58q4UnGBg+h8OfpWb1WT4fS7uXpsgcj7cVsRhgsOphsQ5CnsxL2HmW4sIojEs0ilokEj7V3ZyM/fz61utOsGNrNZXDNDcSDMceTtU9eD7+Ncq7EXjQ3skCsVZsSRkeDKfD6H8q6giRxX1pLctlJ9zs4b/mFl5IGfOkKxepd/8AhNLIEubI8/3E+n3klxPI853Mx5LeIHH6V71Qs9uXDKuDjGec+AoCVvgZLh2GO7dk49+Kv0+5jlPdyIZC3Vi2MemPGk/iFgACLNH4ZUSTY0yN/BLcTOCHdSMtkdD/AE/rSK6U2shRjkjoCD+v9q7UdFtrix225aOQDK4jJKn1HWsHquhSfEyAQmcfzd2RlfdDz+dcb3+LYPU7ftchruYvvZA24lufAHrRtlZNJcJ3jfjIGTztU8Fj7ZqX1p3DYt1bHirAg/QGjdF1a2t7qJriLaQcOSucjwFP5ZGRE8cHBnQ7/R7DVOzz29tsd4UKLIo4JX1Nc7gsJLV3RwRLEcHa+A2fI1u7PtDZzJ3VvKiD/CV25+lLruGxmuhcPs4Pytu4pAbjqOILbmWvhP8AJFIyiNiWQBQFHuR196ommz8ojVSPxEePFaDV1tLlAiOdo5wgyaUPbAmNFfeXO1V8R6n1o1cHUBkI3GvZqwMwErjk9M+Vb23tHaFQn4x04A/p/el3Z/Tu7jQbfAVs7K1VVHGKaRkRYOIuSCXH8ZFf/tBH55q4zfCxFbW0bcevdqqj702e2UjgChJbWaMZhwcfyk8GqGu5CQZndPk1Kyv7y4miDC6ZSyxnlCBtHXrxVuoQx35WSSCFj5yQrkflmjma5kdk7naTjkiiINOlYA3D7m9qrOeoWMbMV29ugiZI9y7upUgfTpj7ivdvpECtvZA7nxZAD9xin8dnGn8oz7V7aNFHCioE+5Rf6iOXTYjbyRhMZBNZ/RtUUXRsJz3RJIJA+dgBwoPhk1rrp1CnJwMVyTVWxqtwi5xvO3PWtFdhTqZ7KhZ3Nzd3STRMzOttA+Zooy+9mYYU9OetZTtndiLQJQCc3DKikjGRnJP/AI/nTXQ7z4Wzt7mY5mDloUIBBA4JPp5YrI/tDvmmv4bYvuMamRyOhLdPyA+9dB3xUSOjOWlWbgG7EzWnXklhfQ3UTFXjbOQAePEc+ma63Z3TLLDHOqmO4iSW2kROCNuTjOcAgk4HQ/WuOVsuy+rR3GmxWFw1x8TaTB7aRDnu06nA9D9Oay0PxJBm6+vlgjxCtfuzPftEikq3Iw2STjrmjuzctjZzAaq+1cZVckgnwobXrexktTeaYDFLESXhfcXHmSTwSTkjHhSiaQ6hGkMYVGRfxMeBWS4MWPLzNlJUKAviarUNWmsNUmuLKMSxn5l+fcR9qDuu1kurOIZbYQMDkiVVDH2Jwc1lRFdWzLFGw77JOQwIb2NERavcFxBeMQ+MF3IbA8MeZrMaRiaRZuWXsPfTFd5kDN8oVuT48Hpn0r1ZaY0mWhkid8cLIuWVvXxOKG1G/wBhxGMxFiCVYYf+2KoTtFIqESQRSMDkNjBFDwsx8YXNAflNKtlHEyvM294hgkpgewPFU3cttKwVLchj/nXH5Gpaa/aT2pSc4LDBEgzg+ftWP1CZWvneDaoB47vOPpVJWzaMp7FXYj6XUY7VztKbj0VPmP1NXdnXk1LXEMqnbGpYDOR5Vk0Yh92SD1zW2/Z8xuNRmO0cIBx41pSsJENYWE6jplsoVT6eVOoU2jFBWCHYMjFMUGBTYmfcV5ZcjGTXqvh54xmqMglYjAI2qBjwr3wK+cMWHhXkttJLH5fboaHqF3PZJxwKRdodQ+GtjiXYenHH502nlVELM+1cVzntNqBubgoJNsRyFYYOR55qEwlEU3esXEjuvevKfDc2KUwqb/WLaBmfdK6q5BBIHj4+VXPbqRnvJCF6A8AD14ph2Z7mOe+NyCAWTDIo3EYxgMeVHJ6deKOtcnEGxsDIni9ubfTNcmAVZII8xYL/AIQByQcHpz+dYXULt769muZPxSNn2HgPtTLV5DbwCDayvJyc5/Dng/X9KTUZZsYMXxXPISVdZ3L2lzHPHyUYHBJww8QceB6VTUoZc6bo80Wp2UrbmS2lhCtuIUEqw25OOCpLDI8DSDULeDTJZoDIySxMRvDcFfAjHUUr7Ma0NMuGguixsLn5ZgBkp4bl9R+Y+ldAvtHsLmxtL6C53qU/iIG2lxnIIOOh6fXNaGHvJruIVvZff+M5xILeTn4huPw5Jz+de7G0nuJUjhga7djtRVUlvYCmvabS7W1vSkasxaMSNsJkVAecBsc8Y8Ku7NRJafGs0nw7CEGKdn292wIY4bzxx7sKzBDy4maS448xF+o6Re2USmSORWYkmKRfmBxnofSlUsJCpcZDo55x0H9q0Wu62bm6gFrM9xBaxLEHlQLI4J5zj7Um75ll4AG5eSo4cA85H9aWw4k4jFPJRnuU3QePu3RvlXgHxX0oAnJyaO1Fo2dYoCQo42kdPT2/vVml6ety7CZLjBiZkMUe/wCYdM+nnRoIDmBR4bChOfAjrW7/AGaHbeXIfgkKT61iO8kgYofkGemOlaf9ndx3etMpYfxFOB1JqDuQ9TtlqSUGOBRYKgdfzoCzl3RqR81Fd7sG5kwB4g5qQZcSB1wM14dzt+TB60F+8oW7zCshA43r1HnVLXMcqLJC28YypyQCPEehFCTDCw1HbvSM5GABXm7nWC3aVzhEG7mq4cAsVHReKC1qTNi/gioSCenpVdCX5mR1e/1HU7oB1b4RWI2Jxj1z40oumtosiWQxh/mMZJ3N9KoudcnJ7zu544O82m5f5gAD4qOo9aCvLvuHX4O4+JmkPOJAyHPkOSPaqAJhkgQ+O1S6jZ4pLaIBgFEqt3khOeQAPSvmq6lb6fbxRWzSL3K5n3dZCfDHgc449ATRem3lxp1tOLq2he8gidR8g2xBuSSceHmelYPV9Tl1CQhnzGHZ+OAzHq2PsPYVsAFYB8zGSbCQeoJc3D3U7SyHLN+Q8B9BVVSpSYySpUqVJJK0fZTtIulk2t6He0ZtyMp+aF/Bh5jzH1HrnKlWrFTkSmUMMGdVu9RjvWDajbw3CTx7YpVbaGx0LAA5OOnmPak+rM//AAmLKS3htoYZTiVYj/EAwcA+ByQT6Y8BWS0zVGtGWOdO/tc/NET056r5VrrfWgtnLb2UvxFnIQRvX50I6ZB8RyPUVq5K4mcKyGYYqyDkMoPOD0IpmkTSWvfCNmCAlmHQ/XzFP9USG8S2Sym7yCMOTbtGFeHJ5GPH3HHXpR2iwRx6REr3BhhiJYxFt0c7NxggcZKnB5yOOKzikscR5uCDM5+O9uJwqK0kj8AKMlvYCtN2S1SDTbe+t2vJLG7uCsYlC5XZ/Op8VPQg+lJQ8ukX5kjEXfxllZMMdmQQevv1HNU3d29/dyXMu1XkbOFzgDwAzk4GBUHw35kPz1NB20+Dm1XdaTR3MndorTqD/HIH4uc8+H0qnsnHLFrUP8Fiz/gVSOT59Ofaq9DtGvpY7dNzyMwCxsuN/OSc+GOvpWnSSPszKktpOtzqWWWRjkxxoQOV6Z/X26jtmzC0q4nQEjkD2r2s6SxqxSds4Yjk9MefH60e2GXDS8ddvnSnSpC0EbswYuu4k8E+p8qagpjIxnzqOcyIMQK9Eyr3wlUlfmcbf5T/AC15TIlL7V2ygFWQ/K/+tGPFHnIA/vQqptRAiHAz8o8SKTjcbnULMqogUn8fU+g8KSaxci8gBE8RtX3KyE7S7A4C5PgeftXq70qa9uVN7P3cYOREPxOMc4Hh5ZPFUanNdNFLYQo9wzsTIVj3HGeFzjwHj409EGMtEO5zhZndUtreOIytfWyMw+VRyUHjgdPyoKzitbDdNpyd08cZZry4AXaMZO0YxnyJ86I1TS9N0e1+M1GVIgp+WCHDSSn/AA56e+M4rD61r9zqx7sgQ2wYssKnP3Pif94o8KniDlm8y7W9dNzF8DZs4swdzM3DSt6+gPQfU+iSpUpZJJyYYAHUlSpUqpclSpUqSSVKlSpJJXuCaS3kDxOUYeIrxUqSTT6brmmXKiPV7d4pg2Uu7fqPdfetENYNlA0IWO/tHYHvLcgNkHIZT44JJwfyrm1e4ZpYG3RSMh9DTVt+4o1/U2/caZJIs6vDeBB8vfja581bOQcZ6k+1fdM0LvrB4QsRjkuBJvg2NLGBjIX5ufIZOBz1rO2HaS6tWJeOOYNw2VALf79qdDtHotxGFkt7m3kH86txn6dftWkPW3czlLF6mh06P9yJexwaZKGkTbHJcIGdmPRCQcY4zgdcVnfgp9QeVRIZJVG9pHbaWJ6jB6nPh6Hyo627RpDC0MWoQ3ELkFknYjP3AIPqCKvXXbYkywOkMhAGyG4GGHPJJyc8nnPNC1aNjBhLY65yJpOxtwz6XF3qlGB24bOSRWillkVInC53SbCoxwCeCeeOAaw1jd20J7u2uLC3jAOJLq9UFTjghRnofTmmM3aHs7DZhJtbDzEIZO5JaPcM8gbTnOeeOaD2lHZhm1m6EfatqY02DvIkM0rnbFGATub3HSvnLW8QmvljuVGHMcZQnP4snqCfSsbdftD0iGR2giurmZBsSUIsalfT/CPZRWa1Pt9qN2jx2kcdkrdXQlpP/wCj0+gzQgog13IQ7nfU6beaxFpEDOoBAAXvXHcxj03N8zH2rD61+0NyrRWR74/4tuyEey9W92+xrAyyyTOXlkaR2OSzHJP1rzQGz6jBX9y++vbnULgz3kzzSnjcx6DyHkPQVRUqUuMkqVKlSSSpUqVJJ//Z","sizes":"180x180","type":"image/png"}]})manifest";
void handleManifest() { server.send_P(200, "application/manifest+json", MANIFEST_JSON); }
void handleStatus() { server.send(200, "application/json", buildStatusJson()); }

void handlePumpCmd() {
  if (!server.hasArg("cmd")) { server.send(400,"text/plain","Missing cmd"); return; }
  String cmd = server.arg("cmd");

  if (cmd == "on") {
    // Block if pump is already running from PIR or Touch — don't fight other sources
    if (pumpRunning && pumpSource != SRC_WIFI) {
      server.send(200, "application/json", buildStatusJson());
      return;
    }
    if (!hasWater()) {
      triggerBlocked(SRC_WIFI);
    } else if (pumpRunning && pumpSource == SRC_WIFI && !pumpContinuous) {
      pumpStartTime = millis();   // restart timed run
    } else {
      startPump(SRC_WIFI, wifiDurationMs);
    }
  } else if (cmd == "continuous") {
    // Block if pump is already running from PIR or Touch
    if (pumpRunning && pumpSource != SRC_WIFI) {
      server.send(200, "application/json", buildStatusJson());
      return;
    }
    if (!hasWater()) {
      triggerBlocked(SRC_WIFI);
    } else {
      startPumpForever(SRC_WIFI);
    }
    Serial.println("[WiFi] Pump CONTINUOUS");
  } else if (cmd == "off") {
    // Stop is ALWAYS allowed regardless of source
    pumpOFF();
    tankEmptyError = false;
  }
  server.send(200, "application/json", buildStatusJson());
}

void handleSettings() {
  bool changed = false;
  if (server.hasArg("touch")) { long v=server.arg("touch").toInt(); if(v>=MIN_SEC&&v<=MAX_SEC){touchDurationMs=v*1000UL;changed=true;} }
  if (server.hasArg("pir"))   { long v=server.arg("pir").toInt();   if(v>=MIN_SEC&&v<=MAX_SEC){pirDurationMs  =v*1000UL;changed=true;} }
  if (server.hasArg("wifi"))  { long v=server.arg("wifi").toInt();  if(v>=MIN_SEC&&v<=MAX_SEC){wifiDurationMs =v*1000UL;changed=true;} }
  if (changed) saveSettings();
  server.send(200,"application/json",
    "{\"ok\":true,\"touch_sec\":"+String(touchDurationMs/1000)+
    ",\"pir_sec\":"+String(pirDurationMs/1000)+
    ",\"wifi_sec\":"+String(wifiDurationMs/1000)+"}");
}

// ── SETUP ──────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== PooKooli Fountain v4 ===");

  // Relay — set OUTPUT and immediately force OFF before anything else
  pinMode(PIN_RELAY, OUTPUT);
  pumpOFF();   // ← this is what keeps the pump off at boot

  // Other pins
  pinMode(PIN_TOUCH,    INPUT);
  pinMode(PIN_PIR,      INPUT);
  pinMode(PIN_WATER_DO, INPUT_PULLUP);  // pullup = safe HIGH (empty) when disconnected

  loadSettings();

  // OLED
  Wire.begin(21, 22);   // SDA=21, SCL=22 (ESP32 defaults)
  if (!oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("[OLED] Not found — check wiring/address");
  } else {
    oled.clearDisplay();
    oled.setTextColor(SSD1306_WHITE);
    oled.setTextSize(1);
    oled.setCursor(4, 8);
    oled.print(F("PooKooli Fountain"));
    oled.setCursor(28, 20);
    oled.print(F("Starting..."));
    oled.display();
    Serial.println("[OLED] OK");
  }

  // WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.printf("Connecting to %s", WIFI_SSID);
  int tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 30) {
    delay(500); Serial.print("."); tries++;
  }
  if (WiFi.status() == WL_CONNECTED)
    Serial.printf("\n✓ http://%s\n", WiFi.localIP().toString().c_str());
  else
    Serial.println("\n✗ WiFi failed");

  server.on("/",              handleRoot);
  server.on("/manifest.json",handleManifest);
  server.on("/status",        handleStatus);
  server.on("/pump",          handlePumpCmd);
  server.on("/settings",      handleSettings);
  server.begin();
  Serial.println("Web server ready.");
}

// ── LOOP ───────────────────────────────────────────────────
void loop() {
  server.handleClient();
  unsigned long now = millis();

  // Auto-clear empty-error after EMPTY_BLINK_MS if no new trigger
  if (tankEmptyError && (now - emptyErrorTime > EMPTY_BLINK_MS)) {
    tankEmptyError = false;
  }

  // Auto-stop pump when timer expires (skip in continuous mode)
  if (pumpRunning && !pumpContinuous && (now - pumpStartTime >= pumpDuration)) {
    Serial.println("[Pump] Timer done");
    if (pumpSource == SRC_PIR) lastPirEndTime = now;
    pumpOFF();
  }

  // In continuous mode: stop immediately if tank runs dry
  if (pumpRunning && pumpContinuous && !hasWater()) {
    Serial.println("[Pump] CONTINUOUS — tank empty, stopping immediately");
    pumpOFF();
    triggerBlocked(SRC_NONE);   // show "tank empty" error on OLED + web
  }

  // Update OLED every 300 ms
  if (now - lastOledUpdate >= 300) {
    lastOledUpdate = now;
    updateOled();
  }

  // Touch sensor — stops any running pump, or starts a new timed run
  if (digitalRead(PIN_TOUCH) == HIGH && (now - lastTouchTime > TOUCH_DEBOUNCE_MS)) {
    lastTouchTime = now;
    if (pumpRunning) {
      // Touch always stops pump (both timed and continuous)
      Serial.println("[Touch] Stopping pump");
      if (pumpSource == SRC_PIR) lastPirEndTime = now;
      pumpOFF();
    } else {
      Serial.println("[Touch] Triggered");
      if (hasWater()) startPump(SRC_TOUCH, touchDurationMs);
      else            triggerBlocked(SRC_TOUCH);
    }
    delay(50); return;
  }

  if (pumpRunning) { delay(50); return; }  // ── LOCK: ignore PIR while pump is on from ANY source

  // PIR sensor
  if (digitalRead(PIN_PIR) == HIGH && (now - lastPirEndTime > PIR_LOCKOUT_MS)) {
    Serial.println("[PIR] Motion");
    if (hasWater()) startPump(SRC_PIR, pirDurationMs);
    else            triggerBlocked(SRC_PIR);
  }

  delay(50);
}
