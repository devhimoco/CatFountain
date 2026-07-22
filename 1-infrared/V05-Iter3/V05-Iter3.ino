/*
 ╔══════════════════════════════════════════════════════════╗
 ║     PooKooli Fountain — ESP32 NodeMCU-32  v4             ║
 ║                                                          ║
 ║  Changes from v3:                                        ║
 ║   ✓ Renamed to "PooKooli Fountain" everywhere            ║
 ║   ✓ Continuous mode: pump runs forever until stopped     ║
 ║     via app Stop button or physical touch button         ║
 ╚══════════════════════════════════════════════════════════╝

 ── INSTALL THESE LIBRARIES (Arduino Library Manager) ──────
   • Adafruit SSD1306   (by Adafruit)
   • Adafruit GFX       (by Adafruit)

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

  Relay Module (Tongling JQC-3FF-S-Z — ACTIVE LOW blue board)
    VCC → Vin (5 V from USB)
    GND → GND
    IN  → GPIO 26
    COM → pump positive wire
    NO  → power-supply positive
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
const char* WIFI_SSID     = "YOUR_WIFI_NAME";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";

// GPIO
#define PIN_TOUCH        4
#define PIN_PIR         14
#define PIN_WATER_DO    34   // DO pin — INPUT_PULLUP; LOW = water present
#define PIN_RELAY       26

// Relay: most blue boards are ACTIVE LOW (LOW = coil ON = pump ON)
// If your pump runs backwards, flip this to false.
#define RELAY_ACTIVE_LOW  true

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
  Serial.printf("[Pump] BLOCKED src=%d — tank empty\n", src);
}

// ── OLED DISPLAY ───────────────────────────────────────────
// Called regularly from loop(); keeps display in sync with state.
void updateOled() {
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);

  // ── ERROR: tank empty ──
  if (tankEmptyError) {
    oled.setTextSize(1);
    oled.setCursor(0, 0);
    oled.println(F("!! TANK EMPTY !!"));
    oled.setCursor(0, 12);
    oled.println(F("Please refill"));
    oled.setCursor(0, 24);
    oled.println(F("water reservoir"));
    oled.display();
    return;
  }

  // ── PUMP RUNNING ──
  if (pumpRunning) {
    oled.setTextSize(1);
    oled.setCursor(0, 0);
    const char* srcLabel =
      (pumpSource == SRC_TOUCH) ? "Touch" :
      (pumpSource == SRC_PIR)   ? "PIR/motion" :
                                  "App";
    oled.print(F("Pump ON - "));
    oled.println(srcLabel);

    if (pumpContinuous) {
      // Show elapsed time scrolling up and a ~~ symbol
      unsigned long elSec = (millis() - pumpStartTime) / 1000;
      unsigned int  em = elSec / 60, es = elSec % 60;
      char buf[16];
      snprintf(buf, sizeof(buf), "~~ %02d:%02d ~~", em, es);
      oled.setTextSize(1);
      oled.setCursor(16, 13);
      oled.print(buf);
      oled.setTextSize(1);
      oled.setCursor(8, 24);
      oled.print(F("CONTINUOUS MODE"));
    } else {
      unsigned long elapsed = millis() - pumpStartTime;
      unsigned long remain  = (elapsed >= pumpDuration) ? 0 : (pumpDuration - elapsed);
      unsigned int  remSec  = remain / 1000;
      unsigned int  m = remSec / 60, s = remSec % 60;
      oled.setTextSize(2);
      char buf[8];
      snprintf(buf, sizeof(buf), "%02d:%02d", m, s);
      int x = (OLED_WIDTH - (int)strlen(buf) * 12) / 2;
      oled.setCursor(max(0, x), 13);
      oled.print(buf);
    }

    oled.display();
    return;
  }

  // ── IDLE ──
  bool water = hasWater();
  oled.setTextSize(1);

  // Line 0: device name
  oled.setCursor(0, 0);
  oled.print(F("PooKooli Fountain"));

  // Line 1: IP or "No WiFi"
  oled.setCursor(0, 11);
  if (WiFi.status() == WL_CONNECTED)
    oled.print(WiFi.localIP().toString());
  else
    oled.print(F("WiFi not connected"));

  // Line 2: water status
  oled.setCursor(0, 22);
  if (water) {
    oled.print(F("Water: OK  Ready"));
  } else {
    // Blink the warning when idle and empty
    if ((millis() / 600) % 2 == 0)
      oled.print(F("!! TANK EMPTY !!"));
    else
      oled.print(F("Please add water"));
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
  padding:20px 14px 56px;
  background-image:
    radial-gradient(ellipse 70% 35% at 50% -5%,rgba(56,189,248,.13) 0%,transparent 65%),
    radial-gradient(ellipse 40% 25% at 80% 90%,rgba(74,222,128,.07) 0%,transparent 60%);
}
.hdr{text-align:center;margin-top:8px}
.paw{font-size:3rem;animation:bob 3.5s ease-in-out infinite}
@keyframes bob{0%,100%{transform:translateY(0) rotate(-5deg)}50%{transform:translateY(-9px) rotate(5deg)}}
h1{font-family:'Syne',sans-serif;font-size:1.9rem;font-weight:800;letter-spacing:-.03em;margin-top:10px;
   background:linear-gradient(120deg,#e0f2fe,#38bdf8 50%,#818cf8);
   -webkit-background-clip:text;-webkit-text-fill-color:transparent}
.sub{font-size:.8rem;color:var(--muted);margin-top:3px}

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
  <div class="paw">🐱</div>
  <h1>PooKooli Fountain</h1>
  <p class="sub">Smart water controller</p>
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
    <button class="btn btn-on" onclick="sendCmd('on')">
      <span class="ico">▶</span>
      Start
      <span class="btn-sub" id="wifiDurLabel">2 min</span>
    </button>
    <button class="btn btn-inf" onclick="sendCmd('continuous')">
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

function adj(key, delta) {
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
  try {
    const r = await fetch(`/settings?touch=${timers.touch}&pir=${timers.pir}&wifi=${timers.wifi}`);
    const j = await r.json();
    showToast(j.ok ? '✓ Settings saved!' : '⚠ Save failed', !j.ok);
  } catch(e) { showToast('⚠ No response', true); }
}

async function sendCmd(cmd) {
  try { applyStatus(await (await fetch('/pump?cmd='+cmd)).json()); } catch(e) {}
}

async function fetchStatus() {
  try { applyStatus(await (await fetch('/status')).json()); } catch(e) {}
}

function stopTimerTick()   { if(timerTick)  {clearInterval(timerTick);  timerTick=null;} }
function stopElapsedTick() { if(elapsedTick){clearInterval(elapsedTick);elapsedTick=null;} }
function stopTicks()       { stopTimerTick(); stopElapsedTick(); }

function applyStatus(d) {
  if (d.touch_sec) { timers.touch = d.touch_sec; render('touch'); }
  if (d.pir_sec)   { timers.pir   = d.pir_sec;   render('pir');   }
  if (d.wifi_sec)  { timers.wifi  = d.wifi_sec;  render('wifi');  }

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
  const wrap   = document.getElementById('timerWrap');
  const bar    = document.getElementById('barFill');
  const tt     = document.getElementById('timerText');
  const tlabel = document.getElementById('timerLabel');
  const banner = document.getElementById('errorBanner');

  // ── Empty tank error ──
  if (d.empty_error) {
    banner.classList.add('show');
    dot.className    = 'dot err';
    label.textContent = 'Water tank is empty!';
    sub.className    = 's-sub red';
    sub.textContent  = 'Refill the reservoir to use the pump';
    wrap.classList.remove('active');
    stopTicks(); return;
  }
  banner.classList.remove('show');

  // ── Continuous mode ──
  if (d.running && d.continuous) {
    dot.className     = 'dot inf';
    label.textContent = 'Running continuously';
    sub.className     = 's-sub inf';
    const map = {TOUCH:'Touch sensor',PIR:'PIR sensor',WIFI:'App — infinite'};
    sub.textContent   = (map[src]||src)+' · Stop via app or touch';
    wrap.classList.add('active');
    tlabel.textContent = 'Elapsed';
    bar.className = 'bar-fill inf';
    tt.className  = 't-time inf';
    elapsedStart  = Date.now() - d.elapsed_ms;
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
    dot.className     = 'dot on';
    label.textContent = 'Pump is running';
    sub.className     = 's-sub';
    const map = {TOUCH:'Touch sensor',PIR:'PIR sensor',WIFI:'App control'};
    sub.textContent   = 'Triggered by: '+(map[src]||src);
    wrap.classList.add('active');
    tlabel.textContent = 'Pump running';
    bar.className = 'bar-fill';
    tt.className  = 't-time';
    lastDurationMs = d.duration_ms;
    const remain   = Math.max(0, d.duration_ms - d.elapsed_ms);
    pumpEndTime    = Date.now() + remain;
    bar.style.width = Math.min(100, d.elapsed_ms/d.duration_ms*100)+'%';
    tt.textContent  = fmtMs(remain);
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

  // ── Idle ──
  dot.className     = 'dot off';
  label.textContent = 'Pump is OFF';
  sub.className     = 's-sub';
  sub.textContent   = 'Waiting for trigger…';
  wrap.classList.remove('active');
  stopTicks();
}

function fmtMs(ms) {
  const s=Math.ceil(ms/1000), m=Math.floor(s/60);
  return m+':'+String(s%60).padStart(2,'0');
}
function showToast(msg, isErr=false) {
  const t=document.getElementById('toast');
  t.textContent=msg;
  t.className='toast show'+(isErr?' err':'');
  setTimeout(()=>t.classList.remove('show'),2800);
}

fetchStatus();
setInterval(fetchStatus, 2000);
</script>
</body>
</html>
)rawliteral";

// ── WEB HANDLERS ───────────────────────────────────────────
void handleRoot()   { server.send_P(200, "text/html", INDEX_HTML); }
void handleStatus() { server.send(200, "application/json", buildStatusJson()); }

void handlePumpCmd() {
  if (!server.hasArg("cmd")) { server.send(400,"text/plain","Missing cmd"); return; }
  String cmd = server.arg("cmd");

  if (cmd == "on") {
    if (!hasWater()) {
      triggerBlocked(SRC_WIFI);
    } else if (pumpRunning && pumpSource == SRC_WIFI && !pumpContinuous) {
      pumpStartTime = millis();   // restart timed run
    } else {
      startPump(SRC_WIFI, wifiDurationMs);
    }
  } else if (cmd == "continuous") {
    if (!hasWater()) {
      triggerBlocked(SRC_WIFI);
    } else {
      startPumpForever(SRC_WIFI);
    }
    Serial.println("[WiFi] Pump CONTINUOUS");
  } else if (cmd == "off") {
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

  server.on("/",         handleRoot);
  server.on("/status",   handleStatus);
  server.on("/pump",     handlePumpCmd);
  server.on("/settings", handleSettings);
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

  // Update OLED every 300 ms
  if (now - lastOledUpdate >= 300) {
    lastOledUpdate = now;
    updateOled();
  }

  // Touch sensor — also stops continuous mode
  if (digitalRead(PIN_TOUCH) == HIGH && (now - lastTouchTime > TOUCH_DEBOUNCE_MS)) {
    lastTouchTime = now;
    if (pumpRunning && pumpContinuous) {
      // Touch stops continuous pump
      Serial.println("[Touch] Stopping continuous mode");
      pumpOFF();
    } else if (!pumpRunning) {
      Serial.println("[Touch] Triggered");
      if (hasWater()) startPump(SRC_TOUCH, touchDurationMs);
      else            triggerBlocked(SRC_TOUCH);
    }
    delay(50); return;
  }

  if (pumpRunning) { delay(50); return; }  // skip PIR re-trigger while pump is on

  // PIR sensor
  if (digitalRead(PIN_PIR) == HIGH && (now - lastPirEndTime > PIR_LOCKOUT_MS)) {
    Serial.println("[PIR] Motion");
    if (hasWater()) startPump(SRC_PIR, pirDurationMs);
    else            triggerBlocked(SRC_PIR);
  }

  delay(50);
}
