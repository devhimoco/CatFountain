/*
 ╔══════════════════════════════════════════════════════════╗
 ║         CAT WATER FOUNTAIN - ESP32 NodeMCU-32            ║
 ║         v2 — Configurable Timers via Web UI              ║
 ║                                                          ║
 ║  All three pump timers can be changed live from the      ║
 ║  web app on your phone — settings are saved to flash     ║
 ║  and survive power cuts.                                 ║
 ║                                                          ║
 ║  GPIO:                                                   ║
 ║   Touch Sensor  → GPIO 4                                 ║
 ║   PIR Sensor    → GPIO 14                                ║
 ║   Water Level   → GPIO 34  (ADC analog)                  ║
 ║   Relay (Pump)  → GPIO 26                                ║
 ╚══════════════════════════════════════════════════════════╝

 WIRING GUIDE:
 ┌─────────────────────────────────────────────────────────┐
 │ Touch Sensor (Red module, 3 pins)                        │
 │   VCC → 3.3V  |  GND → GND  |  OUT → GPIO 4             │
 │                                                          │
 │ PIR Sensor (HC-SR501)                                    │
 │   VCC → 5V    |  GND → GND  |  OUT → GPIO 14            │
 │                                                          │
 │ Water Level Sensor (HW-101)                              │
 │   VCC → 3.3V  |  GND → GND  |  AO  → GPIO 34            │
 │                                                          │
 │ Relay Module (Tongling JQC-3FF-S-Z, 5V coil)            │
 │   VCC → Vin(5V) | GND → GND | IN → GPIO 26              │
 │   COM → Pump(+) | NO → PSU(+)                           │
 └─────────────────────────────────────────────────────────┘
*/

#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>   // NVS flash storage — saves timers across reboots

// ─────────────────────────────────────────
//  USER CONFIGURATION
// ─────────────────────────────────────────
const char* WIFI_SSID     = "HiMo 2G";
const char* WIFI_PASSWORD = "@HiMo9226#";

#define PIN_TOUCH        4
#define PIN_PIR         14
#define PIN_WATER_LEVEL 34
#define PIN_RELAY       26

#define RELAY_ACTIVE_LOW  true   // flip to false if pump logic is reversed
#define WATER_THRESHOLD   500    // ADC 0-4095; tune with Serial when sensor dry vs wet

#define TOUCH_DEBOUNCE_MS  500
#define PIR_LOCKOUT_MS    5000

// Default durations (seconds) — overridden by saved settings
#define DEFAULT_TOUCH_SEC   60
#define DEFAULT_PIR_SEC     60
#define DEFAULT_WIFI_SEC   120

// Min / max allowed from web UI (seconds)
#define MIN_SEC   5
#define MAX_SEC   600   // 10 minutes ceiling

// ─────────────────────────────────────────
//  GLOBALS
// ─────────────────────────────────────────
WebServer   server(80);
Preferences prefs;

// Configurable durations — loaded from flash at boot
unsigned long touchDurationMs = DEFAULT_TOUCH_SEC * 1000UL;
unsigned long pirDurationMs   = DEFAULT_PIR_SEC   * 1000UL;
unsigned long wifiDurationMs  = DEFAULT_WIFI_SEC  * 1000UL;

enum PumpSource { SRC_NONE, SRC_TOUCH, SRC_PIR, SRC_WIFI };

bool          pumpRunning   = false;
PumpSource    pumpSource    = SRC_NONE;
unsigned long pumpStartTime = 0;
unsigned long pumpDuration  = 0;

unsigned long lastTouchTime  = 0;
unsigned long lastPirEndTime = 0;

// ─────────────────────────────────────────
//  FLASH HELPERS
// ─────────────────────────────────────────
void loadSettings() {
  prefs.begin("fountain", true);  // read-only
  touchDurationMs = prefs.getULong("touch_ms", DEFAULT_TOUCH_SEC * 1000UL);
  pirDurationMs   = prefs.getULong("pir_ms",   DEFAULT_PIR_SEC   * 1000UL);
  wifiDurationMs  = prefs.getULong("wifi_ms",  DEFAULT_WIFI_SEC  * 1000UL);
  prefs.end();
  Serial.printf("[Settings] touch=%lu s  pir=%lu s  wifi=%lu s\n",
    touchDurationMs/1000, pirDurationMs/1000, wifiDurationMs/1000);
}

void saveSettings() {
  prefs.begin("fountain", false);  // read-write
  prefs.putULong("touch_ms", touchDurationMs);
  prefs.putULong("pir_ms",   pirDurationMs);
  prefs.putULong("wifi_ms",  wifiDurationMs);
  prefs.end();
  Serial.println("[Settings] Saved to flash.");
}

// ─────────────────────────────────────────
//  RELAY / PUMP HELPERS
// ─────────────────────────────────────────
void pumpON()  { digitalWrite(PIN_RELAY, RELAY_ACTIVE_LOW ? LOW  : HIGH); }
void pumpOFF() {
  digitalWrite(PIN_RELAY, RELAY_ACTIVE_LOW ? HIGH : LOW);
  pumpRunning = false;
  pumpSource  = SRC_NONE;
}

bool hasWater() {
  int v = analogRead(PIN_WATER_LEVEL);
  Serial.printf("[Water] ADC=%d  %s\n", v, v >= WATER_THRESHOLD ? "OK" : "EMPTY");
  return v >= WATER_THRESHOLD;
}

void startPump(PumpSource src, unsigned long dur) {
  pumpRunning   = true;
  pumpSource    = src;
  pumpStartTime = millis();
  pumpDuration  = dur;
  pumpON();
  Serial.printf("[Pump] START src=%d  dur=%lu s\n", src, dur/1000);
}

// ─────────────────────────────────────────
//  JSON HELPERS
// ─────────────────────────────────────────
String srcStr() {
  if (pumpSource == SRC_TOUCH) return "TOUCH";
  if (pumpSource == SRC_PIR)   return "PIR";
  if (pumpSource == SRC_WIFI)  return "WIFI";
  return "NONE";
}

String buildStatusJson() {
  unsigned long elapsed = pumpRunning ? (millis() - pumpStartTime) : 0;
  String j = "{";
  j += "\"running\":"      + String(pumpRunning ? "true":"false") + ",";
  j += "\"source\":\""     + srcStr() + "\",";
  j += "\"water\":"        + String(hasWater() ? "true":"false") + ",";
  j += "\"elapsed_ms\":"   + String(elapsed) + ",";
  j += "\"duration_ms\":"  + String(pumpDuration) + ",";
  j += "\"touch_sec\":"    + String(touchDurationMs / 1000) + ",";
  j += "\"pir_sec\":"      + String(pirDurationMs   / 1000) + ",";
  j += "\"wifi_sec\":"     + String(wifiDurationMs  / 1000);
  j += "}";
  return j;
}

// ─────────────────────────────────────────
//  HTML  (full page served from ESP32)
// ─────────────────────────────────────────
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1">
<title>Cat Fountain</title>
<style>
@import url('https://fonts.googleapis.com/css2?family=Syne:wght@400;700;800&family=DM+Sans:wght@400;500&display=swap');
*{margin:0;padding:0;box-sizing:border-box;-webkit-tap-highlight-color:transparent}
:root{
  --bg:#07090f;
  --card:#0f1520;
  --card2:#131b28;
  --accent:#38bdf8;
  --on:#4ade80;
  --off:#f87171;
  --warn:#fbbf24;
  --text:#e8f0fe;
  --muted:#4a5a72;
  --border:#1a2540;
  --radius:18px;
}
body{
  font-family:'DM Sans',sans-serif;
  background:var(--bg);
  color:var(--text);
  min-height:100vh;
  display:flex;flex-direction:column;align-items:center;
  padding:20px 14px 56px;
  background-image:
    radial-gradient(ellipse 70% 35% at 50% -5%, rgba(56,189,248,.13) 0%, transparent 65%),
    radial-gradient(ellipse 40% 25% at 80% 90%, rgba(74,222,128,.07) 0%, transparent 60%);
}

/* ── Header ── */
.hdr{text-align:center;margin-top:8px}
.paw{font-size:3rem;animation:bob 3.5s ease-in-out infinite}
@keyframes bob{0%,100%{transform:translateY(0) rotate(-5deg)}50%{transform:translateY(-9px) rotate(5deg)}}
h1{font-family:'Syne',sans-serif;font-size:1.9rem;font-weight:800;letter-spacing:-.03em;margin-top:10px;
   background:linear-gradient(120deg,#e0f2fe,#38bdf8 50%,#818cf8);-webkit-background-clip:text;-webkit-text-fill-color:transparent}
.sub{font-size:.8rem;color:var(--muted);margin-top:3px}

/* ── Card ── */
.card{
  background:var(--card);
  border:1px solid var(--border);
  border-radius:var(--radius);
  padding:20px;
  width:100%;max-width:380px;
  margin-top:18px;
}
.card-title{
  font-family:'Syne',sans-serif;
  font-size:.65rem;font-weight:700;letter-spacing:.15em;
  text-transform:uppercase;color:var(--muted);margin-bottom:16px
}

/* ── Pump status ── */
.status-row{display:flex;align-items:center;gap:12px}
.dot{width:12px;height:12px;border-radius:50%;flex-shrink:0;transition:all .4s}
.dot.on{background:var(--on);box-shadow:0 0 0 4px rgba(74,222,128,.2),0 0 12px rgba(74,222,128,.5)}
.dot.off{background:var(--muted)}
.s-label{font-family:'Syne',sans-serif;font-size:1rem;font-weight:700}
.s-sub{font-size:.78rem;color:var(--muted);margin-top:2px}

/* ── Timer bar ── */
.timer-wrap{margin-top:18px;display:none}
.timer-wrap.active{display:block}
.t-row{display:flex;justify-content:space-between;font-size:.75rem;color:var(--muted);margin-bottom:7px}
.t-time{font-family:'Syne',sans-serif;font-weight:700;font-size:.85rem;color:var(--accent)}
.bar-bg{height:5px;background:var(--border);border-radius:99px;overflow:hidden}
.bar-fill{height:100%;width:0%;border-radius:99px;transition:width .8s linear;
          background:linear-gradient(90deg,var(--accent),var(--on))}

/* ── Chips ── */
.chips{display:flex;gap:7px;flex-wrap:wrap;margin-top:18px}
.chip{font-size:.7rem;font-weight:500;padding:4px 11px;border-radius:99px;
      background:var(--border);color:var(--muted);transition:all .3s;border:1px solid transparent}
.chip.active{background:rgba(56,189,248,.15);color:var(--accent);border-color:rgba(56,189,248,.3)}
.chip.water-ok{background:rgba(74,222,128,.12);color:var(--on);border-color:rgba(74,222,128,.3)}
.chip.water-no{background:rgba(248,113,113,.1);color:var(--off);border-color:rgba(248,113,113,.25)}

/* ── Control buttons ── */
.btn-grid{display:grid;grid-template-columns:1fr 1fr;gap:10px;margin-top:16px}
.btn{
  display:flex;flex-direction:column;align-items:center;justify-content:center;
  gap:5px;padding:16px 8px;border:none;border-radius:14px;
  font-family:'Syne',sans-serif;font-size:.9rem;font-weight:700;
  cursor:pointer;transition:transform .12s,opacity .15s;
}
.btn:active{transform:scale(.95);opacity:.8}
.btn .ico{font-size:1.5rem}
.btn-on {background:linear-gradient(145deg,#166534,#22c55e);color:#fff}
.btn-off{background:linear-gradient(145deg,#991b1b,#ef4444);color:#fff}
.btn-sub{font-size:.65rem;font-weight:400;opacity:.8;font-family:'DM Sans',sans-serif}

/* ── Settings card ── */
.settings-card{background:var(--card2)}

.timer-setting{
  display:flex;align-items:center;justify-content:space-between;
  padding:13px 0;
  border-bottom:1px solid var(--border);
}
.timer-setting:last-child{border-bottom:none}

.ts-label{font-size:.88rem;font-weight:500}
.ts-label small{display:block;font-size:.72rem;color:var(--muted);margin-top:1px}

.ts-controls{display:flex;align-items:center;gap:0}
.ts-btn{
  width:34px;height:34px;border:1px solid var(--border);background:var(--bg);
  color:var(--text);font-size:1.1rem;font-weight:700;border-radius:8px;
  cursor:pointer;transition:background .15s;display:flex;align-items:center;justify-content:center;
}
.ts-btn:active{background:var(--border)}
.ts-val{
  min-width:54px;text-align:center;
  font-family:'Syne',sans-serif;font-size:.95rem;font-weight:700;
  color:var(--accent);
  padding:0 4px;
}

.save-btn{
  width:100%;margin-top:16px;padding:14px;
  background:linear-gradient(135deg,#1d4ed8,#38bdf8);
  color:#fff;border:none;border-radius:12px;
  font-family:'Syne',sans-serif;font-size:.95rem;font-weight:700;
  cursor:pointer;transition:opacity .15s,transform .1s;
  display:flex;align-items:center;justify-content:center;gap:8px;
}
.save-btn:active{transform:scale(.97);opacity:.85}
.save-btn .save-ico{font-size:1.1rem}

.toast{
  position:fixed;bottom:24px;left:50%;transform:translateX(-50%) translateY(80px);
  background:#1e293b;color:var(--on);
  padding:10px 22px;border-radius:99px;font-size:.85rem;font-weight:600;
  border:1px solid rgba(74,222,128,.3);
  transition:transform .35s cubic-bezier(.34,1.56,.64,1),opacity .35s;
  opacity:0;white-space:nowrap;z-index:999;
}
.toast.show{transform:translateX(-50%) translateY(0);opacity:1}

.note{font-size:.7rem;color:var(--muted);margin-top:24px;text-align:center;opacity:.55;line-height:1.6}
</style>
</head>
<body>

<div class="hdr">
  <div class="paw">🐱</div>
  <h1>Cat Fountain</h1>
  <p class="sub">Smart water controller</p>
</div>

<!-- STATUS CARD -->
<div class="card">
  <div class="card-title">Live Status</div>
  <div class="status-row">
    <div class="dot" id="pumpDot"></div>
    <div>
      <div class="s-label" id="pumpLabel">Loading…</div>
      <div class="s-sub"   id="pumpSub"></div>
    </div>
  </div>

  <div class="timer-wrap" id="timerWrap">
    <div class="t-row">
      <span>Pump running</span>
      <span class="t-time" id="timerText">--:--</span>
    </div>
    <div class="bar-bg"><div class="bar-fill" id="barFill"></div></div>
  </div>

  <div class="chips" id="chips">
    <span class="chip" id="chipWater">💧 Water</span>
    <span class="chip" id="chipTouch">👆 Touch</span>
    <span class="chip" id="chipPir">🐾 PIR</span>
    <span class="chip" id="chipWifi">📱 App</span>
  </div>
</div>

<!-- CONTROL CARD -->
<div class="card" style="margin-top:12px">
  <div class="card-title">Control</div>
  <div class="btn-grid">
    <button class="btn btn-on" onclick="sendCmd('on')">
      <span class="ico">▶</span>
      Start Pump
      <span class="btn-sub" id="wifiDurLabel">2 min</span>
    </button>
    <button class="btn btn-off" onclick="sendCmd('off')">
      <span class="ico">⏹</span>
      Stop Now
      <span class="btn-sub">immediately</span>
    </button>
  </div>
</div>

<!-- SETTINGS CARD -->
<div class="card settings-card" style="margin-top:12px">
  <div class="card-title">⏱ Timer Settings</div>

  <div class="timer-setting">
    <div class="ts-label">
      👆 Touch Button
      <small>Physical touch sensor</small>
    </div>
    <div class="ts-controls">
      <button class="ts-btn" onclick="adj('touch',-10)">−</button>
      <div class="ts-val" id="val-touch">60s</div>
      <button class="ts-btn" onclick="adj('touch',+10)">+</button>
    </div>
  </div>

  <div class="timer-setting">
    <div class="ts-label">
      🐾 PIR Sensor
      <small>Cat motion detected</small>
    </div>
    <div class="ts-controls">
      <button class="ts-btn" onclick="adj('pir',-10)">−</button>
      <div class="ts-val" id="val-pir">60s</div>
      <button class="ts-btn" onclick="adj('pir',+10)">+</button>
    </div>
  </div>

  <div class="timer-setting">
    <div class="ts-label">
      📱 App Start Button
      <small>Started from this page</small>
    </div>
    <div class="ts-controls">
      <button class="ts-btn" onclick="adj('wifi',-10)">−</button>
      <div class="ts-val" id="val-wifi">120s</div>
      <button class="ts-btn" onclick="adj('wifi',+10)">+</button>
    </div>
  </div>

  <button class="save-btn" onclick="saveSettings()">
    <span class="save-ico">💾</span> Save Settings to Device
  </button>
</div>

<p class="note">Timers adjust in 10-second steps (5 s min · 600 s max)<br>Settings saved to device flash — survive power cuts</p>

<div class="toast" id="toast"></div>

<script>
// ── Local timer state (seconds) ──
const timers = { touch: 60, pir: 60, wifi: 120 };
const MIN = 5, MAX = 600;

let pumpEndTime = 0;
let timerTick   = null;

// ── Adjust a timer value ──
function adj(key, delta) {
  timers[key] = Math.min(MAX, Math.max(MIN, timers[key] + delta));
  render(key);
}

function render(key) {
  const v = timers[key];
  const el = document.getElementById('val-' + key);
  el.textContent = v >= 60 ? (Math.floor(v/60) + 'm' + (v%60 ? (v%60)+'s' : '')) : v + 's';
  if (key === 'wifi') {
    document.getElementById('wifiDurLabel').textContent =
      v >= 60 ? (Math.floor(v/60)+'m'+(v%60?v%60+'s':'')) : v+'s';
  }
}

// ── Save settings to device ──
async function saveSettings() {
  try {
    const url = `/settings?touch=${timers.touch}&pir=${timers.pir}&wifi=${timers.wifi}`;
    const r = await fetch(url);
    const j = await r.json();
    if (j.ok) showToast('✓ Settings saved!');
    else      showToast('⚠ Save failed');
  } catch(e) { showToast('⚠ No response'); }
}

// ── Send pump command ──
async function sendCmd(cmd) {
  try {
    const r = await fetch('/pump?cmd=' + cmd);
    const j = await r.json();
    applyStatus(j);
  } catch(e) { console.error(e); }
}

// ── Poll status ──
async function fetchStatus() {
  try {
    const r = await fetch('/status');
    const j = await r.json();
    applyStatus(j);
  } catch(e) {}
}

function applyStatus(d) {
  // Sync timers from device
  if (d.touch_sec && d.touch_sec !== timers.touch) { timers.touch = d.touch_sec; render('touch'); }
  if (d.pir_sec   && d.pir_sec   !== timers.pir)   { timers.pir   = d.pir_sec;   render('pir');   }
  if (d.wifi_sec  && d.wifi_sec  !== timers.wifi)  { timers.wifi  = d.wifi_sec;  render('wifi');  }

  // Chips
  const src = d.source;
  document.getElementById('chipTouch').classList.toggle('active', src === 'TOUCH');
  document.getElementById('chipPir').classList.toggle('active',   src === 'PIR');
  document.getElementById('chipWifi').classList.toggle('active',  src === 'WIFI');

  const wChip = document.getElementById('chipWater');
  wChip.className = 'chip ' + (d.water ? 'water-ok' : 'water-no');
  wChip.textContent = d.water ? '💧 Water OK' : '💧 No Water';

  const dot   = document.getElementById('pumpDot');
  const label = document.getElementById('pumpLabel');
  const sub   = document.getElementById('pumpSub');
  const wrap  = document.getElementById('timerWrap');
  const bar   = document.getElementById('barFill');
  const tt    = document.getElementById('timerText');

  if (d.running) {
    dot.className = 'dot on';
    label.textContent = 'Pump is running';
    const map = { TOUCH:'Touch sensor', PIR:'PIR sensor', WIFI:'App control' };
    sub.textContent = 'Triggered by: ' + (map[src] || src);

    wrap.classList.add('active');
    const remain = Math.max(0, d.duration_ms - d.elapsed_ms);
    pumpEndTime = Date.now() + remain;
    bar.style.width = Math.min(100, (d.elapsed_ms / d.duration_ms) * 100) + '%';
    tt.textContent = fmtMs(remain);

    if (!timerTick) {
      timerTick = setInterval(() => {
        const r2 = Math.max(0, pumpEndTime - Date.now());
        const e2 = d.duration_ms - r2;
        bar.style.width = Math.min(100, (e2 / d.duration_ms) * 100) + '%';
        tt.textContent = fmtMs(r2);
        if (r2 <= 0) { clearInterval(timerTick); timerTick = null; }
      }, 400);
    }
  } else {
    dot.className = 'dot off';
    label.textContent = 'Pump is OFF';
    sub.textContent = 'Waiting for trigger…';
    wrap.classList.remove('active');
    if (timerTick) { clearInterval(timerTick); timerTick = null; }
  }
}

function fmtMs(ms) {
  const s = Math.ceil(ms / 1000);
  const m = Math.floor(s / 60);
  return m + ':' + String(s % 60).padStart(2, '0');
}

function showToast(msg) {
  const t = document.getElementById('toast');
  t.textContent = msg;
  t.classList.add('show');
  setTimeout(() => t.classList.remove('show'), 2800);
}

// Boot
fetchStatus();
setInterval(fetchStatus, 2000);
</script>
</body>
</html>
)rawliteral";

// ─────────────────────────────────────────
//  WEB HANDLERS
// ─────────────────────────────────────────
void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

void handleStatus() {
  server.send(200, "application/json", buildStatusJson());
}

void handlePumpCmd() {
  if (!server.hasArg("cmd")) { server.send(400, "text/plain", "Missing cmd"); return; }
  String cmd = server.arg("cmd");

  if (cmd == "on") {
    if (pumpRunning && pumpSource == SRC_WIFI) {
      pumpStartTime = millis();   // restart timer
    } else {
      startPump(SRC_WIFI, wifiDurationMs);
    }
    Serial.println("[WiFi] Pump ON");
  } else if (cmd == "off") {
    pumpOFF();
    Serial.println("[WiFi] Pump OFF");
  }
  server.send(200, "application/json", buildStatusJson());
}

void handleSettings() {
  bool changed = false;

  if (server.hasArg("touch")) {
    long v = server.arg("touch").toInt();
    if (v >= MIN_SEC && v <= MAX_SEC) { touchDurationMs = v * 1000UL; changed = true; }
  }
  if (server.hasArg("pir")) {
    long v = server.arg("pir").toInt();
    if (v >= MIN_SEC && v <= MAX_SEC) { pirDurationMs = v * 1000UL; changed = true; }
  }
  if (server.hasArg("wifi")) {
    long v = server.arg("wifi").toInt();
    if (v >= MIN_SEC && v <= MAX_SEC) { wifiDurationMs = v * 1000UL; changed = true; }
  }

  if (changed) saveSettings();

  String json = "{\"ok\":true,\"touch_sec\":" + String(touchDurationMs/1000)
              + ",\"pir_sec\":"               + String(pirDurationMs/1000)
              + ",\"wifi_sec\":"              + String(wifiDurationMs/1000) + "}";
  server.send(200, "application/json", json);

  Serial.printf("[Settings] Updated → touch=%lu s  pir=%lu s  wifi=%lu s\n",
    touchDurationMs/1000, pirDurationMs/1000, wifiDurationMs/1000);
}

// ─────────────────────────────────────────
//  SETUP
// ─────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(400);
  Serial.println("\n=== Cat Water Fountain v2 ===");

  loadSettings();

  pinMode(PIN_TOUCH,       INPUT);
  pinMode(PIN_PIR,         INPUT);
  pinMode(PIN_WATER_LEVEL, INPUT);
  pinMode(PIN_RELAY,       OUTPUT);
  pumpOFF();

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.printf("Connecting to %s", WIFI_SSID);
  int tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 30) { delay(500); Serial.print("."); tries++; }
  if (WiFi.status() == WL_CONNECTED)
    Serial.printf("\n✓ http://%s\n", WiFi.localIP().toString().c_str());
  else
    Serial.println("\n✗ WiFi failed — touch+PIR still work");

  server.on("/",         handleRoot);
  server.on("/status",   handleStatus);
  server.on("/pump",     handlePumpCmd);
  server.on("/settings", handleSettings);
  server.begin();
  Serial.println("Web server ready.");
}

// ─────────────────────────────────────────
//  LOOP
// ─────────────────────────────────────────
void loop() {
  server.handleClient();
  unsigned long now = millis();

  // Auto-stop when duration expires
  if (pumpRunning && (now - pumpStartTime >= pumpDuration)) {
    Serial.println("[Pump] Timer expired — stopping");
    if (pumpSource == SRC_PIR) lastPirEndTime = now;
    pumpOFF();
  }

  if (pumpRunning) return;   // don't re-trigger while running

  // Touch sensor
  if (digitalRead(PIN_TOUCH) == HIGH && (now - lastTouchTime > TOUCH_DEBOUNCE_MS)) {
    lastTouchTime = now;
    Serial.println("[Touch] Triggered");
    if (hasWater()) startPump(SRC_TOUCH, touchDurationMs);
    else Serial.println("[Touch] Blocked — no water");
  }

  // PIR sensor
  if (digitalRead(PIN_PIR) == HIGH && (now - lastPirEndTime > PIR_LOCKOUT_MS)) {
    Serial.println("[PIR] Motion detected");
    if (hasWater()) startPump(SRC_PIR, pirDurationMs);
    else Serial.println("[PIR] Blocked — no water");
  }

  delay(50);
}
