/*
 ╔══════════════════════════════════════════════════════════╗
 ║         CAT WATER FOUNTAIN - ESP32 NodeMCU-32            ║
 ║                                                          ║
 ║  Parts:                                                  ║
 ║   - NodeMCU-32 (ESP32)                                   ║
 ║   - Touch Sensor  → GPIO 4                               ║
 ║   - PIR Sensor    → GPIO 14                              ║
 ║   - Water Level   → GPIO 34 (analog, ADC)                ║
 ║   - Relay (Pump)  → GPIO 26                              ║
 ║                                                          ║
 ║  Features:                                               ║
 ║   1. Touch  → pump 1 min  (if water present)             ║
 ║   2. PIR    → pump 1 min  (if water present)             ║
 ║   3. WiFi   → pump 2 min  (ON button)                    ║
 ║              pump OFF     (OFF button)                   ║
 ║                                                          ║
 ╚══════════════════════════════════════════════════════════╝

 WIRING GUIDE:
 ┌─────────────────────────────────────────────────────────┐
 │ Touch Sensor (Red module, 3 pins)                        │
 │   VCC → 3.3V                                             │
 │   GND → GND                                              │
 │   OUT → GPIO 4                                           │
 │                                                          │
 │ PIR Sensor (HC-SR501)                                    │
 │   VCC → 5V  (or 3.3V if your module supports it)        │
 │   GND → GND                                              │
 │   OUT → GPIO 14                                          │
 │                                                          │
 │ Water Level Sensor (HW-101 module + fork probe)          │
 │   VCC → 3.3V                                             │
 │   GND → GND                                              │
 │   AO  → GPIO 34  (analog reading)                        │
 │   DO  → NOT USED (or use GPIO 35 for digital threshold)  │
 │                                                          │
 │ Relay Module (Tongling JQC-3FF-S-Z, 5V coil)            │
 │   VCC → 5V (use Vin pin on ESP32 = USB 5V)              │
 │   GND → GND                                              │
 │   IN  → GPIO 26                                          │
 │   COM → Pump positive wire                               │
 │   NO  → Power supply positive                            │
 │   (Pump negative → Power supply negative)               │
 └─────────────────────────────────────────────────────────┘

 NOTE: The relay is ACTIVE LOW on most blue modules.
       Set RELAY_ACTIVE_LOW true if pump turns ON when GPIO=LOW.
       Test it: set to false first, if pump is on at boot → flip to true.
*/

#include <WiFi.h>
#include <WebServer.h>

// ─────────────────────────────────────────
//  USER CONFIGURATION — EDIT THESE
// ─────────────────────────────────────────
const char* WIFI_SSID     = "HiMo 2G";       // <-- change this
const char* WIFI_PASSWORD = "@HiMo9226#";   // <-- change this

// GPIO Pins
#define PIN_TOUCH       4    // Touch sensor output
#define PIN_PIR         14   // PIR sensor output
#define PIN_WATER_LEVEL 34   // Water level sensor analog output
#define PIN_RELAY       26   // Relay IN pin

// Relay logic (most blue relay modules are ACTIVE LOW)
#define RELAY_ACTIVE_LOW  true

// Water level threshold (0-4095). Below this = no water → pump blocked.
// Tune by reading Serial when sensor is dry vs wet.
#define WATER_THRESHOLD   500

// Pump run durations
#define PUMP_DURATION_TOUCH_PIR  (1 * 60 * 1000UL)  // 1 minute in ms
#define PUMP_DURATION_WIFI       (2 * 60 * 1000UL)  // 2 minutes in ms

// Debounce / re-trigger lockout
#define TOUCH_DEBOUNCE_MS   500
#define PIR_LOCKOUT_MS      5000   // don't re-trigger PIR for 5s after pump stops

// ─────────────────────────────────────────
//  GLOBALS
// ─────────────────────────────────────────
WebServer server(80);

enum PumpSource { SRC_NONE, SRC_TOUCH, SRC_PIR, SRC_WIFI };

volatile bool     pumpRunning    = false;
volatile PumpSource pumpSource   = SRC_NONE;
unsigned long     pumpStartTime  = 0;
unsigned long     pumpDuration   = 0;
bool              wifiForceOff   = false;   // OFF button pressed via WiFi

unsigned long     lastTouchTime  = 0;
unsigned long     lastPirEndTime = 0;

// ─────────────────────────────────────────
//  RELAY HELPERS
// ─────────────────────────────────────────
void pumpON() {
  digitalWrite(PIN_RELAY, RELAY_ACTIVE_LOW ? LOW : HIGH);
}

void pumpOFF() {
  digitalWrite(PIN_RELAY, RELAY_ACTIVE_LOW ? HIGH : LOW);
  pumpRunning = false;
  pumpSource  = SRC_NONE;
}

bool hasWater() {
  int level = analogRead(PIN_WATER_LEVEL);
  Serial.printf("[Water] ADC = %d  threshold = %d  → %s\n",
                level, WATER_THRESHOLD, level >= WATER_THRESHOLD ? "WATER OK" : "NO WATER");
  return (level >= WATER_THRESHOLD);
}

void startPump(PumpSource src, unsigned long duration) {
  pumpRunning   = true;
  pumpSource    = src;
  pumpStartTime = millis();
  pumpDuration  = duration;
  wifiForceOff  = false;
  pumpON();
  Serial.printf("[Pump] START — source=%d  duration=%lu ms\n", src, duration);
}

// ─────────────────────────────────────────
//  WEB SERVER HTML  (served from ESP32)
// ─────────────────────────────────────────
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1">
<title>Cat Fountain</title>
<style>
  @import url('https://fonts.googleapis.com/css2?family=DM+Sans:wght@300;500;700&display=swap');
  *{margin:0;padding:0;box-sizing:border-box}
  :root{
    --bg:#0a0e1a;
    --card:#111827;
    --accent:#38bdf8;
    --on:#22c55e;
    --off:#ef4444;
    --text:#f0f6ff;
    --muted:#64748b;
    --border:#1e293b;
  }
  body{
    font-family:'DM Sans',sans-serif;
    background:var(--bg);
    color:var(--text);
    min-height:100vh;
    display:flex;
    flex-direction:column;
    align-items:center;
    padding:24px 16px 40px;
    background-image:
      radial-gradient(ellipse 60% 40% at 50% 0%, rgba(56,189,248,.12) 0%, transparent 70%);
  }
  h1{
    font-size:1.6rem;
    font-weight:700;
    letter-spacing:-.02em;
    margin-top:16px;
  }
  .subtitle{font-size:.85rem;color:var(--muted);margin-top:4px}
  .paw{font-size:2.8rem;margin-top:24px;animation:float 3s ease-in-out infinite}
  @keyframes float{0%,100%{transform:translateY(0)}50%{transform:translateY(-8px)}}

  .card{
    background:var(--card);
    border:1px solid var(--border);
    border-radius:20px;
    padding:24px;
    width:100%;
    max-width:360px;
    margin-top:28px;
  }
  .card-title{font-size:.7rem;font-weight:700;letter-spacing:.12em;text-transform:uppercase;color:var(--muted);margin-bottom:16px}

  /* Status row */
  .status-row{display:flex;align-items:center;gap:10px;margin-bottom:10px}
  .dot{width:10px;height:10px;border-radius:50%;flex-shrink:0}
  .dot.on{background:var(--on);box-shadow:0 0 8px var(--on)}
  .dot.off{background:var(--muted)}
  .status-label{font-size:.95rem;font-weight:500}
  .status-sub{font-size:.78rem;color:var(--muted);margin-top:1px}

  /* Timer bar */
  .timer-wrap{margin:18px 0 0;display:none}
  .timer-wrap.active{display:block}
  .timer-label{font-size:.75rem;color:var(--muted);margin-bottom:6px;display:flex;justify-content:space-between}
  .bar-bg{height:6px;background:var(--border);border-radius:99px;overflow:hidden}
  .bar-fill{height:100%;background:linear-gradient(90deg,var(--accent),var(--on));border-radius:99px;transition:width .9s linear;width:0%}

  /* Buttons */
  .btn-row{display:grid;grid-template-columns:1fr 1fr;gap:12px;margin-top:24px}
  .btn{
    padding:16px 12px;
    border:none;
    border-radius:14px;
    font-family:'DM Sans',sans-serif;
    font-size:1rem;
    font-weight:700;
    cursor:pointer;
    transition:opacity .15s,transform .1s;
    position:relative;
    overflow:hidden;
  }
  .btn:active{transform:scale(.97);opacity:.85}
  .btn-on{background:linear-gradient(135deg,#16a34a,#22c55e);color:#fff}
  .btn-off{background:linear-gradient(135deg,#b91c1c,#ef4444);color:#fff}
  .btn .icon{font-size:1.4rem;display:block;margin-bottom:4px}

  /* Water + source chips */
  .chips{display:flex;gap:8px;flex-wrap:wrap;margin-top:20px}
  .chip{
    font-size:.72rem;font-weight:600;letter-spacing:.05em;
    padding:4px 10px;border-radius:99px;
    background:var(--border);color:var(--muted);
    transition:background .3s,color .3s;
  }
  .chip.active{background:rgba(56,189,248,.18);color:var(--accent);border:1px solid rgba(56,189,248,.3)}
  .chip.water-ok{background:rgba(34,197,94,.15);color:var(--on);border:1px solid rgba(34,197,94,.25)}
  .chip.water-no{background:rgba(239,68,68,.12);color:var(--off);border:1px solid rgba(239,68,68,.2)}

  .refresh-note{font-size:.72rem;color:var(--muted);margin-top:28px;text-align:center;opacity:.6}
</style>
</head>
<body>

<div class="paw">🐱</div>
<h1>Cat Fountain</h1>
<p class="subtitle">Smart water controller</p>

<div class="card">
  <div class="card-title">Status</div>

  <div class="status-row">
    <div class="dot" id="pumpDot"></div>
    <div>
      <div class="status-label" id="pumpLabel">Loading…</div>
      <div class="status-sub" id="pumpSub"></div>
    </div>
  </div>

  <div class="timer-wrap" id="timerWrap">
    <div class="timer-label">
      <span>Pump running</span>
      <span id="timerText">--:--</span>
    </div>
    <div class="bar-bg"><div class="bar-fill" id="barFill"></div></div>
  </div>

  <div class="chips" id="chips">
    <span class="chip" id="chipWater">💧 Water</span>
    <span class="chip" id="chipTouch">👆 Touch</span>
    <span class="chip" id="chipPir">🐾 PIR</span>
    <span class="chip" id="chipWifi">📱 WiFi</span>
  </div>
</div>

<div class="card" style="margin-top:16px">
  <div class="card-title">Control</div>
  <div class="btn-row">
    <button class="btn btn-on" onclick="sendCmd('on')">
      <span class="icon">▶</span>Start (2 min)
    </button>
    <button class="btn btn-off" onclick="sendCmd('off')">
      <span class="icon">⏹</span>Stop Now
    </button>
  </div>
</div>

<p class="refresh-note">Status refreshes every 2 seconds</p>

<script>
let pumpEndTime = 0;
let timerInterval = null;

async function sendCmd(cmd){
  try{
    const r = await fetch('/pump?cmd='+cmd);
    const j = await r.json();
    updateUI(j);
  }catch(e){console.error(e)}
}

async function fetchStatus(){
  try{
    const r = await fetch('/status');
    const j = await r.json();
    updateUI(j);
  }catch(e){}
}

function updateUI(d){
  const dot   = document.getElementById('pumpDot');
  const label = document.getElementById('pumpLabel');
  const sub   = document.getElementById('pumpSub');
  const wrap  = document.getElementById('timerWrap');
  const bar   = document.getElementById('barFill');
  const tText = document.getElementById('timerText');

  // Source chips
  const src = d.source; // "NONE","TOUCH","PIR","WIFI"
  ['Touch','Pir','Wifi'].forEach(s=>{
    document.getElementById('chip'+s).classList.toggle('active', src===s.toUpperCase());
  });

  // Water chip
  const wChip = document.getElementById('chipWater');
  wChip.classList.remove('active','water-ok','water-no');
  if(d.water){wChip.classList.add('water-ok'); wChip.textContent='💧 Water OK';}
  else{wChip.classList.add('water-no'); wChip.textContent='💧 No Water';}

  if(d.running){
    dot.className='dot on';
    label.textContent='Pump is running';
    const srcMap={TOUCH:'Touch sensor',PIR:'PIR sensor',WIFI:'App control'};
    sub.textContent='Triggered by: '+(srcMap[src]||src);

    // timer
    wrap.classList.add('active');
    const elapsed = d.elapsed_ms;
    const total   = d.duration_ms;
    const remain  = Math.max(0, total - elapsed);
    pumpEndTime   = Date.now() + remain;

    const pct = Math.min(100, (elapsed/total)*100);
    bar.style.width = pct+'%';
    tText.textContent = fmtMs(remain);

    if(!timerInterval){
      timerInterval = setInterval(()=>{
        const r2 = Math.max(0, pumpEndTime - Date.now());
        const e2 = total - r2;
        bar.style.width = Math.min(100,(e2/total)*100)+'%';
        tText.textContent = fmtMs(r2);
        if(r2<=0){clearInterval(timerInterval);timerInterval=null;}
      },500);
    }
  } else {
    dot.className='dot off';
    label.textContent='Pump is OFF';
    sub.textContent='Waiting for trigger';
    wrap.classList.remove('active');
    if(timerInterval){clearInterval(timerInterval);timerInterval=null;}
  }
}

function fmtMs(ms){
  const s=Math.ceil(ms/1000);
  const m=Math.floor(s/60);
  const ss=s%60;
  return m+':'+(ss<10?'0':'')+ss;
}

fetchStatus();
setInterval(fetchStatus,2000);
</script>
</body>
</html>
)rawliteral";

// ─────────────────────────────────────────
//  WEB SERVER HANDLERS
// ─────────────────────────────────────────
String buildStatusJson() {
  String src = "NONE";
  if (pumpSource == SRC_TOUCH) src = "TOUCH";
  else if (pumpSource == SRC_PIR)   src = "PIR";
  else if (pumpSource == SRC_WIFI)  src = "WIFI";

  unsigned long elapsed = pumpRunning ? (millis() - pumpStartTime) : 0;

  String json = "{";
  json += "\"running\":"    + String(pumpRunning ? "true" : "false") + ",";
  json += "\"source\":\""   + src + "\",";
  json += "\"water\":"      + String(hasWater() ? "true" : "false") + ",";
  json += "\"elapsed_ms\":" + String(elapsed) + ",";
  json += "\"duration_ms\":" + String(pumpDuration);
  json += "}";
  return json;
}

void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

void handleStatus() {
  server.send(200, "application/json", buildStatusJson());
}

void handlePumpCmd() {
  if (!server.hasArg("cmd")) {
    server.send(400, "text/plain", "Missing cmd");
    return;
  }
  String cmd = server.arg("cmd");

  if (cmd == "on") {
    if (pumpRunning && pumpSource == SRC_WIFI) {
      // restart timer
      pumpStartTime = millis();
    } else {
      // WiFi ON doesn't require water check per spec (override)
      startPump(SRC_WIFI, PUMP_DURATION_WIFI);
    }
    Serial.println("[WiFi] Pump ON command received");
  } else if (cmd == "off") {
    pumpOFF();
    wifiForceOff = true;
    Serial.println("[WiFi] Pump OFF command received");
  }

  server.send(200, "application/json", buildStatusJson());
}

// ─────────────────────────────────────────
//  SETUP
// ─────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n\n=== Cat Water Fountain ===");

  // Pin modes
  pinMode(PIN_TOUCH,       INPUT);
  pinMode(PIN_PIR,         INPUT);
  pinMode(PIN_WATER_LEVEL, INPUT);  // ADC pin — no pull
  pinMode(PIN_RELAY,       OUTPUT);
  pumpOFF();   // ensure pump is off at boot

  // Connect WiFi
  Serial.printf("Connecting to WiFi: %s\n", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\n✓ WiFi connected!\n  IP Address: http://%s\n",
                  WiFi.localIP().toString().c_str());
  } else {
    Serial.println("\n✗ WiFi failed — running in local-only mode (touch+PIR still work)");
  }

  // Web routes
  server.on("/",       handleRoot);
  server.on("/status", handleStatus);
  server.on("/pump",   handlePumpCmd);
  server.begin();
  Serial.println("Web server started.");
}

// ─────────────────────────────────────────
//  LOOP
// ─────────────────────────────────────────
void loop() {
  server.handleClient();

  unsigned long now = millis();

  // ── Auto-stop pump when duration expires ──
  if (pumpRunning && (now - pumpStartTime >= pumpDuration)) {
    Serial.println("[Pump] Duration expired → stopping");
    if (pumpSource == SRC_PIR) lastPirEndTime = now;
    pumpOFF();
  }

  // ── Don't process physical sensors if pump already running ──
  if (pumpRunning) return;

  // ── TOUCH SENSOR ──
  if (digitalRead(PIN_TOUCH) == HIGH) {
    if (now - lastTouchTime > TOUCH_DEBOUNCE_MS) {
      lastTouchTime = now;
      Serial.println("[Touch] Detected!");
      if (hasWater()) {
        startPump(SRC_TOUCH, PUMP_DURATION_TOUCH_PIR);
      } else {
        Serial.println("[Touch] Blocked — no water in tank");
      }
    }
  }

  // ── PIR SENSOR ──
  if (digitalRead(PIN_PIR) == HIGH) {
    if (now - lastPirEndTime > PIR_LOCKOUT_MS) {
      Serial.println("[PIR] Motion detected!");
      if (hasWater()) {
        startPump(SRC_PIR, PUMP_DURATION_TOUCH_PIR);
      } else {
        Serial.println("[PIR] Blocked — no water in tank");
      }
    }
  }

  delay(50);  // small loop delay saves CPU without missing events
}
