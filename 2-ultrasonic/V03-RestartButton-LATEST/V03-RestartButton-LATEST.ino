/*
 ╔══════════════════════════════════════════════════════════╗
 ║     PooKooli Fountain — ESP32-DevKitC (WROOM-32D)  v7.3 with Restrat Button   ║
 ║                                                          ║
 ║  Changes vs v6:                                          ║
 ║   ✓ IR sensor REPLACED with HC-SR04 ultrasonic sensor    ║
 ║   ✓ Distance threshold setting (2-100 cm, web UI)        ║
 ║   ✓ Master ON/OFF switch — kills the whole mechanism     ║
 ║   ✓ Ultrasonic ON/OFF switch — disables only that sensor ║
 ║   ✓ Touch ON/OFF switch — disables only that sensor      ║
 ║   ✓ All three switches + distance saved to flash (NVS)   ║
 ║   ✓ Multi-sample median filter on ultrasonic readings    ║
 ║     (fights noise-driven false triggers / glitches —     ║
 ║      does NOT fix the sensor's narrow ~15° beam angle,   ║
 ║      see notes near sr04Stable())                        ║
 ║   ✓ Hardware watchdog: auto-resets on a genuine hang,    ║
 ║     NOT a timed reboot. Pump forced OFF on every boot.   ║
 ╚══════════════════════════════════════════════════════════╝

 ── LIBRARIES  (Arduino Library Manager) ────────────────────
   • Adafruit SSD1306   (by Adafruit)
   • Adafruit GFX       (by Adafruit)
   Board: "ESP32 Dev Module"  by Espressif

 ── BOARD PROTECTION (read before wiring!) ──────────────────
  1. 1N4007 diode: cathode→Pump(+), anode→Pump(−)  ← MUST
  2. 1000µF cap:   Vin(+) to GND on ESP32          ← MUST
  3. 100nF  cap:   3.3V to GND on ESP32             ← MUST
  4. 100µF  cap:   Pump PSU (+) to (−)              ← MUST
  5. 220Ω resistor: series on EACH sensor wire       ← RECOMMENDED
  6. Two separate USB chargers (ESP vs pump)         ← RECOMMENDED
  NEVER connect pump power supply ground directly
  to ESP32 ground — use the PC817 isolation.

 ── WIRING ───────────────────────────────────────────────────
  OLED 0.91" SSD1306:  VCC→3.3V  GND→GND  SDA→GPIO21  SCL→GPIO22
  Touch sensor:        VCC→3.3V  GND→GND  OUT→GPIO4
  HC-SR04 ultrasonic:  VCC→5V    GND→GND  TRIG→GPIO27  ECHO→GPIO12
                       NOTE: ECHO is 5V logic — use a voltage divider
                       (e.g. 1kΩ/2kΩ) or level shifter to GPIO12, or
                       you risk damaging the ESP32 input pin.
                       GPIO12 is a strapping pin (boot flash voltage) —
                       it must read LOW at boot, which it will since the
                       HC-SR04 ECHO line idles LOW. Don't add a pull-up.
  Water level (HW-101):VCC→3.3V  GND→GND  DO →GPIO13
  MOSFET module:       Control GND→GND     PWM→GPIO26
 ─────────────────────────────────────────────────────────────
*/

#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <esp_task_wdt.h>   // hardware watchdog — auto-resets if loop() ever hangs

// ── WiFi ─────────────────────────────────────────────────────
const char* WIFI_SSID     = "HiMo 2G";
const char* WIFI_PASSWORD = "@HiMo9226#";

// ── GPIO Pins ────────────────────────────────────────────────
#define PIN_TOUCH     4    // Touch sensor output (HIGH = touched)
#define PIN_SR04_TRIG 27   // HC-SR04 ultrasonic — trigger pulse out
#define PIN_SR04_ECHO 12   // HC-SR04 ultrasonic — echo pulse in
                          // ECHO is 5V logic on the HC-SR04 — use a
                          // voltage divider / level shifter to GPIO12
                          // GPIO12 is a strapping pin — must be LOW at
                          // boot; OK since ECHO idles LOW, but don't
                          // add a pull-up resistor on this line.
#define PIN_WATER_DO 13    // Water level DO (see WATER_DO_LOW_MEANS_WET)
#define PIN_PUMP     26    // MOSFET module PWM input (via LEDC)
// OLED: SDA=21, SCL=22 (ESP32 defaults — set in Wire.begin())

// ── Water sensor polarity ────────────────────────────────────
// HW-101 DO: set to true  if LOW  means water present (most common)
//            set to false if HIGH means water present (some modules)
// If your tank shows EMPTY when FULL, flip this to the other value.
#define WATER_DO_LOW_MEANS_WET  true

// ── LEDC PWM (pump motor speed) ──────────────────────────────
#define LEDC_FREQ_HZ    1000    // 1 kHz — good for DC pump motor
#define LEDC_RESOLUTION 10      // 10-bit: 0 – 1023

// ── Defaults ─────────────────────────────────────────────────
#define DEFAULT_TOUCH_SEC    60
#define DEFAULT_PIR_SEC      60
#define DEFAULT_WIFI_SEC    120
#define DEFAULT_PUMP_POWER    8   // 1-10; 8 = 80% power
#define DEFAULT_DISTANCE_CM   9   // trigger when object closer than this
#define MIN_SEC   5
#define MAX_SEC 600
#define MIN_DISTANCE_CM   2
#define MAX_DISTANCE_CM 100

// ── HC-SR04 ultrasonic sensor ──────────────────────────────────
#define SR04_TIMEOUT_US   30000   // ~5m max range; avoids blocking forever
                                  // if no echo returns

// ── Hardware watchdog ───────────────────────────────────────────
// If loop() doesn't check in within this many seconds (frozen WiFi
// stack, hung sensor read, etc.), the ESP32 hardware forces a reset.
// This is a SAFETY NET for hangs, not a fix for the sensor's beam-angle
// limitation, and not a blind timed reboot — it only fires if something
// has genuinely gone wrong.
#define WDT_TIMEOUT_SEC 10

// ── Timing constants ─────────────────────────────────────────
#define TOUCH_DEBOUNCE_MS   500
#define SR04_LOCKOUT_MS    5000
#define EMPTY_BLINK_MS     3000

// ── Noise-rejection (software debounce) ───────────────────────
// Touch and ultrasonic signals must hold STABLE for this many ms
// before being accepted as real — filters electrical noise spikes
// from the pump's PWM switching, and filters single-reading echo
// glitches from the HC-SR04, which a single distance reading can't
// tell apart from a genuine touch/detection.
#define TOUCH_CONFIRM_MS     60   // touch must stay HIGH this long
#define SR04_CONFIRM_MS     120   // object must stay near this long

// ── OLED ─────────────────────────────────────────────────────
#define OLED_WIDTH  128
#define OLED_HEIGHT  32
#define OLED_ADDR   0x3C
#define OLED_RESET    -1

// ── Globals ──────────────────────────────────────────────────
WebServer    server(80);
Preferences  prefs;
Adafruit_SSD1306 oled(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RESET);

unsigned long touchDurationMs = DEFAULT_TOUCH_SEC * 1000UL;
unsigned long pirDurationMs   = DEFAULT_PIR_SEC   * 1000UL;   // kept name for NVS/JSON compat — now drives the ultrasonic trigger
unsigned long wifiDurationMs  = DEFAULT_WIFI_SEC  * 1000UL;
uint8_t       pumpPower       = DEFAULT_PUMP_POWER;
uint8_t       triggerDistanceCm = DEFAULT_DISTANCE_CM;  // HC-SR04: trigger when object closer than this

// ── Master / per-sensor enable switches (web app toggles) ──────
bool          systemEnabled    = true;   // master switch — whole mechanism
bool          sr04Enabled      = true;   // ultrasonic sensor switch
bool          touchEnabled     = true;   // touch sensor switch

enum PumpSource { SRC_NONE, SRC_TOUCH, SRC_PIR, SRC_WIFI };

bool          pumpRunning    = false;
bool          pumpContinuous = false;
PumpSource    pumpSource     = SRC_NONE;
unsigned long pumpStartTime  = 0;
unsigned long pumpDuration   = 0;
bool          tankEmptyError = false;
unsigned long emptyErrorTime = 0;

unsigned long lastTouchTime  = 0;
unsigned long lastIrEndTime  = 0;
unsigned long lastOledUpdate = 0;

// Debounce state (raw candidate + when it started)
bool          touchCandidate      = false;
unsigned long touchCandidateSince = 0;
bool          sr04Candidate       = false;
unsigned long sr04CandidateSince  = 0;

// ── NVS / Preferences ────────────────────────────────────────
void loadSettings() {
  prefs.begin("fountain", true);
  touchDurationMs = prefs.getULong("touch_ms", DEFAULT_TOUCH_SEC * 1000UL);
  pirDurationMs   = prefs.getULong("pir_ms",   DEFAULT_PIR_SEC   * 1000UL);
  wifiDurationMs  = prefs.getULong("wifi_ms",  DEFAULT_WIFI_SEC  * 1000UL);
  pumpPower       = prefs.getUChar("pump_pwr", DEFAULT_PUMP_POWER);
  if (pumpPower < 1 || pumpPower > 10) pumpPower = DEFAULT_PUMP_POWER;
  triggerDistanceCm = prefs.getUChar("dist_cm", DEFAULT_DISTANCE_CM);
  if (triggerDistanceCm < MIN_DISTANCE_CM || triggerDistanceCm > MAX_DISTANCE_CM)
    triggerDistanceCm = DEFAULT_DISTANCE_CM;
  systemEnabled = prefs.getBool("sys_en",  true);
  sr04Enabled   = prefs.getBool("sr04_en", true);
  touchEnabled  = prefs.getBool("touch_en",true);
  prefs.end();
  Serial.printf("[NVS] touch=%lus sr04=%lus wifi=%lus power=%d dist=%dcm sys=%d sr04on=%d touchon=%d\n",
    touchDurationMs/1000, pirDurationMs/1000, wifiDurationMs/1000, pumpPower,
    triggerDistanceCm, systemEnabled, sr04Enabled, touchEnabled);
}
void saveSettings() {
  prefs.begin("fountain", false);
  prefs.putULong("touch_ms", touchDurationMs);
  prefs.putULong("pir_ms",   pirDurationMs);
  prefs.putULong("wifi_ms",  wifiDurationMs);
  prefs.putUChar("pump_pwr", pumpPower);
  prefs.putUChar("dist_cm",  triggerDistanceCm);
  prefs.putBool("sys_en",   systemEnabled);
  prefs.putBool("sr04_en",  sr04Enabled);
  prefs.putBool("touch_en", touchEnabled);
  prefs.end();
  Serial.println("[NVS] Settings saved");
}

// ── Water sensor ─────────────────────────────────────────────
bool hasWater() {
  bool rawLow = (digitalRead(PIN_WATER_DO) == LOW);
  return WATER_DO_LOW_MEANS_WET ? rawLow : !rawLow;
}

// ── Noise-rejecting debounce for Touch + Ultrasonic ───────────
// Returns true only once the raw signal has been continuously
// active for >= CONFIRM_MS. Any brief glitch (noise spike from
// the pump's PWM switching, or a stray HC-SR04 echo misread)
// resets the timer back to zero, so it can never accumulate
// enough time to be accepted. A genuine touch or a cat sitting
// in front of the sensor easily holds for hundreds of ms, so
// this doesn't add noticeable lag.
bool touchStable() {
  if (!touchEnabled) { touchCandidate = false; return false; }
  bool raw = (digitalRead(PIN_TOUCH) == HIGH);
  if (raw != touchCandidate) {
    touchCandidate      = raw;
    touchCandidateSince = millis();
  }
  return raw && (millis() - touchCandidateSince >= TOUCH_CONFIRM_MS);
}

// Reads HC-SR04 distance ONCE in cm. Returns -1 if no echo (out of
// range, sensor fault, or the ping missed entirely — common when the
// object is at the edge of the ~15° beam cone). Single reads are noisy;
// callers needing a trustworthy value should use readDistanceCmFiltered().
long readDistanceCm() {
  digitalWrite(PIN_SR04_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_SR04_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_SR04_TRIG, LOW);
  unsigned long us = pulseIn(PIN_SR04_ECHO, HIGH, SR04_TIMEOUT_US);
  if (us == 0) return -1;           // timeout — no object in range
  return (long)(us / 58);           // speed of sound ≈ 343 m/s
}

// Takes several quick readings and returns a noise-rejected estimate.
// HC-SR04 false/garbage readings are usually one-off spikes (electrical
// noise from the pump's PWM, a stray reflection) rather than a
// consistent wrong value, so a median of several short-interval samples
// throws out spikes that a single reading can't tell from a real
// detection. Returns -1 if too few of the samples agree there's
// something in range at all (i.e. mostly timeouts — nothing there).
#define SR04_SAMPLES 5
long readDistanceCmFiltered() {
  long vals[SR04_SAMPLES];
  int  validCount = 0;
  for (int i = 0; i < SR04_SAMPLES; i++) {
    long d = readDistanceCm();
    if (d > 0) vals[validCount++] = d;
    delayMicroseconds(1500);   // let the sensor's prior echo fully die down
                                // before the next ping — pinging too fast is
                                // a common cause of HC-SR04 ghost readings
  }
  // Need at least a majority of samples to agree something is there;
  // otherwise treat it as "nothing in range" rather than guessing.
  if (validCount < (SR04_SAMPLES/2 + 1)) return -1;
  // Simple insertion sort (N is tiny) then take the median.
  for (int i = 1; i < validCount; i++) {
    long key = vals[i]; int j = i-1;
    while (j >= 0 && vals[j] > key) { vals[j+1]=vals[j]; j--; }
    vals[j+1] = key;
  }
  return vals[validCount/2];
}

bool sr04Stable() {
  if (!sr04Enabled) { sr04Candidate = false; return false; }
  long d = readDistanceCmFiltered();
  bool raw = (d > 0 && d <= (long)triggerDistanceCm);
  if (raw != sr04Candidate) {
    sr04Candidate      = raw;
    sr04CandidateSince = millis();
  }
  return raw && (millis() - sr04CandidateSince >= SR04_CONFIRM_MS);
}
// NOTE on direction-dependent misses: the HC-SR04 has a narrow ~15° beam.
// An object centered in front of the sensor reflects sound straight back;
// the same object 10cm away but off to the side is mostly outside that
// cone, so the ping passes by it and comes back as a timeout (read as
// "nothing there") rather than a wrong distance. This filtering layer
// fixes noise-driven false triggers and one-off missed pings — it cannot
// fix a genuinely missed detection caused by approach angle, since
// there's no real echo to recover in that case. That needs either a
// wider-beam sensor, multiple angled sensors, or a PIR sensor instead.

// ── MOSFET / LEDC pump ───────────────────────────────────────
void pumpON() {
  // Map 1-10 → 100-1023  (100 minimum ensures motor starts spinning)
  int pwmVal = map((int)pumpPower, 1, 10, 100, 1023);
  ledcWrite(PIN_PUMP, pwmVal);
}
void pumpOFF() {
  ledcWrite(PIN_PUMP, 0);
  pumpRunning    = false;
  pumpContinuous = false;
  pumpSource     = SRC_NONE;
}
void triggerBlocked(PumpSource src) {
  tankEmptyError = true;
  emptyErrorTime = millis();
  if (src == SRC_NONE) Serial.println("[Pump] STOPPED — tank ran dry");
  else                 Serial.printf ("[Pump] BLOCKED src=%d — empty\n", src);
}
void startPump(PumpSource src, unsigned long dur) {
  tankEmptyError = false;
  pumpRunning    = true;
  pumpContinuous = false;
  pumpSource     = src;
  pumpStartTime  = millis();
  pumpDuration   = dur;
  pumpON();
  Serial.printf("[Pump] ON  src=%d  dur=%lus  pwr=%d\n", src, dur/1000, pumpPower);
}
void startPumpForever(PumpSource src) {
  tankEmptyError = false;
  pumpRunning    = true;
  pumpContinuous = true;
  pumpSource     = src;
  pumpStartTime  = millis();
  pumpDuration   = 0;
  pumpON();
  Serial.printf("[Pump] CONTINUOUS  src=%d  pwr=%d\n", src, pumpPower);
}

// ── OLED display ─────────────────────────────────────────────
void updateOled() {
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextWrap(false);

  // Error: tank empty
  if (tankEmptyError) {
    oled.setTextSize(1);
    oled.setCursor(0,  0); oled.print(F("!! TANK EMPTY !!"));
    oled.setCursor(0, 12); oled.print(F("Please refill"));
    oled.setCursor(0, 24); oled.print(F("water reservoir"));
    oled.display(); return;
  }

  // Pump running
  if (pumpRunning) {
    const char* who = (pumpSource==SRC_TOUCH)?"Touch":(pumpSource==SRC_PIR)?"Ultra":"App";
    char row0[22];
    snprintf(row0, sizeof(row0), "%s P:%d/10", who, pumpPower);
    oled.setTextSize(1);
    oled.setCursor(0, 0); oled.print(row0);

    if (pumpContinuous) {
      unsigned long el = (millis()-pumpStartTime)/1000;
      char buf[14];
      snprintf(buf, sizeof(buf), "~%02lu:%02lu~", el/60, el%60);
      oled.setTextSize(2);
      oled.setCursor(16, 12); oled.print(buf);
    } else {
      unsigned long el  = millis()-pumpStartTime;
      unsigned long rem = (el>=pumpDuration)?0:(pumpDuration-el);
      char buf[6];
      snprintf(buf, sizeof(buf), "%02lu:%02lu", rem/1000/60, (rem/1000)%60);
      oled.setTextSize(2);
      int x=(OLED_WIDTH-(int)strlen(buf)*12)/2;
      oled.setCursor(max(0,x), 12); oled.print(buf);
    }
    oled.display(); return;
  }

  // System OFF (master switch)
  if (!systemEnabled) {
    oled.setTextSize(1);
    oled.setCursor(0,  0); oled.print(F("PooKooli Fountain"));
    oled.setCursor(0, 12); oled.print(F("SYSTEM OFF"));
    oled.setCursor(0, 24); oled.print(F("Enable in web app"));
    oled.display(); return;
  }

  // Idle
  bool water = hasWater();
  oled.setTextSize(1);
  oled.setCursor(0,  0); oled.print(F("PooKooli Fountain"));
  oled.setCursor(0, 11);
  if (WiFi.status()==WL_CONNECTED) oled.print(WiFi.localIP().toString());
  else                              oled.print(F("No WiFi"));
  oled.setCursor(0, 22);
  if (water) {
    char r[22]; snprintf(r, sizeof(r), "Water OK  Pwr:%d/10", pumpPower);
    oled.print(r);
  } else {
    oled.print(((millis()/600)%2==0) ? F("!! TANK EMPTY !!"): F("Add water please"));
  }
  oled.display();
}

// ── JSON helper ──────────────────────────────────────────────
String srcStr() {
  if (pumpSource==SRC_TOUCH) return "TOUCH";
  if (pumpSource==SRC_PIR)   return "PIR";   // kept for compat — ultrasonic source
  if (pumpSource==SRC_WIFI)  return "WIFI";
  return "NONE";
}
String buildStatusJson() {
  unsigned long el = pumpRunning?(millis()-pumpStartTime):0;
  long liveDistance = sr04Enabled ? readDistanceCmFiltered() : -1;
  String j="{";
  j+="\"running\":"      + String(pumpRunning?"true":"false") + ",";
  j+="\"continuous\":"   + String(pumpContinuous?"true":"false") + ",";
  j+="\"empty_error\":"  + String(tankEmptyError?"true":"false") + ",";
  j+="\"locked\":"       + String((pumpRunning&&pumpSource!=SRC_WIFI)?"true":"false") + ",";
  j+="\"source\":\""     + srcStr() + "\",";
  j+="\"water\":"        + String(hasWater()?"true":"false") + ",";
  j+="\"elapsed_ms\":"   + String(el) + ",";
  j+="\"duration_ms\":"  + String(pumpDuration) + ",";
  j+="\"touch_sec\":"    + String(touchDurationMs/1000) + ",";
  j+="\"pir_sec\":"      + String(pirDurationMs/1000) + ",";
  j+="\"wifi_sec\":"     + String(wifiDurationMs/1000) + ",";
  j+="\"pump_power\":"   + String(pumpPower) + ",";
  j+="\"distance_cm\":"  + String(triggerDistanceCm) + ",";
  j+="\"live_distance_cm\":" + String(liveDistance) + ",";
  j+="\"system_enabled\":" + String(systemEnabled?"true":"false") + ",";
  j+="\"sr04_enabled\":"   + String(sr04Enabled?"true":"false") + ",";
  j+="\"touch_enabled\":"  + String(touchEnabled?"true":"false");
  j+="}";
  return j;
}

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
<link rel="apple-touch-icon" href="data:image/jpeg;base64,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">
<link rel="icon" type="image/png" sizes="192x192" href="data:image/jpeg;base64,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">
<link rel="manifest" href="/manifest.json">
<style>
@import url('https://fonts.googleapis.com/css2?family=Syne:wght@400;700;800&family=DM+Sans:wght@400;500&display=swap');
*{margin:0;padding:0;box-sizing:border-box;-webkit-tap-highlight-color:transparent}
:root{
  --bg:#07090f;--card:#0f1520;--card2:#131b28;
  --accent:#38bdf8;--on:#4ade80;--off:#f87171;--warn:#fbbf24;--inf:#a78bfa;
  --text:#e8f0fe;--muted:#4a5a72;--border:#1a2540;--radius:18px;
}
body{font-family:'DM Sans',sans-serif;background:var(--bg);color:var(--text);
  min-height:100vh;display:flex;flex-direction:column;align-items:center;
  padding:0 14px 56px;
  background-image:
    radial-gradient(ellipse 70% 35% at 50% -5%,rgba(56,189,248,.13) 0%,transparent 65%),
    radial-gradient(ellipse 40% 25% at 80% 90%,rgba(74,222,128,.07) 0%,transparent 60%);
}
/* Header */
.hdr{text-align:center;margin-top:0;width:100%;max-width:380px}
.cat-banner{width:100%;height:auto;border-radius:20px 20px 0 0;display:block}
.hdr-text{background:var(--card);border:1px solid var(--border);border-top:none;
  border-radius:0 0 20px 20px;padding:14px 16px 18px}
h1{font-family:'Syne',sans-serif;font-size:1.7rem;font-weight:800;letter-spacing:-.03em;
  background:linear-gradient(120deg,#e0f2fe,#38bdf8 50%,#818cf8);
  -webkit-background-clip:text;-webkit-text-fill-color:transparent;margin:0}
.sub{font-size:.78rem;color:var(--muted);margin-top:3px}
/* Card */
.card{background:var(--card);border:1px solid var(--border);border-radius:var(--radius);
  padding:20px;width:100%;max-width:380px;margin-top:16px}
.card-title{font-family:'Syne',sans-serif;font-size:.65rem;font-weight:700;
  letter-spacing:.15em;text-transform:uppercase;color:var(--muted);margin-bottom:16px}
/* Error banner */
.error-banner{display:none;background:rgba(248,113,113,.12);border:1px solid rgba(248,113,113,.4);
  border-radius:12px;padding:14px 16px;margin-bottom:14px;align-items:center;gap:12px}
.error-banner.show{display:flex}
.error-ico{font-size:1.8rem;flex-shrink:0}
.error-title{font-family:'Syne',sans-serif;font-size:.95rem;font-weight:700;color:var(--off)}
.error-msg{font-size:.78rem;color:#fca5a5;margin-top:2px}
/* Dot */
.dot{width:12px;height:12px;border-radius:50%;flex-shrink:0;transition:all .4s}
.dot.on {background:var(--on);box-shadow:0 0 0 4px rgba(74,222,128,.2),0 0 12px rgba(74,222,128,.5)}
.dot.inf{background:var(--inf);box-shadow:0 0 0 4px rgba(167,139,250,.2),0 0 12px rgba(167,139,250,.6);animation:pulse 1.4s ease-in-out infinite}
.dot.off{background:var(--muted)}
.dot.err{background:var(--off);box-shadow:0 0 0 4px rgba(248,113,113,.2),0 0 12px rgba(248,113,113,.5)}
@keyframes pulse{0%,100%{opacity:1}50%{opacity:.4}}
.status-row{display:flex;align-items:center;gap:12px}
.s-label{font-family:'Syne',sans-serif;font-size:1rem;font-weight:700}
.s-sub{font-size:.78rem;color:var(--muted);margin-top:2px}
.s-sub.red{color:#fca5a5}
.s-sub.inf{color:var(--inf)}
/* Timer bar */
.timer-wrap{margin-top:18px;display:none}
.timer-wrap.active{display:block}
.t-row{display:flex;justify-content:space-between;font-size:.75rem;color:var(--muted);margin-bottom:7px}
.t-time{font-family:'Syne',sans-serif;font-weight:700;font-size:.85rem;color:var(--accent)}
.t-time.inf{color:var(--inf)}
.bar-bg{height:5px;background:var(--border);border-radius:99px;overflow:hidden}
.bar-fill{height:100%;width:0%;border-radius:99px;transition:width .8s linear;background:linear-gradient(90deg,var(--accent),var(--on))}
.bar-fill.inf{width:100%!important;background:linear-gradient(90deg,var(--inf),#c084fc);animation:shimmer 2s linear infinite;background-size:200% 100%}
@keyframes shimmer{0%{background-position:100% 0}100%{background-position:-100% 0}}
/* Chips */
.chips{display:flex;gap:7px;flex-wrap:wrap;margin-top:18px}
.chip{font-size:.7rem;font-weight:500;padding:4px 11px;border-radius:99px;background:var(--border);color:var(--muted);transition:all .3s;border:1px solid transparent}
.chip.active{background:rgba(56,189,248,.15);color:var(--accent);border-color:rgba(56,189,248,.3)}
.chip.water-ok{background:rgba(74,222,128,.12);color:var(--on);border-color:rgba(74,222,128,.3)}
.chip.water-no{background:rgba(248,113,113,.1);color:var(--off);border-color:rgba(248,113,113,.25)}
.chip.inf-chip{background:rgba(167,139,250,.15);color:var(--inf);border-color:rgba(167,139,250,.35)}
/* Buttons */
.btn-grid-3{display:grid;grid-template-columns:1fr 1fr 1fr;gap:8px;margin-top:16px}
.btn{display:flex;flex-direction:column;align-items:center;justify-content:center;
  gap:4px;padding:13px 6px;border:none;border-radius:14px;
  font-family:'Syne',sans-serif;font-size:.82rem;font-weight:700;
  cursor:pointer;transition:transform .12s,opacity .15s;position:relative}
.btn:active{transform:scale(.95);opacity:.8}
.btn .ico{font-size:1.35rem}
.btn-on {background:linear-gradient(145deg,#166534,#22c55e);color:#fff}
.btn-inf{background:linear-gradient(145deg,#4c1d95,#a78bfa);color:#fff}
.btn-off{background:linear-gradient(145deg,#991b1b,#ef4444);color:#fff}
.btn-restart{background:linear-gradient(145deg,#164e63,#22d3ee);color:#fff}
.btn-sub{font-size:.6rem;font-weight:400;opacity:.8;font-family:'DM Sans',sans-serif}
.btn-locked{opacity:.35;pointer-events:none;filter:grayscale(.7)}
/* Toggle switches (master / sr04 / touch) */
.toggle-row{display:flex;align-items:center;justify-content:space-between;padding:13px 0;border-bottom:1px solid var(--border)}
.toggle-row:last-child{border-bottom:none}
.toggle-label{font-size:.88rem;font-weight:500}
.toggle-label small{display:block;font-size:.72rem;color:var(--muted);margin-top:1px}
.switch{position:relative;width:50px;height:28px;flex-shrink:0}
.switch input{opacity:0;width:0;height:0;position:absolute}
.switch-track{position:absolute;inset:0;background:var(--border);border-radius:99px;
  cursor:pointer;transition:background .25s}
.switch-track::before{content:'';position:absolute;width:22px;height:22px;left:3px;top:3px;
  background:#fff;border-radius:50%;transition:transform .25s;box-shadow:0 1px 3px rgba(0,0,0,.4)}
.switch input:checked + .switch-track{background:var(--on)}
.switch input:checked + .switch-track::before{transform:translateX(22px)}
.master-card{transition:opacity .3s}
.master-card.disabled-look{opacity:.55}
.master-off-banner{display:none;background:rgba(248,113,113,.1);border:1px solid rgba(248,113,113,.3);
  border-radius:12px;padding:12px 16px;margin-bottom:14px;font-size:.82rem;color:#fca5a5;
  display:none;align-items:center;gap:10px}
.master-off-banner.show{display:flex}
/* Distance slider */
.dist-row{padding:13px 0}
.dist-row input[type=range]{width:100%;margin-top:10px;accent-color:var(--accent)}
.dist-readout{display:flex;justify-content:space-between;align-items:center}
/* Settings */
.settings-card{background:var(--card2)}
.timer-setting{display:flex;align-items:center;justify-content:space-between;padding:13px 0;border-bottom:1px solid var(--border)}
.timer-setting:last-child{border-bottom:none}
.ts-label{font-size:.88rem;font-weight:500}
.ts-label small{display:block;font-size:.72rem;color:var(--muted);margin-top:1px}
.ts-controls{display:flex;align-items:center}
.ts-btn{width:36px;height:36px;border:1px solid var(--border);background:var(--bg);
  color:var(--text);font-size:1.2rem;font-weight:700;border-radius:8px;
  cursor:pointer;display:flex;align-items:center;justify-content:center;transition:background .15s}
.ts-btn:active{background:var(--border)}
.ts-val{min-width:56px;text-align:center;font-family:'Syne',sans-serif;
  font-size:.95rem;font-weight:700;color:var(--accent);padding:0 4px;transition:color .2s}
/* ── PENDING (unsaved) indicator ── */
.ts-val.pending{color:var(--warn)!important;position:relative}
.ts-val.pending::after{content:'●';position:absolute;top:-6px;right:0;font-size:7px;color:var(--warn)}
/* Power level bar */
.power-bar{display:flex;gap:3px;margin-top:6px}
.power-pip{height:6px;flex:1;border-radius:2px;background:var(--border);transition:background .2s}
/* Save button */
.save-btn{width:100%;margin-top:16px;padding:14px;
  background:linear-gradient(135deg,#1d4ed8,#38bdf8);color:#fff;
  border:none;border-radius:12px;font-family:'Syne',sans-serif;
  font-size:.95rem;font-weight:700;cursor:pointer;
  display:flex;align-items:center;justify-content:center;gap:8px;
  transition:opacity .15s,transform .1s,background .3s}
.save-btn:active{transform:scale(.97);opacity:.85}
.save-btn.pending{background:linear-gradient(135deg,#92400e,#fbbf24)!important;animation:pulse-save 1.4s ease-in-out infinite}
@keyframes pulse-save{0%,100%{opacity:1}50%{opacity:.7}}
/* Toast */
.toast{position:fixed;bottom:24px;left:50%;transform:translateX(-50%) translateY(80px);
  background:#1e293b;color:var(--on);padding:10px 22px;border-radius:99px;
  font-size:.85rem;font-weight:600;border:1px solid rgba(74,222,128,.3);
  transition:transform .35s cubic-bezier(.34,1.56,.64,1),opacity .35s;
  opacity:0;white-space:nowrap;z-index:999}
.toast.show{transform:translateX(-50%) translateY(0);opacity:1}
.toast.err{color:var(--off);border-color:rgba(248,113,113,.4)}
.note{font-size:.7rem;color:var(--muted);margin-top:24px;text-align:center;opacity:.55;line-height:1.7}
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

<!-- MASTER SWITCH -->
<div class="card master-card" id="masterCard" style="margin-top:16px">
  <div class="toggle-row" style="border-bottom:none;padding-bottom:0">
    <div class="toggle-label">🔌 System Power<small>Master switch — turns the whole fountain on/off</small></div>
    <label class="switch">
      <input type="checkbox" id="swSystem" onchange="toggleSwitch('system',this.checked)">
      <span class="switch-track"></span>
    </label>
  </div>
</div>

<!-- STATUS -->
<div class="card" style="margin-top:12px">
  <div class="card-title">Live Status</div>
  <div class="master-off-banner" id="masterOffBanner">
    <span style="font-size:1.3rem">🔌</span>
    <span>System is OFF — the pump will not start from any sensor or the app until you turn it back on above.</span>
  </div>
  <div class="error-banner" id="errorBanner">
    <div class="error-ico">🪣</div>
    <div><div class="error-title">Water tank is empty!</div>
         <div class="error-msg">Refill the reservoir before using the pump.</div></div>
  </div>
  <div class="status-row">
    <div class="dot" id="pumpDot"></div>
    <div><div class="s-label" id="pumpLabel">Loading…</div>
         <div class="s-sub"   id="pumpSub"></div></div>
  </div>
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
    <span class="chip" id="chipPir">📡 Ultra</span>
    <span class="chip" id="chipWifi">📱 App</span>
    <span class="chip" id="chipInf">∞ Continuous</span>
  </div>
</div>

<!-- CONTROL -->
<div class="card" style="margin-top:12px">
  <div class="card-title">Control</div>
  <div class="btn-grid-3">
    <button class="btn btn-on" id="btnOn" onclick="sendCmd('on')">
      <span class="ico">▶</span>Start<span class="btn-sub" id="wifiDurLabel">2 min</span>
    </button>
    <button class="btn btn-inf" id="btnInf" onclick="sendCmd('continuous')">
      <span class="ico">∞</span>Infinite<span class="btn-sub">until stopped</span>
    </button>
    <button class="btn btn-off" onclick="sendCmd('off')">
      <span class="ico">⏹</span>Stop<span class="btn-sub">immediately</span>
    </button>
  </div>
  <button class="btn btn-restart" id="btnRestart" onclick="restartBoard()" style="margin-top:10px;width:100%">
    <span class="ico">🔄</span>Restart Board<span class="btn-sub">~5 sec downtime</span>
  </button>
</div>

<!-- SENSOR SWITCHES -->
<div class="card" style="margin-top:12px">
  <div class="card-title">Sensor Switches</div>
  <div class="toggle-row">
    <div class="toggle-label">📡 Ultrasonic Sensor<small>Distance-trigger detection</small></div>
    <label class="switch">
      <input type="checkbox" id="swSr04" onchange="toggleSwitch('sr04',this.checked)">
      <span class="switch-track"></span>
    </label>
  </div>
  <div class="toggle-row">
    <div class="toggle-label">👆 Touch Sensor<small>Manual touch-trigger</small></div>
    <label class="switch">
      <input type="checkbox" id="swTouch" onchange="toggleSwitch('touch',this.checked)">
      <span class="switch-track"></span>
    </label>
  </div>
</div>

<!-- SETTINGS -->
<div class="card settings-card" style="margin-top:12px">
  <div class="card-title">⏱ Timer &amp; Power Settings</div>

  <div class="timer-setting">
    <div class="ts-label">👆 Touch Button<small>Timed run from touch sensor</small></div>
    <div class="ts-controls">
      <button class="ts-btn" onclick="adj('touch',-10)">−</button>
      <div class="ts-val" id="val-touch">60s</div>
      <button class="ts-btn" onclick="adj('touch',+10)">+</button>
    </div>
  </div>

  <div class="timer-setting">
    <div class="ts-label">📡 Ultrasonic Sensor<small>Run time once triggered</small></div>
    <div class="ts-controls">
      <button class="ts-btn" onclick="adj('pir',-10)">−</button>
      <div class="ts-val" id="val-pir">60s</div>
      <button class="ts-btn" onclick="adj('pir',+10)">+</button>
    </div>
  </div>

  <div class="timer-setting">
    <div class="ts-label">📱 App Start<small>Timed start from this page</small></div>
    <div class="ts-controls">
      <button class="ts-btn" onclick="adj('wifi',-10)">−</button>
      <div class="ts-val" id="val-wifi">120s</div>
      <button class="ts-btn" onclick="adj('wifi',+10)">+</button>
    </div>
  </div>

  <div class="timer-setting" style="flex-direction:column;align-items:stretch;gap:10px">
    <div style="display:flex;justify-content:space-between;align-items:center">
      <div class="ts-label">⚡ Pump Power<small>Motor speed when running</small></div>
      <div style="display:flex;align-items:center;gap:0">
        <button class="ts-btn" onclick="adj('power',-1)">−</button>
        <div class="ts-val" id="val-power">8/10</div>
        <button class="ts-btn" onclick="adj('power',+1)">+</button>
      </div>
    </div>
    <div class="power-bar" id="powerBar">
      <div class="power-pip" id="pip1"></div><div class="power-pip" id="pip2"></div>
      <div class="power-pip" id="pip3"></div><div class="power-pip" id="pip4"></div>
      <div class="power-pip" id="pip5"></div><div class="power-pip" id="pip6"></div>
      <div class="power-pip" id="pip7"></div><div class="power-pip" id="pip8"></div>
      <div class="power-pip" id="pip9"></div><div class="power-pip" id="pip10"></div>
    </div>
  </div>

  <div class="dist-row">
    <div class="dist-readout">
      <div class="ts-label">📏 Trigger Distance<small>Pump starts when object is closer than this</small></div>
      <div class="ts-val" id="val-distance">9 cm</div>
    </div>
    <input type="range" id="rangeDistance" min="2" max="100" step="1" value="9"
           oninput="adjDistance(this.value)">
  </div>

  <button class="save-btn" id="saveBtn" onclick="saveSettings()">
    💾 Save Settings to Device
  </button>
</div>

<p class="note">
  Timers: ± 10 s steps &nbsp;·&nbsp; Power: ± 1 steps (1–10) &nbsp;·&nbsp; Distance: 2–100 cm<br>
  🟡 Yellow = unsaved change &nbsp;·&nbsp; tap Save to write to flash<br>
  ∞ Infinite stops via Stop button or touch sensor only<br>
  🔌 System switch overrides everything — pump stays off until it's back on
</p>
<div class="toast" id="toast"></div>

<script>
// ── State ────────────────────────────────────────────────────
var settings = { touch:60, pir:60, wifi:120, power:8, distance:9 };
var localOverride = new Set();   // keys with unsaved local changes
var MIN_SEC=5, MAX_SEC=600;
var MIN_DIST=2, MAX_DIST=100;
var pumpEndTime=0, timerTick=null, lastDurationMs=0;
var elapsedTick=null, elapsedStart=0;
var switchesInitialized=false;   // avoid clobbering a switch mid-flip from a stale poll

// ── Adjust a setting ─────────────────────────────────────────
var adj = function(key, delta) {
  localOverride.add(key);
  if (key==='power') {
    settings.power = Math.min(10, Math.max(1, settings.power+delta));
  } else {
    settings[key] = Math.min(MAX_SEC, Math.max(MIN_SEC, settings[key]+delta));
  }
  render(key);
  updateSaveBtn();
};

// ── Adjust distance via slider ─────────────────────────────────
var adjDistance = function(val) {
  localOverride.add('distance');
  settings.distance = Math.min(MAX_DIST, Math.max(MIN_DIST, parseInt(val,10)));
  render('distance');
  updateSaveBtn();
};

// ── Render a setting value ────────────────────────────────────
var render = function(key) {
  var el = document.getElementById('val-'+key);
  if (!el) return;
  var pending = localOverride.has(key);
  el.classList.toggle('pending', pending);
  if (key==='power') {
    el.textContent = settings.power+'/10';
    renderPowerBar();
  } else if (key==='distance') {
    el.textContent = settings.distance+' cm';
    var slider = document.getElementById('rangeDistance');
    if (slider && document.activeElement!==slider) slider.value = settings.distance;
  } else {
    var v = settings[key];
    var s = v>=60 ? Math.floor(v/60)+'m'+(v%60?v%60+'s':'') : v+'s';
    el.textContent = s;
    if (key==='wifi') document.getElementById('wifiDurLabel').textContent = s;
  }
};

var renderPowerBar = function() {
  var p = settings.power;
  var pending = localOverride.has('power');
  for (var i=1; i<=10; i++) {
    var pip = document.getElementById('pip'+i);
    if (!pip) continue;
    if (i<=p) {
      var c = (i<=3)?'#f87171':(i<=6)?'#fbbf24':'#4ade80';
      pip.style.background = pending ? '#fbbf24' : c;
    } else {
      pip.style.background = 'var(--border)';
    }
  }
};

var updateSaveBtn = function() {
  var btn = document.getElementById('saveBtn');
  if (localOverride.size>0) {
    btn.classList.add('pending');
    btn.textContent = '💾 Save ('+localOverride.size+' unsaved change'+(localOverride.size>1?'s':'')+')';
  } else {
    btn.classList.remove('pending');
    btn.textContent = '💾 Save Settings to Device';
  }
};

// ── Save to device ────────────────────────────────────────────
var saveSettings = async function() {
  var url = '/settings?touch='+settings.touch+'&pir='+settings.pir
          + '&wifi='+settings.wifi+'&power='+settings.power
          + '&distance='+settings.distance;
  try {
    var r = await fetch(url);
    var j = await r.json();
    if (j.ok) {
      localOverride.clear();
      ['touch','pir','wifi','power','distance'].forEach(function(k){ render(k); });
      updateSaveBtn();
      showToast('✓ All settings saved to device!');
    } else {
      showToast('⚠ Save failed — try again', true);
    }
  } catch(e) { showToast('⚠ No connection', true); }
};

// ── Send pump command ─────────────────────────────────────────
var sendCmd = async function(cmd) {
  if (cmd!=='off' && window._pumpLocked) return;
  if (cmd!=='off' && window._systemOff) return;
  try { applyStatus(await (await fetch('/pump?cmd='+cmd)).json()); }
  catch(e) {}
};

// ── Restart the board ─────────────────────────────────────────
var restartBoard = async function() {
  if (!confirm('Restart the board?\nThe fountain will be offline for ~5 seconds.')) return;
  var btn = document.getElementById('btnRestart');
  btn.disabled = true;
  btn.innerHTML = '<span class="ico">⏳</span>Restarting…<span class="btn-sub">reconnecting</span>';
  try { await fetch('/restart'); } catch(e) { /* board cuts connection mid-response — expected */ }
  // Poll /status until the board comes back online
  var waited = 0;
  var poll = setInterval(async function() {
    waited += 1;
    btn.innerHTML = '<span class="ico">⏳</span>Restarting…<span class="btn-sub">'+waited+'s</span>';
    try {
      var r = await fetch('/status');
      if (r.ok) {
        clearInterval(poll);
        btn.disabled = false;
        btn.innerHTML = '<span class="ico">🔄</span>Restart Board<span class="btn-sub">~5 sec downtime</span>';
        showToast('✓ Board is back online');
        applyStatus(await r.json());
      }
    } catch(e) { /* still rebooting */ }
    if (waited > 30) {   // give up after 30s — something went wrong
      clearInterval(poll);
      btn.disabled = false;
      btn.innerHTML = '<span class="ico">🔄</span>Restart Board<span class="btn-sub">~5 sec downtime</span>';
      showToast('⚠ Board not responding — check connection', true);
    }
  }, 1000);
};

// ── Flip a master/sensor switch ────────────────────────────────
var toggleSwitch = async function(which, checked) {
  try {
    applyStatus(await (await fetch('/toggle?which='+which+'&state='+(checked?'1':'0'))).json());
    showToast(checked ? '✓ Turned on' : '✓ Turned off');
  } catch(e) {
    showToast('⚠ No connection', true);
    // revert the checkbox visually since the request failed
    var idMap={system:'swSystem',sr04:'swSr04',touch:'swTouch'};
    var box=document.getElementById(idMap[which]);
    if (box) box.checked=!checked;
  }
};

// ── Poll status every 2s ─────────────────────────────────────
var fetchStatus = async function() {
  try { applyStatus(await (await fetch('/status')).json()); }
  catch(e) {}
};

// ── Apply status from server ──────────────────────────────────
var applyStatus = function(d) {
  // Only sync settings that have NO pending local change
  if (!localOverride.has('touch') && d.touch_sec) { settings.touch=d.touch_sec; render('touch'); }
  if (!localOverride.has('pir')   && d.pir_sec)   { settings.pir  =d.pir_sec;   render('pir');   }
  if (!localOverride.has('wifi')  && d.wifi_sec)  { settings.wifi =d.wifi_sec;  render('wifi');  }
  if (!localOverride.has('power') && d.pump_power!==undefined) {
    settings.power=d.pump_power; render('power');
  }
  if (!localOverride.has('distance') && d.distance_cm!==undefined) {
    settings.distance=d.distance_cm; render('distance');
  }

  // Sync the three on/off switches (skip while user has focus mid-drag — not
  // applicable here since these are instant toggles, so always sync)
  if (d.system_enabled!==undefined) {
    var swSys=document.getElementById('swSystem');
    if (swSys && document.activeElement!==swSys) swSys.checked=d.system_enabled;
  }
  if (d.sr04_enabled!==undefined) {
    var swS=document.getElementById('swSr04');
    if (swS && document.activeElement!==swS) swS.checked=d.sr04_enabled;
  }
  if (d.touch_enabled!==undefined) {
    var swT=document.getElementById('swTouch');
    if (swT && document.activeElement!==swT) swT.checked=d.touch_enabled;
  }

  window._systemOff = (d.system_enabled===false);
  var masterCard = document.getElementById('masterCard');
  var masterOffBanner = document.getElementById('masterOffBanner');
  if (masterCard)      masterCard.classList.toggle('disabled-look', window._systemOff);
  if (masterOffBanner) masterOffBanner.classList.toggle('show', window._systemOff);
  document.getElementById('btnOn').classList.toggle('btn-locked', window._systemOff);
  document.getElementById('btnInf').classList.toggle('btn-locked', window._systemOff);

  var src = d.source;
  ['Touch','Pir','Wifi'].forEach(function(s){
    document.getElementById('chip'+s).classList.toggle('active', src===s.toUpperCase());
  });
  document.getElementById('chipInf').className='chip'+(d.continuous?' inf-chip':'');
  var wc=document.getElementById('chipWater');
  wc.className='chip '+(d.water?'water-ok':'water-no');
  wc.textContent=d.water?'💧 Water OK':'💧 Tank Empty';

  var dot=document.getElementById('pumpDot');
  var label=document.getElementById('pumpLabel');
  var sub=document.getElementById('pumpSub');
  var wrap=document.getElementById('timerWrap');
  var bar=document.getElementById('barFill');
  var tt=document.getElementById('timerText');
  var tlabel=document.getElementById('timerLabel');
  var banner=document.getElementById('errorBanner');

  window._pumpLocked = d.locked;
  document.getElementById('btnOn').classList.toggle('btn-locked', !!d.locked || window._systemOff);
  document.getElementById('btnInf').classList.toggle('btn-locked', !!d.locked || window._systemOff);

  if (window._systemOff && !d.running) {
    dot.className='dot off'; label.textContent='System is OFF';
    sub.className='s-sub'; sub.textContent='Pump disabled — turn system back on above';
    banner.classList.remove('show');
    stopTicks(); resetTimerUI(); return;
  }

  if (d.empty_error) {
    banner.classList.add('show');
    dot.className='dot err'; label.textContent='Water tank is empty!';
    sub.className='s-sub red'; sub.textContent='Refill the reservoir';
    stopTicks(); resetTimerUI(); return;
  }
  banner.classList.remove('show');

  if (d.running && d.continuous) {
    dot.className='dot inf'; label.textContent='Running continuously';
    sub.className='s-sub inf';
    var map2={TOUCH:'Touch sensor',PIR:'Ultrasonic sensor',WIFI:'App — infinite'};
    sub.textContent=(map2[src]||src)+' · Stop via app or touch';
    wrap.classList.add('active'); tlabel.textContent='Elapsed';
    bar.className='bar-fill inf'; tt.className='t-time inf';
    elapsedStart=Date.now()-d.elapsed_ms; stopTimerTick();
    if (!elapsedTick) elapsedTick=setInterval(function(){tt.textContent=fmtMs(Date.now()-elapsedStart);},500);
    return;
  }

  if (d.running) {
    dot.className='dot on'; label.textContent='Pump is running';
    sub.className='s-sub';
    var map3={TOUCH:'Touch sensor',PIR:'Ultrasonic sensor',WIFI:'App control'};
    sub.textContent='Triggered by: '+(map3[src]||src)+' · Power: '+d.pump_power+'/10';
    wrap.classList.add('active'); tlabel.textContent='Pump running';
    bar.className='bar-fill'; tt.className='t-time';
    lastDurationMs=d.duration_ms;
    var rem=Math.max(0,d.duration_ms-d.elapsed_ms);
    pumpEndTime=Date.now()+rem;
    bar.style.width=Math.min(100,d.elapsed_ms/d.duration_ms*100)+'%';
    tt.textContent=fmtMs(rem); stopElapsedTick();
    if (!timerTick) timerTick=setInterval(function(){
      var r2=Math.max(0,pumpEndTime-Date.now());
      bar.style.width=Math.min(100,(lastDurationMs-r2)/lastDurationMs*100)+'%';
      tt.textContent=fmtMs(r2);
      if (r2<=0) stopTimerTick();
    },400);
    return;
  }

  window._pumpLocked=false;
  dot.className='dot off'; label.textContent='Pump is OFF';
  sub.className='s-sub'; sub.textContent='Waiting for trigger…';
  stopTicks(); resetTimerUI();
};

var stopTimerTick=function(){ if(timerTick){clearInterval(timerTick);timerTick=null;} };
var stopElapsedTick=function(){ if(elapsedTick){clearInterval(elapsedTick);elapsedTick=null;} };
var stopTicks=function(){ stopTimerTick(); stopElapsedTick(); };
var resetTimerUI=function(){
  var wrap=document.getElementById('timerWrap');
  var bar=document.getElementById('barFill');
  var tt=document.getElementById('timerText');
  wrap.classList.remove('active');
  bar.className='bar-fill'; bar.style.width='0%';
  tt.className='t-time'; tt.textContent='0:00';
};

var fmtMs=function(ms){
  var s=Math.ceil(ms/1000), m=Math.floor(s/60);
  return m+':'+String(s%60).padStart(2,'0');
};
var showToast=function(msg,isErr){
  var t=document.getElementById('toast');
  t.textContent=msg;
  t.className='toast show'+(isErr?' err':'');
  setTimeout(function(){t.classList.remove('show');},3000);
};

fetchStatus();
setInterval(fetchStatus, 2000);
</script>
</body>
</html>
)rawliteral";

const char MANIFEST_JSON[] PROGMEM = R"manifest({"name":"PooKooli Fountain","short_name":"PooKooli","start_url":"/","display":"standalone","background_color":"#07090f","theme_color":"#07090f","icons":[{"src":"data:image/jpeg;base64,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","sizes":"192x192","type":"image/jpeg"},{"src":"data:image/jpeg;base64,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","sizes":"180x180","type":"image/jpeg"}]})manifest";
// ── Web handlers ─────────────────────────────────────────────
void handleRoot()     { server.send_P(200,"text/html",INDEX_HTML); }
void handleManifest() { server.send_P(200,"application/manifest+json",MANIFEST_JSON); }
void handleStatus()   { server.send(200,"application/json",buildStatusJson()); }

void handlePumpCmd() {
  if (!systemEnabled) {
    // Master switch is OFF — pump must never start, no matter the command.
    pumpOFF();
    server.send(200,"application/json",buildStatusJson());
    return;
  }
  if (!server.hasArg("cmd")) { server.send(400,"text/plain","Missing cmd"); return; }
  String cmd = server.arg("cmd");
  if (cmd=="on") {
    if (pumpRunning && pumpSource!=SRC_WIFI) { server.send(200,"application/json",buildStatusJson()); return; }
    if (!hasWater()) triggerBlocked(SRC_WIFI);
    else if (pumpRunning && pumpSource==SRC_WIFI && !pumpContinuous) pumpStartTime=millis();
    else startPump(SRC_WIFI, wifiDurationMs);
  } else if (cmd=="continuous") {
    if (pumpRunning && pumpSource!=SRC_WIFI) { server.send(200,"application/json",buildStatusJson()); return; }
    if (!hasWater()) triggerBlocked(SRC_WIFI);
    else startPumpForever(SRC_WIFI);
  } else if (cmd=="off") {
    pumpOFF(); tankEmptyError=false;
  }
  server.send(200,"application/json",buildStatusJson());
}

void handleSettings() {
  bool changed=false;
  if (server.hasArg("touch")) { long v=server.arg("touch").toInt(); if(v>=MIN_SEC&&v<=MAX_SEC){touchDurationMs=v*1000UL;changed=true;} }
  if (server.hasArg("pir"))   { long v=server.arg("pir").toInt();   if(v>=MIN_SEC&&v<=MAX_SEC){pirDurationMs  =v*1000UL;changed=true;} }
  if (server.hasArg("wifi"))  { long v=server.arg("wifi").toInt();  if(v>=MIN_SEC&&v<=MAX_SEC){wifiDurationMs =v*1000UL;changed=true;} }
  if (server.hasArg("power")) { int  v=server.arg("power").toInt(); if(v>=1&&v<=10){pumpPower=(uint8_t)v;     changed=true;} }
  if (server.hasArg("distance")) { int v=server.arg("distance").toInt(); if(v>=MIN_DISTANCE_CM&&v<=MAX_DISTANCE_CM){triggerDistanceCm=(uint8_t)v; changed=true;} }
  if (changed) saveSettings();
  String json = "{\"ok\":true,\"touch_sec\":" + String(touchDurationMs/1000)
              + ",\"pir_sec\":"               + String(pirDurationMs/1000)
              + ",\"wifi_sec\":"              + String(wifiDurationMs/1000)
              + ",\"pump_power\":"            + String(pumpPower)
              + ",\"distance_cm\":"           + String(triggerDistanceCm) + "}";
  server.send(200,"application/json",json);
}

// Toggle one of the three on/off switches: system | sr04 | touch
void handleToggle() {
  if (!server.hasArg("which") || !server.hasArg("state")) {
    server.send(400,"text/plain","Missing which/state"); return;
  }
  String which = server.arg("which");
  bool   state = (server.arg("state")=="1" || server.arg("state")=="true");

  if (which=="system") {
    systemEnabled = state;
    if (!systemEnabled) { pumpOFF(); tankEmptyError=false; }  // kill pump immediately
  } else if (which=="sr04") {
    sr04Enabled = state;
  } else if (which=="touch") {
    touchEnabled = state;
  } else {
    server.send(400,"text/plain","Unknown 'which'"); return;
  }
  saveSettings();
  server.send(200,"application/json",buildStatusJson());
}

// Restart handler — stops the pump first (safety), sends the HTTP
// response, then waits a moment for the response to flush before
// calling ESP.restart() so the board doesn't cut the connection
// before the browser gets the 200 OK.
void handleRestart() {
  pumpOFF();   // safety — never restart with the pump running
  server.send(200, "application/json", "{\"restarting\":true}");
  delay(300);  // give the TCP stack time to flush the response
  Serial.println("[System] Restart requested via web app");
  ESP.restart();
}

// ── Setup ────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== PooKooli Fountain v7 (ESP32-DevKitC) ===");

  // LEDC PWM — new API (ESP32 core v3.x): pin-based, no channel needed
  ledcAttach(PIN_PUMP, LEDC_FREQ_HZ, LEDC_RESOLUTION);
  pumpOFF();   // ensure pump starts OFF — this runs on EVERY boot, including
               // a watchdog-triggered reset, so a freeze/reset never leaves
               // the pump stuck running

  // Input pins
  pinMode(PIN_TOUCH,    INPUT);
  pinMode(PIN_SR04_TRIG, OUTPUT);
  pinMode(PIN_SR04_ECHO, INPUT);
  digitalWrite(PIN_SR04_TRIG, LOW);
  pinMode(PIN_WATER_DO, INPUT_PULLUP);

  loadSettings();

  // OLED
  Wire.begin(21, 22);
  if (!oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("[OLED] Not found");
  } else {
    oled.clearDisplay();
    oled.setTextColor(SSD1306_WHITE);
    oled.setTextSize(1);
    oled.setCursor(4,  8); oled.print(F("PooKooli Fountain"));
    oled.setCursor(28,20); oled.print(F("Starting..."));
    oled.display();
    Serial.println("[OLED] OK");
  }

  // WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.printf("Connecting to %s", WIFI_SSID);
  int tries=0;
  while (WiFi.status()!=WL_CONNECTED && tries<30) { delay(500); Serial.print("."); tries++; }
  if (WiFi.status()==WL_CONNECTED)
    Serial.printf("\n✓ http://%s\n", WiFi.localIP().toString().c_str());
  else
    Serial.println("\n✗ WiFi failed — touch+ultrasonic still work");

  server.on("/",              handleRoot);
  server.on("/manifest.json", handleManifest);
  server.on("/status",        handleStatus);
  server.on("/pump",          handlePumpCmd);
  server.on("/settings",      handleSettings);
  server.on("/toggle",        handleToggle);
  server.on("/restart",       handleRestart);
  server.begin();
  Serial.println("Web server ready.");

  // Hardware watchdog: if loop() doesn't "feed" it within WDT_TIMEOUT_SEC,
  // the ESP32 force-resets itself. This recovers from genuine hangs (a
  // sensor read that never returns, a stuck WiFi stack, etc.) — it does
  // NOT run on a timer and does NOT mask the ultrasonic beam-angle issue,
  // it only fires if the firmware actually stops responding.
  //
  // NOTE — API depends on your ESP32 Arduino core version:
  //   Core 3.x (current): esp_task_wdt_init(esp_task_wdt_config_t*) — used below
  //   Core 2.x (older):    esp_task_wdt_init(uint32_t timeout_s, bool panic)
  // If this fails to compile, check Tools > Board Manager for your
  // installed "esp32" core version and swap to the matching call.
  esp_task_wdt_config_t wdtConfig = {
    .timeout_ms = WDT_TIMEOUT_SEC * 1000,
    .idle_core_mask = 0,   // don't watch idle tasks, only our own feed below
    .trigger_panic = true  // true reset, not just a warning
  };
  esp_task_wdt_init(&wdtConfig);
  esp_task_wdt_add(NULL);  // register the current (loop) task
  Serial.printf("[WDT] Hardware watchdog armed (%ds timeout)\n", WDT_TIMEOUT_SEC);
}

// ── Loop ─────────────────────────────────────────────────────
void loop() {
  esp_task_wdt_reset();   // "feed" the watchdog — confirms loop() is alive
  server.handleClient();
  unsigned long now = millis();

  // Master switch: if the whole mechanism is OFF, the pump can never run.
  // Kill it immediately (covers the case where it was just toggled off
  // mid-run) and skip every trigger check below.
  if (!systemEnabled) {
    if (pumpRunning) { Serial.println("[System] Master OFF — stopping pump"); pumpOFF(); }
    touchCandidate = false;
    sr04Candidate  = false;
    if (now-lastOledUpdate >= 300) { lastOledUpdate=now; updateOled(); }
    delay(50); return;
  }

  // Auto-clear empty error
  if (tankEmptyError && (now-emptyErrorTime > EMPTY_BLINK_MS)) tankEmptyError=false;

  // Auto-stop when timer expires (skip in continuous mode)
  if (pumpRunning && !pumpContinuous && (now-pumpStartTime >= pumpDuration)) {
    Serial.println("[Pump] Timer done");
    if (pumpSource==SRC_PIR) lastIrEndTime=now;
    pumpOFF();
  }

  // Stop continuous when tank runs dry
  if (pumpRunning && pumpContinuous && !hasWater()) {
    Serial.println("[Pump] Continuous — tank empty, stopping");
    pumpOFF(); triggerBlocked(SRC_NONE);
  }

  // Update OLED every 300 ms
  if (now-lastOledUpdate >= 300) { lastOledUpdate=now; updateOled(); }

  // Touch sensor — always allowed (stops pump OR starts new run),
  // unless the touch switch itself is OFF (touchStable() returns false then)
  if (touchStable() && (now-lastTouchTime > TOUCH_DEBOUNCE_MS)) {
    lastTouchTime=now;
    if (pumpRunning) {
      Serial.println("[Touch] Stopping pump");
      if (pumpSource==SRC_PIR) lastIrEndTime=now;
      pumpOFF();
    } else {
      Serial.println("[Touch] Triggered");
      if (hasWater()) startPump(SRC_TOUCH, touchDurationMs);
      else            triggerBlocked(SRC_TOUCH);
    }
    delay(50); return;
  }

  // Lock all other triggers while pump is on
  if (pumpRunning) { delay(50); return; }

  // Ultrasonic sensor (HC-SR04) — unless the sensor switch itself is OFF
  // (sr04Stable() returns false then). Triggers when an object is closer
  // than triggerDistanceCm.
  if (sr04Stable() && (now-lastIrEndTime > SR04_LOCKOUT_MS)) {
    Serial.printf("[Ultra] Object within %dcm\n", triggerDistanceCm);
    if (hasWater()) startPump(SRC_PIR, pirDurationMs);
    else            triggerBlocked(SRC_PIR);
  }

  delay(50);
}
