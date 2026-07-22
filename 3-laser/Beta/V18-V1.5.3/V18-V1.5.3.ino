/*
 ╔══════════════════════════════════════════════════════════╗
 ║      PooKooli Fountain BETA V1.5  (ESP32-DevKitC)        ║
 ║                                                          ║
 ║  VERSIONING:                                             ║
 ║   BETA Vx.y  — test track: +0.1 per module added          ║
 ║   X    Vx.y  — complete edition: +0.1 small, +1.0 big     ║
 ║                                                          ║
 ║   Modules included in this build, ONLY:                  ║
 ║    ✓ 2× VL53L0X laser sensors (Adafruit lib, XSHUT,       ║
 ║      addr 0x30/0x31 — matches user-verified reference)   ║
 ║      Distance range: 5–70 cm on both sensors               ║
 ║    ✓ Water sensor ATTACHED — real ADC readings             ║
 ║    ✓ MOSFET pump (LEDC PWM, 10 power levels)              ║
 ║    ✓ Web app: status, contrrol, sensor switches, settings ║
 ║    ✓ DEBUG: boot connectivity check + live distances     ║
 ║      shown on the web app AND Serial monitor              ║
 ║    ✓ TEST: water test-mode switch — forces level to 50%  ║
 ║      (optional now — real sensor is used unless toggled)  ║
 ║    ✓ Hardware watchdog + NVS-saved settings               ║
 ║    ✓ Laser-triggered pump renews its timer while the cat  ║
 ║      stays in range, instead of stopping mid-drink         ║
 ║    ✓ 0.96" OLED (SSD1306) — status + live cm readout      ║
 ║    ✓ Touch sensor (capacitive, active-HIGH) — universal   ║
 ║      manual override: stops any run, or starts a timed    ║
 ║      touch-triggered run                                  ║
 ║    ✓ Analog water level sensor — real readings, test mode ║
 ║      now optional/user-toggled instead of locked ON       ║
 ║    ✓ 433MHz remote (YK04 + PT2272-M4 momentary decoder)   ║
 ║      A=timed start/stop  B=infinite start/stop             ║
 ║      C=manual I2C recovery  D=restart (C/D work even       ║
 ║      with the system master switch off)                   ║
 ║                                                          ║
 ║   NOT included yet — this build has every planned module  ║
 ║    ✗ Touch / push button                                 ║
 ╚══════════════════════════════════════════════════════════╝

 ── LIBRARIES  (Arduino Library Manager) ────────────────────
   • Adafruit VL53L0X       — search "VL53L0X" by Adafruit
     (pulls in Adafruit BusIO as a dependency)
   • Adafruit SSD1306        (by Adafruit)
   • Adafruit GFX            (by Adafruit)
   Board: "ESP32 Dev Module"  by Espressif

 ── WIRING ───────────────────────────────────────────────────
  VL53L0X sensor #1:  VCC→3.3V  GND→GND  SDA→GPIO21  SCL→GPIO22
                       XSHUT→GPIO32  (reassigned to addr 0x30 at boot)
  VL53L0X sensor #2:  VCC→3.3V  GND→GND  SDA→GPIO21  SCL→GPIO22
                       XSHUT→GPIO33  (reassigned to addr 0x31 at boot)
  Water level sensor:  GND→GND  +→3.3V  S→GPIO34  (ADC1 ch6)
                       Calibrate WATER_ADC_DRY/WATER_ADC_FULL below to
                       your actual sensor before trusting the % reading
  OLED 0.96" SSD1306:  VCC→3.3V  GND→GND  SDA→GPIO21  SCL→GPIO22
                       (shares I2C bus with both sensors — no extra pins)
  Touch sensor:        VCC→3.3V  GND→GND  SIG→GPIO13
                       (active-HIGH output — no pull-up needed)
  433MHz remote (YK04): VCC→5V  GND→GND
                       A→GPIO18  B→GPIO16  C→GPIO19  D→GPIO17
                       (confirmed by test sketch — this module's D0-D3
                       physical order didn't match the datasheet-typical
                       left-to-right assumption, so match YOUR module's
                       actual wiring, not the label silkscreened on it)
                       (VT pin exists on the module but isn't wired —
                       not needed since we read each button directly)
  MOSFET module:       Control GND→GND     PWM→GPIO26
 ─────────────────────────────────────────────────────────────
*/

#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <Wire.h>
#include "Adafruit_VL53L0X.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <esp_task_wdt.h>

// ── WiFi ─────────────────────────────────────────────────────
const char* WIFI_SSID     = "HiMo 2G";
const char* WIFI_PASSWORD = "@HiMo9226#";

// ── GPIO Pins ────────────────────────────────────────────────
#define PIN_XSHUT1      32    // VL53L0X #1 XSHUT (reassigned to 0x30)
#define PIN_XSHUT2      33    // VL53L0X #2 XSHUT (reassigned to 0x31)

// ── Build configuration — attach/detach modules (0 = detached) ─
#define SENSOR2_ATTACHED   1   // laser sensor #2 now wired in
#define WATER_ATTACHED     1   // water sensor now wired in
#define OLED_ATTACHED      1   // 0.96" SSD1306 now wired in
#define PIN_WATER_ADC   34    // Analog water level (ADC1 ch6, input-only)
#define PIN_PUMP        26    // MOSFET PWM
#define PIN_TOUCH       13    // Touch sensor SIG (active-HIGH, driven output)

// ── 433MHz remote (YK04 + PT2272-M4 decoder) ───────────────────
// Momentary decoder: each Dx pin goes HIGH only while that button is
// physically held, LOW when released. VT pin exists on the module but
// isn't wired — not needed since we read each button pin directly.
#define RF_ATTACHED        1   // 433MHz remote now wired in
#define PIN_RF_A           18  // start/stop (timed run) — corrected: this
                                // physical pin lights up when remote
                                // button A is pressed (confirmed by test)
#define PIN_RF_B           16  // infinite start/stop — corrected
#define PIN_RF_C           19  // manual I2C bus recovery — corrected
#define PIN_RF_D           17  // restart board — corrected
#define RF_HOLD_MS       3000  // button must be held HIGH continuously
                                // for this long before an action fires.
                                // A real press easily holds this; a brief
                                // noise glitch on the D-pin (motor EMI,
                                // no antenna yet) can't sustain it — this
                                // is what actually filters out random/
                                // spurious triggers, not a short debounce.

// ── VL53L0X I²C addresses ────────────────────────────────────
#define VL53_ADDR_1   0x30
#define VL53_ADDR_2   0x31   // real, non-default address — was 0x29 (factory
                             // default) before; that left a bus-collision
                             // window with sensor #1 during boot, which
                             // caused the random detect/error failures

// ── LEDC PWM ─────────────────────────────────────────────────
// ── OLED ─────────────────────────────────────────────────────
#define OLED_WIDTH   128
#define OLED_HEIGHT   32
#define OLED_ADDR    0x3C
#define OLED_RESET     -1
#define OLED_REFRESH_MS 300

#define LEDC_FREQ_HZ    1000
#define LEDC_RESOLUTION   10   // 10-bit: 0–1023

// ── Hardware watchdog ─────────────────────────────────────────
#define WDT_TIMEOUT_SEC 10

// ── Defaults ─────────────────────────────────────────────────
#define DEFAULT_PIR_SEC      60   // pump run time when a laser triggers
#define DEFAULT_WIFI_SEC    120   // pump run time when started from web app
#define DEFAULT_TOUCH_SEC     60   // pump run time when touch sensor triggers
#define DEFAULT_REMOTE_SEC    60   // pump run time when remote button A triggers
#define DEFAULT_PUMP_POWER    8
#define DEFAULT_DISTANCE1_CM 15
#define DEFAULT_DISTANCE2_CM 15
#define MIN_SEC               5
#define MAX_SEC             600
#define MIN_DISTANCE_CM       5
#define MAX_DISTANCE_CM      70

// ── Water level ADC calibration ──────────────────────────────
// Adjust to match your sensor's dry/fully-submerged readings.
#define WATER_ADC_DRY    400
#define WATER_ADC_FULL  3000
#define WATER_LOW_PCT     20

// ── Timing constants ─────────────────────────────────────────
#define LASER_LOCKOUT_MS    5000
#define LASER_CONFIRM_MS     150
#define MAX_LASER_RUNTIME_MS (5UL*60UL*1000UL)  // 5 min hard cap — the pump
                                                 // can NEVER run longer than
                                                 // this on laser triggers,
                                                 // no matter how many times
                                                 // the timer renews. Backstop
                                                 // against a stuck-sensor
                                                 // reading falsely showing
                                                 // "cat still present" forever.
#define TOUCH_CONFIRM_MS      60   // clean driven output — no floating-pin noise
#define TOUCH_DEBOUNCE_MS    500   // cooldown before a new touch is accepted
#define EMPTY_BLINK_MS      3000
#define WATER_SAMPLE_MS     2000
#define DIST_SAMPLE_MS       120   // refresh cached laser distances
#define DIST_PRINT_MS       2000   // Serial distance report interval

// ── Globals ──────────────────────────────────────────────────
WebServer         server(80);
Adafruit_SSD1306  oled(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RESET);
unsigned long     lastOledUpdate = 0;
Preferences       prefs;
Adafruit_VL53L0X  sensor1 = Adafruit_VL53L0X();
Adafruit_VL53L0X  sensor2 = Adafruit_VL53L0X();
VL53L0X_RangingMeasurementData_t measure1, measure2;
bool              tof1Ok = false, tof2Ok = false;

unsigned long pirDurationMs    = DEFAULT_PIR_SEC    * 1000UL;
unsigned long wifiDurationMs   = DEFAULT_WIFI_SEC   * 1000UL;
unsigned long touchDurationMs  = DEFAULT_TOUCH_SEC   * 1000UL;
unsigned long remoteDurationMs = DEFAULT_REMOTE_SEC  * 1000UL;
uint8_t       pumpPower        = DEFAULT_PUMP_POWER;
uint8_t       triggerDistance1Cm = DEFAULT_DISTANCE1_CM;
uint8_t       triggerDistance2Cm = DEFAULT_DISTANCE2_CM;

bool          systemEnabled    = true;
bool          touchEnabled     = true;
bool          rfEnabled        = true;
bool          laser1Enabled    = true;
bool          laser2Enabled    = true;

enum PumpSource { SRC_NONE, SRC_PIR, SRC_WIFI, SRC_TOUCH, SRC_REMOTE };

bool          pumpRunning    = false;
bool          pumpContinuous = false;
PumpSource    pumpSource     = SRC_NONE;
unsigned long pumpStartTime  = 0;
unsigned long pumpDuration   = 0;
bool          tankLowError   = false;
unsigned long emptyErrorTime = 0;

int           waterPct       = 100;
unsigned long lastWaterRead  = 0;

// Test / debug
bool          testMode       = !WATER_ATTACHED; // water detached: defaults ON (50%)
long          lastDist1Cm    = -1;     // latest cached readings (cm), -1 = none
long          lastDist2Cm    = -1;
unsigned long lastDistRead   = 0;
unsigned long lastDistPrint  = 0;

bool          laserCandidate      = false;
unsigned long laserCandidateSince = 0;
unsigned long lastLaserEndTime    = 0;
unsigned long laserFirstTriggerTime = 0;  // set on the FIRST trigger of a
                                           // renewal chain, never touched by
                                           // renewals themselves — used to
                                           // enforce MAX_LASER_RUNTIME_MS

// Touch debounce state
bool          touchCandidate      = false;
unsigned long touchCandidateSince = 0;
unsigned long lastTouchTime       = 0;

// RF remote edge-detection state — one entry per button (A,B,C,D)
const int     rfPins[4]        = { PIN_RF_A, PIN_RF_B, PIN_RF_C, PIN_RF_D };
bool          rfCandidate[4]   = { false, false, false, false };
unsigned long rfCandidateSince[4] = { 0, 0, 0, 0 };
bool          rfFired[4]       = { false, false, false, false };

// ── NVS ──────────────────────────────────────────────────────
void loadSettings() {
  prefs.begin("fountain", true);
  pirDurationMs      = prefs.getULong("pir_ms",   DEFAULT_PIR_SEC    * 1000UL);
  wifiDurationMs     = prefs.getULong("wifi_ms",  DEFAULT_WIFI_SEC   * 1000UL);
  touchDurationMs    = prefs.getULong("touch_ms", DEFAULT_TOUCH_SEC  * 1000UL);
  remoteDurationMs   = prefs.getULong("remote_ms", DEFAULT_REMOTE_SEC * 1000UL);
  pumpPower          = prefs.getUChar("pump_pwr", DEFAULT_PUMP_POWER);
  if (pumpPower < 1 || pumpPower > 10) pumpPower = DEFAULT_PUMP_POWER;
  triggerDistance1Cm = prefs.getUChar("dist1_cm", DEFAULT_DISTANCE1_CM);
  triggerDistance2Cm = prefs.getUChar("dist2_cm", DEFAULT_DISTANCE2_CM);
  if (triggerDistance1Cm < MIN_DISTANCE_CM || triggerDistance1Cm > MAX_DISTANCE_CM)
    triggerDistance1Cm = DEFAULT_DISTANCE1_CM;
  if (triggerDistance2Cm < MIN_DISTANCE_CM || triggerDistance2Cm > MAX_DISTANCE_CM)
    triggerDistance2Cm = DEFAULT_DISTANCE2_CM;
  systemEnabled = prefs.getBool("sys_en",    true);
  laser1Enabled = prefs.getBool("laser1_en", true);
  laser2Enabled = prefs.getBool("laser2_en", true);
  touchEnabled  = prefs.getBool("touch_en",  true);
  rfEnabled     = prefs.getBool("rf_en",     true);
  prefs.end();
  Serial.printf("[NVS] pir=%lus wifi=%lus touch=%lus pwr=%d d1=%dcm d2=%dcm sys=%d l1=%d l2=%d touch_en=%d\n",
    pirDurationMs/1000, wifiDurationMs/1000, touchDurationMs/1000, pumpPower,
    triggerDistance1Cm, triggerDistance2Cm, systemEnabled, laser1Enabled, laser2Enabled, touchEnabled);
}
void saveSettings() {
  prefs.begin("fountain", false);
  prefs.putULong("pir_ms",   pirDurationMs);
  prefs.putULong("wifi_ms",  wifiDurationMs);
  prefs.putULong("touch_ms", touchDurationMs);
  prefs.putULong("remote_ms", remoteDurationMs);
  prefs.putUChar("pump_pwr", pumpPower);
  prefs.putUChar("dist1_cm", triggerDistance1Cm);
  prefs.putUChar("dist2_cm", triggerDistance2Cm);
  prefs.putBool("sys_en",    systemEnabled);
  prefs.putBool("laser1_en", laser1Enabled);
  prefs.putBool("laser2_en", laser2Enabled);
  prefs.putBool("touch_en",  touchEnabled);
  prefs.putBool("rf_en",     rfEnabled);
  prefs.end();
  Serial.println("[NVS] Saved");
}

// ── Water level ──────────────────────────────────────────────
void readWaterLevel() {
  if (!WATER_ATTACHED) {          // sensor not wired: locked at 50%, no ADC read
    waterPct = 50;
    return;
  }
  if (testMode) {                 // TEST MODE: pretend the tank is half full
    waterPct = 50;
    Serial.println("[Water] TEST MODE — forced to 50%");
    return;
  }
  int raw = analogRead(PIN_WATER_ADC);
  raw = constrain(raw, WATER_ADC_DRY, WATER_ADC_FULL);
  waterPct = map(raw, WATER_ADC_DRY, WATER_ADC_FULL, 0, 100);
  Serial.printf("[Water] raw=%d  pct=%d%%\n", raw, waterPct);
}
bool hasWater() { return waterPct >= WATER_LOW_PCT; }

// ── Distance sampling ─────────────────────────────────────────
// One cached read per sensor, refreshed every DIST_SAMPLE_MS in loop().
// Used by the trigger logic, the web app JSON, and the Serial report —
// so the sensor is read in exactly ONE place.
// Uses rangingTest() — a single on-demand ranging call, matching the
// Adafruit_VL53L0X reference sketch that tested reliable on this hardware.
long readOneCm(Adafruit_VL53L0X &sensor, VL53L0X_RangingMeasurementData_t &measure, bool okFlag) {
  if (!okFlag) return -1;
  sensor.rangingTest(&measure, false);
  if (measure.RangeStatus == 4) return -1;   // 4 = out of range / invalid
  return (long)(measure.RangeMilliMeter / 10);
}
// Lightweight connectivity check — verifies the sensor still ACKs its
// I2C address, independent of whatever rangingTest() last returned.
// This matters because rangingTest() doesn't report success/failure:
// if a transaction gets corrupted by motor/PWM noise, the measurement
// struct can be left holding stale data from the last GOOD read —
// which is exactly how a sensor can appear "stuck" reporting the same
// in-range distance forever, or stop updating entirely.
bool i2cPing(uint8_t addr) {
  Wire.beginTransmission(addr);
  return (Wire.endTransmission() == 0);
}

void readDistances() {
  bool sensor1Lost = tof1Ok && !i2cPing(VL53_ADDR_1);
  bool sensor2Lost = tof2Ok && SENSOR2_ATTACHED && !i2cPing(VL53_ADDR_2);
  if (sensor1Lost) { Serial.println("[VL53L0X] Sensor #1 stopped ACKing — will attempt recovery"); tof1Ok = false; }
  if (sensor2Lost) { Serial.println("[VL53L0X] Sensor #2 stopped ACKing — will attempt recovery"); tof2Ok = false; }

  lastDist1Cm = readOneCm(sensor1, measure1, tof1Ok);
  lastDist2Cm = readOneCm(sensor2, measure2, tof2Ok);

  // Self-heal: a dropped-out sensor gets the same full bus recovery +
  // re-init as the web app's manual button, automatically. Throttled
  // so a persistent noise source doesn't hammer retries constantly.
  static unsigned long lastAutoRecover = 0;
  bool needRecover = (!tof1Ok) || (SENSOR2_ATTACHED && !tof2Ok);
  if (needRecover && (millis() - lastAutoRecover > 3000)) {
    lastAutoRecover = millis();
    Serial.println("[Recovery] Auto-recovering I2C bus after sensor dropout");
    Wire.end();
    recoverI2CBus(21, 22);
    Wire.begin(21, 22);
    Wire.setClock(100000);
    delay(50);
    initI2CDevices();
  }
}

// ── Debounced dual-laser check (uses cached distances) ───────
bool laserStable() {
  if (!laser1Enabled && !laser2Enabled) { laserCandidate = false; return false; }
  bool raw = false;
  if (laser1Enabled && lastDist1Cm > 0 && lastDist1Cm <= (long)triggerDistance1Cm) raw = true;
  if (!raw && laser2Enabled && lastDist2Cm > 0 && lastDist2Cm <= (long)triggerDistance2Cm) raw = true;
  if (raw != laserCandidate) {
    laserCandidate      = raw;
    laserCandidateSince = millis();
  }
  return raw && (millis() - laserCandidateSince >= LASER_CONFIRM_MS);
}

// ── Instant presence check (no debounce) — used to decide whether to
// renew a running laser-triggered pump timer instead of stopping it.
// laserStable()'s debounce state is only updated while NOT pumpRunning,
// so it can't be reused here; this reads the same cached distances
// directly against each sensor's own threshold.
bool catStillPresent() {
  if (!laser1Enabled && !laser2Enabled) return false;
  if (laser1Enabled && lastDist1Cm > 0 && lastDist1Cm <= (long)triggerDistance1Cm) return true;
  if (laser2Enabled && lastDist2Cm > 0 && lastDist2Cm <= (long)triggerDistance2Cm) return true;
  return false;
}

// ── Debounced touch check ─────────────────────────────────────
// Touch module drives SIG actively (push-pull, not floating), so a
// short confirm window is enough — no noise-flicker risk like an
// unconnected mechanical pin would have.
bool touchStable() {
  if (!touchEnabled) { touchCandidate = false; return false; }
  bool raw = (digitalRead(PIN_TOUCH) == HIGH);
  if (raw != touchCandidate) {
    touchCandidate      = raw;
    touchCandidateSince = millis();
  }
  return raw && (millis() - touchCandidateSince >= TOUCH_CONFIRM_MS);
}

// ── Pump control ─────────────────────────────────────────────
void pumpON() {
  int pwmVal = map((int)pumpPower, 1, 10, 100, 1023);
  ledcWrite(PIN_PUMP, pwmVal);
}
void pumpOFF() {
  ledcWrite(PIN_PUMP, 0);
  pumpRunning = false; pumpContinuous = false; pumpSource = SRC_NONE;
}
void triggerBlocked(PumpSource src) {
  tankLowError = true; emptyErrorTime = millis();
  Serial.printf("[Pump] BLOCKED src=%d water=%d%%\n", src, waterPct);
}
void startPump(PumpSource src, unsigned long dur) {
  tankLowError = false; pumpRunning = true; pumpContinuous = false;
  pumpSource = src; pumpStartTime = millis(); pumpDuration = dur;
  pumpON();
  Serial.printf("[Pump] ON src=%d dur=%lus pwr=%d\n", src, dur/1000, pumpPower);
}
void startPumpForever(PumpSource src) {
  tankLowError = false; pumpRunning = true; pumpContinuous = true;
  pumpSource = src; pumpStartTime = millis(); pumpDuration = 0;
  pumpON();
  Serial.printf("[Pump] CONTINUOUS src=%d pwr=%d\n", src, pumpPower);
}

// ── OLED display ────────────────────────────────────────────
void drawBar(int x, int y, int w, int h, int pct) {
  oled.drawRect(x, y, w, h, SSD1306_WHITE);
  int fill = (w - 2) * constrain(pct, 0, 100) / 100;
  if (fill > 0) oled.fillRect(x+1, y+1, fill, h-2, SSD1306_WHITE);
}

void updateOled() {
  if (!OLED_ATTACHED) return;
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextWrap(false);
  oled.setTextSize(1);

  if (tankLowError) {
    oled.setCursor(0,  0); oled.print(F("!! WATER LOW !!"));
    char buf[18]; snprintf(buf, sizeof(buf), "Level: %d%%", waterPct);
    oled.setCursor(0, 11); oled.print(buf);
    oled.setCursor(0, 22); oled.print(F("Refill reservoir"));
    oled.display(); return;
  }

  if (!systemEnabled) {
    oled.setCursor(0,  0); oled.print(F("PooKooli BETA"));
    oled.setCursor(0, 12); oled.print(F("SYSTEM OFF"));
    oled.setCursor(0, 24); oled.print(F("Enable in web app"));
    oled.display(); return;
  }

  if (pumpRunning) {
    const char* who = (pumpSource==SRC_PIR) ? "Laser" : (pumpSource==SRC_TOUCH) ? "Touch" : (pumpSource==SRC_REMOTE) ? "Remote" : "App";
    char row0[22]; snprintf(row0, sizeof(row0), "%s  Pwr:%d/10", who, pumpPower);
    oled.setCursor(0, 0); oled.print(row0);
    if (pumpContinuous) {
      unsigned long el = (millis()-pumpStartTime)/1000;
      char buf[14]; snprintf(buf, sizeof(buf), "~%02lu:%02lu~", el/60, el%60);
      oled.setTextSize(2); oled.setCursor(16, 14); oled.print(buf);
    } else {
      unsigned long el  = millis()-pumpStartTime;
      unsigned long rem = (el>=pumpDuration)?0:(pumpDuration-el);
      char buf[6]; snprintf(buf, sizeof(buf), "%02lu:%02lu", rem/1000/60, (rem/1000)%60);
      oled.setTextSize(2);
      int x = (OLED_WIDTH-(int)strlen(buf)*12)/2;
      oled.setCursor(max(0,x), 14); oled.print(buf);
    }
    oled.display(); return;
  }

  // Idle — debug-focused: IP, live sensor distances, water bar
  oled.setCursor(0, 0);
  if (WiFi.status()==WL_CONNECTED) oled.print(WiFi.localIP().toString());
  else                              oled.print(F("No WiFi"));

  char distLine[22];
  char d1[8], d2[8];
  if (!tof1Ok)                strcpy(d1, "ERR");
  else if (lastDist1Cm >= 0)  snprintf(d1, sizeof(d1), "%ld", lastDist1Cm);
  else                        strcpy(d1, "--");
  if (!SENSOR2_ATTACHED)      strcpy(d2, "--");
  else if (!tof2Ok)           strcpy(d2, "ERR");
  else if (lastDist2Cm >= 0)  snprintf(d2, sizeof(d2), "%ld", lastDist2Cm);
  else                        strcpy(d2, "--");
  snprintf(distLine, sizeof(distLine), "S1:%scm S2:%scm", d1, d2);
  oled.setCursor(0, 11); oled.print(distLine);

  char wbuf[10]; snprintf(wbuf, sizeof(wbuf), "W:%d%%", waterPct);
  oled.setCursor(0, 23); oled.print(wbuf);
  drawBar(34, 23, 76, 8, waterPct);
  oled.display();
}

// ── JSON status ──────────────────────────────────────────────
String srcStr() {
  switch (pumpSource) {
    case SRC_PIR:    return "PIR";
    case SRC_WIFI:   return "WIFI";
    case SRC_TOUCH:  return "TOUCH";
    case SRC_REMOTE: return "REMOTE";
    default:       return "NONE";
  }
}
String buildStatusJson() {
  unsigned long el = pumpRunning ? (millis()-pumpStartTime) : 0;
  String j = "{";
  j += "\"running\":"        + String(pumpRunning    ?"true":"false") + ",";
  j += "\"continuous\":"     + String(pumpContinuous ?"true":"false") + ",";
  j += "\"empty_error\":"    + String(tankLowError   ?"true":"false") + ",";
  j += "\"locked\":"         + String((pumpRunning&&pumpSource!=SRC_WIFI)?"true":"false") + ",";
  j += "\"source\":\""       + srcStr() + "\",";
  j += "\"water\":"          + String(hasWater()?"true":"false") + ",";
  j += "\"water_pct\":"      + String(waterPct) + ",";
  j += "\"elapsed_ms\":"     + String(el) + ",";
  j += "\"duration_ms\":"    + String(pumpDuration) + ",";
  j += "\"pir_sec\":"        + String(pirDurationMs/1000) + ",";
  j += "\"touch_sec\":"      + String(touchDurationMs/1000) + ",";
  j += "\"remote_sec\":"     + String(remoteDurationMs/1000) + ",";
  j += "\"wifi_sec\":"       + String(wifiDurationMs/1000) + ",";
  j += "\"pump_power\":"     + String(pumpPower) + ",";
  j += "\"distance1_cm\":"   + String(triggerDistance1Cm) + ",";
  j += "\"distance2_cm\":"   + String(triggerDistance2Cm) + ",";
  j += "\"live_dist1_cm\":"  + String(lastDist1Cm) + ",";
  j += "\"live_dist2_cm\":"  + String(lastDist2Cm) + ",";
  j += "\"sensor1_ok\":"     + String(tof1Ok?"true":"false") + ",";
  j += "\"sensor2_ok\":"     + String(tof2Ok?"true":"false") + ",";
  j += "\"sensor2_attached\":" + String(SENSOR2_ATTACHED?"true":"false") + ",";
  j += "\"water_attached\":" + String(WATER_ATTACHED?"true":"false") + ",";
  j += "\"test_mode\":"      + String(testMode?"true":"false") + ",";
  j += "\"system_enabled\":" + String(systemEnabled?"true":"false") + ",";
  j += "\"laser1_enabled\":" + String(laser1Enabled?"true":"false") + ",";
  j += "\"laser2_enabled\":" + String(laser2Enabled?"true":"false") + ",";
  j += "\"touch_enabled\":"  + String(touchEnabled?"true":"false") + ",";
  j += "\"rf_enabled\":"     + String(rfEnabled?"true":"false");
  j += "}";
  return j;
}

// ── Web page ──────────────────────────────────────────────────
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1">
<title>PooKooli Fountain BETA V1.5</title>
<meta name="theme-color" content="#07090f">
<style>
@import url('https://fonts.googleapis.com/css2?family=Syne:wght@400;700;800&family=DM+Sans:wght@400;500&display=swap');
*{margin:0;padding:0;box-sizing:border-box;-webkit-tap-highlight-color:transparent}
:root{--bg:#07090f;--card:#0f1520;--card2:#131b28;--accent:#38bdf8;--on:#4ade80;--off:#f87171;
  --warn:#fbbf24;--inf:#a78bfa;--text:#e8f0fe;--muted:#4a5a72;--border:#1a2540;--radius:18px}
body{font-family:'DM Sans',sans-serif;background:var(--bg);color:var(--text);min-height:100vh;
  display:flex;flex-direction:column;align-items:center;padding:0 14px 56px;
  background-image:radial-gradient(ellipse 70% 35% at 50% -5%,rgba(56,189,248,.13) 0%,transparent 65%)}
.hdr{text-align:center;width:100%;max-width:380px}
.hdr-text{background:var(--card);border:1px solid var(--border);border-radius:20px;padding:14px 16px 18px;margin-top:16px}
h1{font-family:'Syne',sans-serif;font-size:1.6rem;font-weight:800;letter-spacing:-.03em;
  background:linear-gradient(120deg,#e0f2fe,#38bdf8 50%,#818cf8);-webkit-background-clip:text;-webkit-text-fill-color:transparent;margin:0}
.sub{font-size:.78rem;color:var(--muted);margin-top:3px}
.card{background:var(--card);border:1px solid var(--border);border-radius:var(--radius);padding:20px;width:100%;max-width:380px;margin-top:12px}
.card-title{font-family:'Syne',sans-serif;font-size:.65rem;font-weight:700;letter-spacing:.15em;text-transform:uppercase;color:var(--muted);margin-bottom:16px}
.water-gauge{margin-bottom:14px}
.water-label-row{display:flex;justify-content:space-between;align-items:center;margin-bottom:7px}
.water-pct{font-family:'Syne',sans-serif;font-size:1.4rem;font-weight:800}
.water-pct.ok{color:var(--on)}.water-pct.low{color:var(--warn)}.water-pct.empty{color:var(--off)}
.water-bar-bg{height:12px;background:var(--border);border-radius:99px;overflow:hidden}
.water-bar-fill{height:100%;border-radius:99px;transition:width .8s ease,background .5s}
.error-banner{display:none;background:rgba(248,113,113,.12);border:1px solid rgba(248,113,113,.4);border-radius:12px;padding:14px 16px;margin-bottom:14px;align-items:center;gap:12px}
.error-banner.show{display:flex}
.error-ico{font-size:1.8rem;flex-shrink:0}
.error-title{font-family:'Syne',sans-serif;font-size:.95rem;font-weight:700;color:var(--off)}
.error-msg{font-size:.78rem;color:#fca5a5;margin-top:2px}
.master-off-banner{display:none;background:rgba(248,113,113,.1);border:1px solid rgba(248,113,113,.3);border-radius:12px;padding:12px 16px;margin-bottom:14px;font-size:.82rem;color:#fca5a5;align-items:center;gap:10px}
.master-off-banner.show{display:flex}
.dot{width:12px;height:12px;border-radius:50%;flex-shrink:0;transition:all .4s}
.dot.on{background:var(--on);box-shadow:0 0 0 4px rgba(74,222,128,.2),0 0 12px rgba(74,222,128,.5)}
.dot.inf{background:var(--inf);box-shadow:0 0 0 4px rgba(167,139,250,.2),0 0 12px rgba(167,139,250,.6);animation:pulse 1.4s ease-in-out infinite}
.dot.off{background:var(--muted)}.dot.err{background:var(--off)}
@keyframes pulse{0%,100%{opacity:1}50%{opacity:.4}}
.status-row{display:flex;align-items:center;gap:12px}
.s-label{font-family:'Syne',sans-serif;font-size:1rem;font-weight:700}
.s-sub{font-size:.78rem;color:var(--muted);margin-top:2px}
.s-sub.red{color:#fca5a5}.s-sub.inf{color:var(--inf)}
.timer-wrap{margin-top:18px;display:none}.timer-wrap.active{display:block}
.t-row{display:flex;justify-content:space-between;font-size:.75rem;color:var(--muted);margin-bottom:7px}
.t-time{font-family:'Syne',sans-serif;font-weight:700;font-size:.85rem;color:var(--accent)}
.t-time.inf{color:var(--inf)}
.bar-bg{height:5px;background:var(--border);border-radius:99px;overflow:hidden}
.bar-fill{height:100%;width:0%;border-radius:99px;transition:width .8s linear;background:linear-gradient(90deg,var(--accent),var(--on))}
.bar-fill.inf{width:100%!important;background:linear-gradient(90deg,var(--inf),#c084fc)}
.chips{display:flex;gap:7px;flex-wrap:wrap;margin-top:18px}
.chip{font-size:.7rem;font-weight:500;padding:4px 11px;border-radius:99px;background:var(--border);color:var(--muted);transition:all .3s;border:1px solid transparent}
.chip.active{background:rgba(56,189,248,.15);color:var(--accent);border-color:rgba(56,189,248,.3)}
.chip.water-ok{background:rgba(74,222,128,.12);color:var(--on);border-color:rgba(74,222,128,.3)}
.chip.water-low{background:rgba(251,191,36,.12);color:var(--warn);border-color:rgba(251,191,36,.3)}
.chip.water-no{background:rgba(248,113,113,.1);color:var(--off);border-color:rgba(248,113,113,.25)}
.chip.inf-chip{background:rgba(167,139,250,.15);color:var(--inf);border-color:rgba(167,139,250,.35)}
.btn-grid-3{display:grid;grid-template-columns:1fr 1fr 1fr;gap:8px;margin-top:16px}
.btn{display:flex;flex-direction:column;align-items:center;justify-content:center;gap:4px;padding:13px 6px;border:none;border-radius:14px;
  font-family:'Syne',sans-serif;font-size:.82rem;font-weight:700;cursor:pointer;transition:transform .12s,opacity .15s}
.btn:active{transform:scale(.95);opacity:.8}.btn .ico{font-size:1.35rem}
.btn-on{background:linear-gradient(145deg,#166534,#22c55e);color:#fff}
.btn-inf{background:linear-gradient(145deg,#4c1d95,#a78bfa);color:#fff}
.btn-off{background:linear-gradient(145deg,#991b1b,#ef4444);color:#fff}
.btn-restart{background:linear-gradient(145deg,#164e63,#22d3ee);color:#fff}
.btn-sub{font-size:.6rem;font-weight:400;opacity:.8;font-family:'DM Sans',sans-serif}
.btn-locked{opacity:.35;pointer-events:none;filter:grayscale(.7)}
.toggle-row{display:flex;align-items:center;justify-content:space-between;padding:13px 0;border-bottom:1px solid var(--border)}
.toggle-row:last-child{border-bottom:none}
.toggle-label{font-size:.88rem;font-weight:500}
.toggle-label small{display:block;font-size:.72rem;color:var(--muted);margin-top:1px}
.switch{position:relative;width:50px;height:28px;flex-shrink:0}
.switch input{opacity:0;width:0;height:0;position:absolute}
.switch-track{position:absolute;inset:0;background:var(--border);border-radius:99px;cursor:pointer;transition:background .25s}
.switch-track::before{content:'';position:absolute;width:22px;height:22px;left:3px;top:3px;background:#fff;border-radius:50%;transition:transform .25s;box-shadow:0 1px 3px rgba(0,0,0,.4)}
.switch input:checked + .switch-track{background:var(--on)}
.switch input:checked + .switch-track::before{transform:translateX(22px)}
.master-card.disabled-look{opacity:.55}
.settings-card{background:var(--card2)}
.timer-setting{display:flex;align-items:center;justify-content:space-between;padding:13px 0;border-bottom:1px solid var(--border)}
.timer-setting:last-child{border-bottom:none}
.ts-label{font-size:.88rem;font-weight:500}
.ts-label small{display:block;font-size:.72rem;color:var(--muted);margin-top:1px}
.ts-controls{display:flex;align-items:center}
.ts-btn{width:36px;height:36px;border:1px solid var(--border);background:var(--bg);color:var(--text);font-size:1.2rem;font-weight:700;border-radius:8px;cursor:pointer;display:flex;align-items:center;justify-content:center;transition:background .15s}
.ts-btn:active{background:var(--border)}
.ts-val{min-width:56px;text-align:center;font-family:'Syne',sans-serif;font-size:.95rem;font-weight:700;color:var(--accent);padding:0 4px;transition:color .2s}
.ts-val.pending{color:var(--warn)!important;position:relative}
.ts-val.pending::after{content:'●';position:absolute;top:-6px;right:0;font-size:7px;color:var(--warn)}
.power-bar{display:flex;gap:3px;margin-top:6px}
.power-pip{height:6px;flex:1;border-radius:2px;background:var(--border);transition:background .2s}
.dist-row{padding:13px 0}
.dist-row input[type=range]{width:100%;margin-top:10px;accent-color:var(--accent)}
.dist-readout{display:flex;justify-content:space-between;align-items:center}
.remote-card{background:var(--card2)}
.remote-grid{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-top:2px}
.rkey{background:var(--border);border-radius:10px;padding:10px 6px;text-align:center}
.rkey .rk{font-family:'Syne',sans-serif;font-weight:800;font-size:1.1rem;color:var(--accent);display:block}
.rkey .rl{font-size:.68rem;color:var(--muted);margin-top:3px;display:block}
.save-btn{width:100%;margin-top:16px;padding:14px;background:linear-gradient(135deg,#1d4ed8,#38bdf8);color:#fff;border:none;border-radius:12px;
  font-family:'Syne',sans-serif;font-size:.95rem;font-weight:700;cursor:pointer;display:flex;align-items:center;justify-content:center;gap:8px;transition:opacity .15s,transform .1s,background .3s}
.save-btn:active{transform:scale(.97);opacity:.85}
.save-btn.pending{background:linear-gradient(135deg,#92400e,#fbbf24)!important;animation:pulse-save 1.4s ease-in-out infinite}
@keyframes pulse-save{0%,100%{opacity:1}50%{opacity:.7}}
.toast{position:fixed;bottom:24px;left:50%;transform:translateX(-50%) translateY(80px);background:#1e293b;color:var(--on);padding:10px 22px;border-radius:99px;
  font-size:.85rem;font-weight:600;border:1px solid rgba(74,222,128,.3);transition:transform .35s cubic-bezier(.34,1.56,.64,1),opacity .35s;opacity:0;white-space:nowrap;z-index:999}
.toast.show{transform:translateX(-50%) translateY(0);opacity:1}
.toast.err{color:var(--off);border-color:rgba(248,113,113,.4)}
.note{font-size:.7rem;color:var(--muted);margin-top:24px;text-align:center;opacity:.55;line-height:1.7}
.test-badge{display:none;font-size:.6rem;font-weight:700;padding:2px 8px;border-radius:99px;background:rgba(251,191,36,.15);color:var(--warn);border:1px solid rgba(251,191,36,.4);margin-left:6px;vertical-align:middle}
.test-badge.show{display:inline-block}
.dbg-row{display:flex;align-items:center;justify-content:space-between;padding:11px 0;border-bottom:1px solid var(--border)}
.dbg-name{font-size:.85rem;font-weight:500}
.dbg-name small{display:block;font-size:.7rem;margin-top:2px;color:var(--muted)}
.dbg-name small.ok{color:var(--on)}
.dbg-name small.err{color:var(--off);font-weight:700}
.dbg-dist{font-family:'Syne',sans-serif;font-size:1.05rem;font-weight:800;color:var(--accent);min-width:80px;text-align:right}
</style>
</head>
<body>

<div class="hdr">
  <div class="hdr-text">
    <h1>🐾 PooKooli Fountain</h1>
    <p class="sub">BETA V1.5 — full module set: lasers, touch, water, RF, OLED</p>
  </div>
</div>

<div class="card master-card" id="masterCard">
  <div class="toggle-row" style="border-bottom:none;padding:0">
    <div class="toggle-label">🔌 System Power<small>Master switch — whole fountain on/off</small></div>
    <label class="switch"><input type="checkbox" id="swSystem" onchange="toggleSwitch('system',this.checked)"><span class="switch-track"></span></label>
  </div>
</div>

<div class="card">
  <div class="card-title">Live Status</div>
  <div class="master-off-banner" id="masterOffBanner">
    <span style="font-size:1.3rem">🔌</span><span>System is OFF — pump will not start until you turn it back on.</span>
  </div>
  <div class="error-banner" id="errorBanner">
    <div class="error-ico">🪣</div>
    <div><div class="error-title">Water level is low!</div><div class="error-msg">Refill the reservoir before using the pump.</div></div>
  </div>
  <div class="water-gauge">
    <div class="water-label-row"><span class="ts-label">💧 Water Level<span class="test-badge" id="waterTestBadge">TEST 50%</span></span><span class="water-pct" id="waterPctLabel">--%</span></div>
    <div class="water-bar-bg"><div class="water-bar-fill" id="waterBarFill" style="width:0%"></div></div>
  </div>
  <div class="status-row">
    <div class="dot" id="pumpDot"></div>
    <div><div class="s-label" id="pumpLabel">Loading…</div><div class="s-sub" id="pumpSub"></div></div>
  </div>
  <div class="timer-wrap" id="timerWrap">
    <div class="t-row"><span id="timerLabel">Pump running</span><span class="t-time" id="timerText">--:--</span></div>
    <div class="bar-bg"><div class="bar-fill" id="barFill"></div></div>
  </div>
  <div class="chips">
    <span class="chip" id="chipWater">💧 Water</span>
    <span class="chip" id="chipPir">🔴 Laser</span>
    <span class="chip" id="chipTouch">👆 Touch</span>
    <span class="chip" id="chipRemote">📺 Remote</span>
    <span class="chip" id="chipWifi">📱 App</span>
    <span class="chip" id="chipInf">∞ Continuous</span>
  </div>
</div>

<div class="card">
  <div class="card-title">Control</div>
  <div class="btn-grid-3">
    <button class="btn btn-on" id="btnOn" onclick="sendCmd('on')"><span class="ico">▶</span>Start<span class="btn-sub" id="wifiDurLabel">2 min</span></button>
    <button class="btn btn-inf" id="btnInf" onclick="sendCmd('continuous')"><span class="ico">∞</span>Infinite<span class="btn-sub">until stopped</span></button>
    <button class="btn btn-off" onclick="sendCmd('off')"><span class="ico">⏹</span>Stop<span class="btn-sub">immediately</span></button>
  </div>
  <button class="btn btn-restart" id="btnRestart" onclick="restartBoard()" style="margin-top:10px;width:100%"><span class="ico">🔄</span>Restart Board<span class="btn-sub">~5 sec downtime</span></button>
</div>

<div class="card">
  <div class="card-title">Sensor Switches</div>
  <div class="toggle-row">
    <div class="toggle-label">🔴 Laser Sensor #1<small>XSHUT addr 0x30</small></div>
    <label class="switch"><input type="checkbox" id="swLaser1" onchange="toggleSwitch('laser1',this.checked)"><span class="switch-track"></span></label>
  </div>
  <div class="toggle-row">
    <div class="toggle-label">🔴 Laser Sensor #2<small id="lblLaser2">XSHUT addr 0x31</small></div>
    <label class="switch"><input type="checkbox" id="swLaser2" onchange="toggleSwitch('laser2',this.checked)"><span class="switch-track"></span></label>
  </div>
  <div class="toggle-row">
    <div class="toggle-label">👆 Touch Sensor<small>Capacitive module — press to start/stop</small></div>
    <label class="switch"><input type="checkbox" id="swTouch" onchange="toggleSwitch('touch',this.checked)"><span class="switch-track"></span></label>
  </div>
  <div class="toggle-row">
    <div class="toggle-label">📺 433MHz Remote<small>A/B control pump · C/D are maintenance</small></div>
    <label class="switch"><input type="checkbox" id="swRf" onchange="toggleSwitch('rf',this.checked)"><span class="switch-track"></span></label>
  </div>
</div>

<!-- REMOTE CONTROL GUIDE -->
<div class="card remote-card">
  <div class="card-title">📺 Remote Control Guide</div>
  <p style="font-size:.72rem;color:var(--muted);margin:-4px 0 10px">Hold each button for 3 seconds — a quick tap won't trigger anything (filters out noise/false triggers)</p>
  <div class="remote-grid">
    <div class="rkey"><span class="rk">A</span><span class="rl">Start/Stop timed</span></div>
    <div class="rkey"><span class="rk">B</span><span class="rl">Infinite Start/Stop</span></div>
    <div class="rkey"><span class="rk">C</span><span class="rl">Recover I2C bus</span></div>
    <div class="rkey"><span class="rk">D</span><span class="rl">Restart board</span></div>
  </div>
</div>

<!-- DEBUG & TEST -->
<div class="card">
  <div class="card-title">🔧 Debug &amp; Test</div>
  <div class="dbg-row">
    <div class="dbg-name">🔴 Sensor #1 — live distance<small id="dbgS1Status">checking…</small></div>
    <div class="dbg-dist" id="dbgS1Dist">--</div>
  </div>
  <div class="dbg-row">
    <div class="dbg-name">🔴 Sensor #2 — live distance<small id="dbgS2Status">checking…</small></div>
    <div class="dbg-dist" id="dbgS2Dist">--</div>
  </div>
  <div class="toggle-row">
    <div class="toggle-label">🧪 Water Test Mode<small id="lblTestMode">Forces level to 50% so the pump runs with an empty tank.<br>Never saved — always OFF after a restart.</small></div>
    <label class="switch"><input type="checkbox" id="swTestMode" onchange="toggleSwitch('testmode',this.checked)"><span class="switch-track"></span></label>
  </div>
  <button class="btn btn-restart" id="btnI2cRecover" onclick="recoverI2C()" style="margin-top:12px;width:100%"><span class="ico">🔌</span>Recover I2C Bus<span class="btn-sub">re-inits sensors + OLED, no reboot</span></button>
</div>

<div class="card settings-card">
  <div class="card-title">⏱ Timer &amp; Power Settings</div>
  <div class="timer-setting">
    <div class="ts-label">📺 Remote Timer (A)<small>Run time when remote button A triggers</small></div>
    <div class="ts-controls">
      <button class="ts-btn" onclick="adj('remote',-10)">−</button><div class="ts-val" id="val-remote">60s</div><button class="ts-btn" onclick="adj('remote',+10)">+</button>
    </div>
  </div>
  <div class="timer-setting">
    <div class="ts-label">👆 Touch Button<small>Run time when touch sensor triggers</small></div>
    <div class="ts-controls">
      <button class="ts-btn" onclick="adj('touch',-10)">−</button><div class="ts-val" id="val-touch">60s</div><button class="ts-btn" onclick="adj('touch',+10)">+</button>
    </div>
  </div>
  <div class="timer-setting">
    <div class="ts-label">🔴 Laser Sensors<small>Run time once triggered</small></div>
    <div class="ts-controls">
      <button class="ts-btn" onclick="adj('pir',-10)">−</button><div class="ts-val" id="val-pir">60s</div><button class="ts-btn" onclick="adj('pir',+10)">+</button>
    </div>
  </div>
  <div class="timer-setting">
    <div class="ts-label">📱 App Start<small>Timed start from this page</small></div>
    <div class="ts-controls">
      <button class="ts-btn" onclick="adj('wifi',-10)">−</button><div class="ts-val" id="val-wifi">120s</div><button class="ts-btn" onclick="adj('wifi',+10)">+</button>
    </div>
  </div>
  <div class="timer-setting" style="flex-direction:column;align-items:stretch;gap:10px">
    <div style="display:flex;justify-content:space-between;align-items:center">
      <div class="ts-label">⚡ Pump Power<small>Motor speed when running</small></div>
      <div style="display:flex;align-items:center"><button class="ts-btn" onclick="adj('power',-1)">−</button><div class="ts-val" id="val-power">8/10</div><button class="ts-btn" onclick="adj('power',+1)">+</button></div>
    </div>
    <div class="power-bar" id="powerBar">
      <div class="power-pip" id="pip1"></div><div class="power-pip" id="pip2"></div><div class="power-pip" id="pip3"></div><div class="power-pip" id="pip4"></div><div class="power-pip" id="pip5"></div>
      <div class="power-pip" id="pip6"></div><div class="power-pip" id="pip7"></div><div class="power-pip" id="pip8"></div><div class="power-pip" id="pip9"></div><div class="power-pip" id="pip10"></div>
    </div>
  </div>
  <div class="dist-row">
    <div class="dist-readout"><div class="ts-label">📏 Sensor #1 Distance<small>Trigger when object closer than this</small></div><div class="ts-val" id="val-distance1">15 cm</div></div>
    <input type="range" id="rangeDistance1" min="5" max="70" step="1" value="15" oninput="adjDistance('distance1',this.value)">
  </div>
  <div class="dist-row">
    <div class="dist-readout"><div class="ts-label">📏 Sensor #2 Distance<small>Trigger when object closer than this</small></div><div class="ts-val" id="val-distance2">15 cm</div></div>
    <input type="range" id="rangeDistance2" min="5" max="70" step="1" value="15" oninput="adjDistance('distance2',this.value)">
  </div>
  <button class="save-btn" id="saveBtn" onclick="saveSettings()">💾 Save Settings to Device</button>
</div>

<p class="note">
  PooKooli Fountain BETA V1.5: 2× laser sensors + touch + water + 433MHz remote + pump + OLED.<br>
  Distance range: 5–70 cm on both sensors. Water sensor now ATTACHED.<br>
  🐾 Laser-triggered pump keeps renewing while the cat stays in range.<br>
  OLED mirrors status + live cm readings — handy without opening this page.<br>
  👆 Touch is a universal override — press to stop any run, or start one.<br>
  💧 Calibrate WATER_ADC_DRY/FULL in the firmware to your actual sensor.<br>
  📺 Remote: hold 3s. A=timed start/stop · B=infinite · C=I2C recovery · D=restart.<br>
  🔧 Debug card: boot check + live cm readings (also on Serial monitor)<br>
  🧪 Test mode fakes 50% water — remember it resets to OFF on restart<br>
  Timers: ±10s · Power: ±1 (1–10) · Distance: 2–120cm<br>
  🟡 Yellow = unsaved · tap Save to write to flash
</p>
<div class="toast" id="toast"></div>

<script>
var settings = { pir:60, wifi:120, touch:60, remote:60, power:8, distance1:15, distance2:15 };
var localOverride = new Set();
var MIN_SEC=5, MAX_SEC=600, MIN_DIST=5, MAX_DIST=70;
var pumpEndTime=0, timerTick=null, lastDurationMs=0;
var elapsedTick=null, elapsedStart=0;

var adj = function(key, delta) {
  localOverride.add(key);
  if (key==='power') settings.power = Math.min(10, Math.max(1, settings.power+delta));
  else settings[key] = Math.min(MAX_SEC, Math.max(MIN_SEC, settings[key]+delta));
  render(key); updateSaveBtn();
};
var adjDistance = function(key, val) {
  localOverride.add(key);
  settings[key] = Math.min(MAX_DIST, Math.max(MIN_DIST, parseInt(val,10)));
  render(key); updateSaveBtn();
};
var render = function(key) {
  var el = document.getElementById('val-'+key); if (!el) return;
  el.classList.toggle('pending', localOverride.has(key));
  if (key==='power') { el.textContent=settings.power+'/10'; renderPowerBar(); }
  else if (key==='distance1' || key==='distance2') {
    el.textContent=settings[key]+' cm';
    var sliderId=key==='distance1'?'rangeDistance1':'rangeDistance2';
    var s=document.getElementById(sliderId);
    if (s && document.activeElement!==s) s.value=settings[key];
  } else {
    var v=settings[key], str=v>=60?Math.floor(v/60)+'m'+(v%60?v%60+'s':''):v+'s';
    el.textContent=str;
    if (key==='wifi') document.getElementById('wifiDurLabel').textContent=str;
  }
};
var renderPowerBar = function() {
  var p=settings.power, pending=localOverride.has('power');
  for (var i=1;i<=10;i++) {
    var pip=document.getElementById('pip'+i); if (!pip) continue;
    pip.style.background = i<=p ? (pending?'#fbbf24':(i<=3?'#f87171':i<=6?'#fbbf24':'#4ade80')) : 'var(--border)';
  }
};
var updateSaveBtn = function() {
  var btn=document.getElementById('saveBtn');
  if (localOverride.size>0) { btn.classList.add('pending'); btn.textContent='💾 Save ('+localOverride.size+' unsaved change'+(localOverride.size>1?'s':'')+')'; }
  else { btn.classList.remove('pending'); btn.textContent='💾 Save Settings to Device'; }
};
var saveSettings = async function() {
  var url='/settings?pir='+settings.pir+'&touch='+settings.touch+'&remote='+settings.remote+'&wifi='+settings.wifi+'&power='+settings.power+'&distance1='+settings.distance1+'&distance2='+settings.distance2;
  try {
    var j=await (await fetch(url)).json();
    if (j.ok) { localOverride.clear(); ['pir','wifi','touch','remote','power','distance1','distance2'].forEach(render); updateSaveBtn(); showToast('✓ Saved!'); }
    else showToast('⚠ Save failed',true);
  } catch(e) { showToast('⚠ No connection',true); }
};
var sendCmd = async function(cmd) {
  if (cmd!=='off' && (window._pumpLocked || window._systemOff)) return;
  try { applyStatus(await (await fetch('/pump?cmd='+cmd)).json()); } catch(e) {}
};
var restartBoard = async function() {
  if (!confirm('Restart the board?\nFountain offline ~5 seconds.')) return;
  var btn=document.getElementById('btnRestart');
  btn.disabled=true; btn.innerHTML='<span class="ico">⏳</span>Restarting…<span class="btn-sub">reconnecting</span>';
  try { await fetch('/restart'); } catch(e) {}
  var waited=0, poll=setInterval(async function() {
    waited++;
    btn.innerHTML='<span class="ico">⏳</span>Restarting…<span class="btn-sub">'+waited+'s</span>';
    try { var r=await fetch('/status'); if(r.ok){ clearInterval(poll); btn.disabled=false; btn.innerHTML='<span class="ico">🔄</span>Restart Board<span class="btn-sub">~5 sec downtime</span>'; showToast('✓ Back online'); applyStatus(await r.json()); } } catch(e) {}
    if (waited>30){ clearInterval(poll); btn.disabled=false; btn.innerHTML='<span class="ico">🔄</span>Restart Board<span class="btn-sub">~5 sec downtime</span>'; showToast('⚠ Not responding',true); }
  },1000);
};
var recoverI2C = async function() {
  var btn=document.getElementById('btnI2cRecover');
  btn.disabled=true; btn.innerHTML='<span class="ico">⏳</span>Recovering…<span class="btn-sub">re-scanning bus</span>';
  try {
    var r = await fetch('/i2crecover');
    if (r.ok) { applyStatus(await r.json()); showToast('✓ Bus recovery done — check sensor status above'); }
    else showToast('⚠ Recovery request failed', true);
  } catch(e) { showToast('⚠ No connection', true); }
  btn.disabled=false; btn.innerHTML='<span class="ico">🔌</span>Recover I2C Bus<span class="btn-sub">re-inits sensors + OLED, no reboot</span>';
};
var toggleSwitch = async function(which, checked) {
  try { applyStatus(await (await fetch('/toggle?which='+which+'&state='+(checked?'1':'0'))).json()); showToast(checked?'✓ On':'✓ Off'); }
  catch(e) { showToast('⚠ No connection',true); var m={system:'swSystem',laser1:'swLaser1',laser2:'swLaser2',touch:'swTouch',rf:'swRf',testmode:'swTestMode'}; var b=document.getElementById(m[which]); if(b) b.checked=!checked; }
};
var fetchStatus = async function() { try { applyStatus(await (await fetch('/status')).json()); } catch(e) {} };
var applyStatus = function(d) {
  if (!localOverride.has('pir')   && d.pir_sec)               { settings.pir  =d.pir_sec;       render('pir');   }
  if (!localOverride.has('touch') && d.touch_sec)             { settings.touch=d.touch_sec;     render('touch'); }
  if (!localOverride.has('wifi')  && d.wifi_sec)              { settings.wifi =d.wifi_sec;      render('wifi');  }
  if (!localOverride.has('power') && d.pump_power!==undefined) { settings.power=d.pump_power;   render('power'); }
  if (!localOverride.has('distance1') && d.distance1_cm!==undefined) { settings.distance1=d.distance1_cm; render('distance1'); }
  if (!localOverride.has('distance2') && d.distance2_cm!==undefined) { settings.distance2=d.distance2_cm; render('distance2'); }

  if (d.water_pct !== undefined) {
    var pct=d.water_pct;
    var lbl=document.getElementById('waterPctLabel');
    var fill=document.getElementById('waterBarFill');
    if (lbl) { lbl.textContent=pct+'%'; lbl.className='water-pct '+(pct>=50?'ok':pct>=20?'low':'empty'); }
    if (fill) { fill.style.width=pct+'%'; fill.style.background=pct>=50?'var(--on)':pct>=20?'var(--warn)':'var(--off)'; }
  }

  var swSys=document.getElementById('swSystem'); if (swSys && document.activeElement!==swSys && d.system_enabled!==undefined) swSys.checked=d.system_enabled;
  var swL1=document.getElementById('swLaser1');  if (swL1  && document.activeElement!==swL1  && d.laser1_enabled!==undefined) swL1.checked=d.laser1_enabled;
  var swL2=document.getElementById('swLaser2');  if (swL2  && document.activeElement!==swL2  && d.laser2_enabled!==undefined) swL2.checked=d.laser2_enabled;
  var swTc=document.getElementById('swTouch');   if (swTc  && document.activeElement!==swTc  && d.touch_enabled!==undefined)  swTc.checked=d.touch_enabled;
  var swRf=document.getElementById('swRf');      if (swRf  && document.activeElement!==swRf  && d.rf_enabled!==undefined)     swRf.checked=d.rf_enabled;
  if (!localOverride.has('remote') && d.remote_sec)           { settings.remote=d.remote_sec;   render('remote'); }
  var swTM=document.getElementById('swTestMode'); if (swTM && document.activeElement!==swTM && d.test_mode!==undefined) swTM.checked=d.test_mode;
  var tb=document.getElementById('waterTestBadge'); if(tb) tb.classList.toggle('show', !!d.test_mode);

  // Debug card: sensor connectivity + live distance
  var s1s=document.getElementById('dbgS1Status'), s1d=document.getElementById('dbgS1Dist');
  if (s1s && d.sensor1_ok!==undefined) { s1s.textContent = d.sensor1_ok ? '✓ connected (addr 0x30)' : '✗ ERROR — check wiring/power'; s1s.className = d.sensor1_ok ? 'ok' : 'err'; }
  if (s1d) s1d.textContent = (d.sensor1_ok===false) ? 'ERR' : (d.live_dist1_cm>=0 ? d.live_dist1_cm+' cm' : '--');
  var s2s=document.getElementById('dbgS2Status'), s2d=document.getElementById('dbgS2Dist');
  if (s2s) {
    if (d.sensor2_attached===false) { s2s.textContent='— detached (not in this build)'; s2s.className=''; }
    else if (d.sensor2_ok!==undefined) { s2s.textContent = d.sensor2_ok ? '✓ connected (addr 0x31)' : '✗ ERROR — check wiring/power'; s2s.className = d.sensor2_ok ? 'ok' : 'err'; }
  }
  if (s2d) s2d.textContent = (d.sensor2_attached===false) ? '—' : ((d.sensor2_ok===false) ? 'ERR' : (d.live_dist2_cm>=0 ? d.live_dist2_cm+' cm' : '--'));
  var l2=document.getElementById('lblLaser2');
  if (l2 && d.sensor2_attached===false) l2.textContent='DETACHED in this build';
  var ltm=document.getElementById('lblTestMode');
  if (ltm && d.water_attached===false) ltm.innerHTML='Water sensor DETACHED in this build.<br>Test mode is locked ON — level fixed at 50%.';

  window._systemOff  = (d.system_enabled===false);
  window._pumpLocked = d.locked;
  var mc=document.getElementById('masterCard'); if(mc) mc.classList.toggle('disabled-look',window._systemOff);
  var mob=document.getElementById('masterOffBanner'); if(mob) mob.classList.toggle('show',window._systemOff);
  document.getElementById('btnOn').classList.toggle('btn-locked',  !!d.locked||window._systemOff);
  document.getElementById('btnInf').classList.toggle('btn-locked', !!d.locked||window._systemOff);

  var src=d.source;
  ['Pir','Touch','Remote','Wifi'].forEach(function(s){ var c=document.getElementById('chip'+s); if(c) c.classList.toggle('active',src===s.toUpperCase()); });
  var ci=document.getElementById('chipInf'); if(ci) ci.className='chip'+(d.continuous?' inf-chip':'');
  var wc=document.getElementById('chipWater');
  if (wc) { var wp=d.water_pct||0; wc.className='chip '+(wp>=50?'water-ok':wp>=20?'water-low':'water-no'); wc.textContent='💧 Water '+wp+'%'; }

  var banner=document.getElementById('errorBanner');
  var dot=document.getElementById('pumpDot'), lbl=document.getElementById('pumpLabel'), sub=document.getElementById('pumpSub');
  var wrap=document.getElementById('timerWrap'), bar=document.getElementById('barFill'), tt=document.getElementById('timerText'), tl=document.getElementById('timerLabel');

  if (window._systemOff && !d.running) {
    dot.className='dot off'; lbl.textContent='System is OFF'; sub.className='s-sub'; sub.textContent='Pump disabled';
    if(banner) banner.classList.remove('show'); stopTicks(); resetTimerUI(); return;
  }
  if (d.empty_error) {
    if(banner) banner.classList.add('show');
    dot.className='dot err'; lbl.textContent='Water level low!'; sub.className='s-sub red'; sub.textContent='Refill the reservoir';
    stopTicks(); resetTimerUI(); return;
  }
  if(banner) banner.classList.remove('show');

  var srcMap={PIR:'Laser sensor',TOUCH:'Touch sensor',REMOTE:'433MHz remote',WIFI:'App'};
  if (d.running && d.continuous) {
    dot.className='dot inf'; lbl.textContent='Running continuously'; sub.className='s-sub inf';
    sub.textContent=(srcMap[src]||src)+' · Stop via app';
    wrap.classList.add('active'); tl.textContent='Elapsed';
    bar.className='bar-fill inf'; tt.className='t-time inf';
    elapsedStart=Date.now()-d.elapsed_ms; stopTimerTick();
    if (!elapsedTick) elapsedTick=setInterval(function(){tt.textContent=fmtMs(Date.now()-elapsedStart);},500);
    return;
  }
  if (d.running) {
    dot.className='dot on'; lbl.textContent='Pump is running'; sub.className='s-sub';
    sub.textContent='Triggered by: '+(srcMap[src]||src)+' · Power: '+d.pump_power+'/10';
    wrap.classList.add('active'); tl.textContent='Pump running';
    bar.className='bar-fill'; tt.className='t-time';
    lastDurationMs=d.duration_ms;
    var rem=Math.max(0,d.duration_ms-d.elapsed_ms);
    pumpEndTime=Date.now()+rem;
    bar.style.width=Math.min(100,d.elapsed_ms/d.duration_ms*100)+'%';
    tt.textContent=fmtMs(rem); stopElapsedTick();
    if (!timerTick) timerTick=setInterval(function(){
      var r2=Math.max(0,pumpEndTime-Date.now());
      bar.style.width=Math.min(100,(lastDurationMs-r2)/lastDurationMs*100)+'%';
      tt.textContent=fmtMs(r2); if(r2<=0) stopTimerTick();
    },400);
    return;
  }
  window._pumpLocked=false;
  dot.className='dot off'; lbl.textContent='Pump is OFF'; sub.className='s-sub'; sub.textContent='Waiting for trigger…';
  stopTicks(); resetTimerUI();
};
var stopTimerTick=function(){ if(timerTick){clearInterval(timerTick);timerTick=null;} };
var stopElapsedTick=function(){ if(elapsedTick){clearInterval(elapsedTick);elapsedTick=null;} };
var stopTicks=function(){ stopTimerTick(); stopElapsedTick(); };
var resetTimerUI=function(){
  var wrap=document.getElementById('timerWrap'),bar=document.getElementById('barFill'),tt=document.getElementById('timerText');
  wrap.classList.remove('active'); bar.className='bar-fill'; bar.style.width='0%'; tt.className='t-time'; tt.textContent='0:00';
};
var fmtMs=function(ms){ var s=Math.ceil(ms/1000),m=Math.floor(s/60); return m+':'+String(s%60).padStart(2,'0'); };
var showToast=function(msg,isErr){
  var t=document.getElementById('toast'); t.textContent=msg;
  t.className='toast show'+(isErr?' err':'');
  setTimeout(function(){t.classList.remove('show');},3000);
};
fetchStatus();
setInterval(fetchStatus, 400);    // 400ms — matches firmware's faster sample rate
</script>
</body>
)rawliteral";

// ── Web handlers ──────────────────────────────────────────────
void handleRoot()   { server.send(200,"text/html",FPSTR(INDEX_HTML)); }
void handleStatus() { server.send(200,"application/json",buildStatusJson()); }

void handlePumpCmd() {
  if (!systemEnabled) { pumpOFF(); server.send(200,"application/json",buildStatusJson()); return; }
  if (!server.hasArg("cmd")) { server.send(400,"text/plain","Missing cmd"); return; }
  String cmd = server.arg("cmd");
  if (cmd=="on") {
    if (pumpRunning && pumpSource!=SRC_WIFI) { server.send(200,"application/json",buildStatusJson()); return; }
    if (!hasWater()) triggerBlocked(SRC_WIFI);
    else startPump(SRC_WIFI, wifiDurationMs);
  } else if (cmd=="continuous") {
    if (pumpRunning && pumpSource!=SRC_WIFI) { server.send(200,"application/json",buildStatusJson()); return; }
    if (!hasWater()) triggerBlocked(SRC_WIFI);
    else startPumpForever(SRC_WIFI);
  } else if (cmd=="off") {
    pumpOFF(); tankLowError=false;
  }
  server.send(200,"application/json",buildStatusJson());
}

void handleSettings() {
  bool changed=false;
  if (server.hasArg("pir"))       { long v=server.arg("pir").toInt();       if(v>=MIN_SEC&&v<=MAX_SEC)                 { pirDurationMs =v*1000UL;        changed=true; } }
  if (server.hasArg("touch"))     { long v=server.arg("touch").toInt();     if(v>=MIN_SEC&&v<=MAX_SEC)                 { touchDurationMs=v*1000UL;       changed=true; } }
  if (server.hasArg("remote"))    { long v=server.arg("remote").toInt();    if(v>=MIN_SEC&&v<=MAX_SEC)                 { remoteDurationMs=v*1000UL;      changed=true; } }
  if (server.hasArg("wifi"))      { long v=server.arg("wifi").toInt();      if(v>=MIN_SEC&&v<=MAX_SEC)                 { wifiDurationMs=v*1000UL;        changed=true; } }
  if (server.hasArg("power"))     { int  v=server.arg("power").toInt();     if(v>=1&&v<=10)                            { pumpPower=(uint8_t)v;           changed=true; } }
  if (server.hasArg("distance1")) { int  v=server.arg("distance1").toInt(); if(v>=MIN_DISTANCE_CM&&v<=MAX_DISTANCE_CM) { triggerDistance1Cm=(uint8_t)v;  changed=true; } }
  if (server.hasArg("distance2")) { int  v=server.arg("distance2").toInt(); if(v>=MIN_DISTANCE_CM&&v<=MAX_DISTANCE_CM) { triggerDistance2Cm=(uint8_t)v;  changed=true; } }
  if (changed) saveSettings();
  String j="{\"ok\":true,\"pir_sec\":"+String(pirDurationMs/1000)
           +",\"touch_sec\":"+String(touchDurationMs/1000)
           +",\"remote_sec\":"+String(remoteDurationMs/1000)
           +",\"wifi_sec\":"+String(wifiDurationMs/1000)
           +",\"pump_power\":"+String(pumpPower)
           +",\"distance1_cm\":"+String(triggerDistance1Cm)
           +",\"distance2_cm\":"+String(triggerDistance2Cm)+"}";
  server.send(200,"application/json",j);
}

void handleToggle() {
  if (!server.hasArg("which")||!server.hasArg("state")) { server.send(400,"text/plain","Missing args"); return; }
  String which = server.arg("which");
  bool   state = (server.arg("state")=="1"||server.arg("state")=="true");
  bool   persist = true;
  if      (which=="system") { systemEnabled=state; if(!systemEnabled){pumpOFF();tankLowError=false;} }
  else if (which=="laser1") { laser1Enabled=state; }
  else if (which=="laser2") { laser2Enabled=state; }
  else if (which=="touch")  { touchEnabled=state; }
  else if (which=="rf")     { rfEnabled=state; }
  else if (which=="testmode") {
    persist = false;    // deliberately NOT saved to flash
    if (!state && !WATER_ATTACHED) {
      // water sensor isn't wired — refuse to leave test mode
      testMode = true;
      Serial.println("[Test] Water sensor DETACHED — test mode stays ON");
    } else {
      testMode = state;
      if (testMode) {
        waterPct = 50;
        Serial.println("[Test] WATER TEST MODE ON — level forced to 50%");
      } else {
        readWaterLevel();
        Serial.println("[Test] WATER TEST MODE OFF — real sensor readings resumed");
      }
    }
  }
  else { server.send(400,"text/plain","Unknown which"); return; }
  if (persist) saveSettings();
  server.send(200,"application/json",buildStatusJson());
}

void handleRestart() {
  pumpOFF();
  server.send(200,"application/json","{\"restarting\":true}");
  delay(300);
  Serial.println("[System] Restarting via web");
  ESP.restart();
}

void handleI2CRecover() {
  Serial.println("[Recovery] Manual I2C bus recovery triggered via web app");
  Wire.end();               // release the bus before bit-banging it directly
  recoverI2CBus(21, 22);
  Wire.begin(21, 22);
  Wire.setClock(100000);
  delay(50);
  initI2CDevices();         // full re-init: both sensors + OLED
  server.send(200,"application/json",buildStatusJson());
}

// ── I2C bus recovery ───────────────────────────────────────────
// If a device gets interrupted mid-transaction (noise, brief power
// glitch, an unplugged sensor mid-read), it can leave SDA stuck LOW,
// jamming the entire bus for every device on it. This manually pulses
// SCL up to 9 times to force a stuck device to release SDA, then sends
// a manual STOP condition. Standard I2C bus recovery procedure — must
// be called with Wire NOT active (call Wire.end() first if it was).
void recoverI2CBus(int sdaPin, int sclPin) {
  Serial.printf("[Recovery] I2C bus recovery on SDA=%d SCL=%d...\n", sdaPin, sclPin);
  pinMode(sdaPin, INPUT_PULLUP);
  pinMode(sclPin, OUTPUT);

  bool stuckBefore = (digitalRead(sdaPin) == LOW);
  if (stuckBefore) Serial.println("[Recovery] SDA was stuck LOW — clearing...");

  for (int i = 0; i < 9; i++) {
    digitalWrite(sclPin, LOW);  delayMicroseconds(5);
    digitalWrite(sclPin, HIGH); delayMicroseconds(5);
    if (digitalRead(sdaPin) == HIGH) break;
  }

  // Manual STOP condition: SDA goes LOW->HIGH while SCL is HIGH
  pinMode(sdaPin, OUTPUT);
  digitalWrite(sdaPin, LOW);  delayMicroseconds(5);
  digitalWrite(sclPin, HIGH); delayMicroseconds(5);
  digitalWrite(sdaPin, HIGH); delayMicroseconds(5);

  Serial.printf("[Recovery] Done. SDA now: %s\n",
    digitalRead(sdaPin)==HIGH ? "HIGH (ok)" : "STILL LOW (bus may be damaged)");
}

// ── I2C device init (sensors + OLED) ────────────────────────────
// Pulled out of setup() so it can also be re-run at runtime from the
// web app's "Recover I2C Bus" button, without a full board restart.
void initI2CDevices() {
  tof1Ok = false;
  tof2Ok = false;

  digitalWrite(PIN_XSHUT1, LOW);
  digitalWrite(PIN_XSHUT2, LOW);
  delay(50);

  // Sensors first, on a quiet bus, while they're most fragile (just out
  // of hard reset). OLED goes LAST — its init traffic/power draw was
  // disturbing the sensors when it ran first, causing both to fail.
  digitalWrite(PIN_XSHUT1, HIGH);
  delay(10);
  if (sensor1.begin(VL53_ADDR_1)) {
    tof1Ok = true;
    // Fast timing budget (20ms — the sensor's practical minimum) instead
    // of the ~33ms+ default. A cat-proximity trigger doesn't need the
    // extra averaging; this directly cuts trigger latency since every
    // rangingTest() call blocks for roughly this long.
    sensor1.setMeasurementTimingBudgetMicroSeconds(20000);
    Serial.println("[VL53L0X] Sensor #1 OK at 0x30 (20ms timing budget)");
  } else {
    Serial.println("[VL53L0X] Sensor #1 FAILED");
  }

  if (SENSOR2_ATTACHED) {
    digitalWrite(PIN_XSHUT2, HIGH);
    delay(10);
    if (sensor2.begin(VL53_ADDR_2)) {
      tof2Ok = true;
      sensor2.setMeasurementTimingBudgetMicroSeconds(20000);
      Serial.println("[VL53L0X] Sensor #2 OK at 0x31 (20ms timing budget)");
    } else {
      Serial.println("[VL53L0X] Sensor #2 FAILED");
    }
  } else {
    Serial.println("[VL53L0X] Sensor #2 DETACHED in this build — skipped");
  }

  if (OLED_ATTACHED) {
    if (!oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
      Serial.println("[OLED] Not found — check wiring/address (0x3C)");
    } else {
      oled.clearDisplay(); oled.setTextColor(SSD1306_WHITE); oled.setTextSize(1);
      oled.setCursor(4,  8); oled.print(F("PooKooli BETA V1.5"));
      oled.setCursor(4, 20); oled.print(F("Starting..."));
      oled.display();
      Serial.println("[OLED] OK at 0x3C");
    }
  }
}

// ── Setup ─────────────────────────────────────────────────────
void setup() {
  // Claim both XSHUT pins and force them LOW FIRST — before Serial,
  // before the pump, before anything else. Every millisecond this is
  // delayed is a millisecond the sensors' pins can float and let a
  // sensor start powering up early, which caused the random boot
  // failures (race condition: whichever sensor "woke up" first grabbed
  // 0x29, so results varied between resets).
  pinMode(PIN_XSHUT1, OUTPUT);
  pinMode(PIN_XSHUT2, OUTPUT);
  digitalWrite(PIN_XSHUT1, LOW);
  digitalWrite(PIN_XSHUT2, LOW);

  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== PooKooli Fountain BETA V1.5 ===");

  ledcAttach(PIN_PUMP, LEDC_FREQ_HZ, LEDC_RESOLUTION);
  pumpOFF();   // safety: pump OFF on every boot

  pinMode(PIN_WATER_ADC, INPUT);
  pinMode(PIN_TOUCH, INPUT);   // actively driven by the module — no pull-up needed
  for (int i = 0; i < 4; i++) pinMode(rfPins[i], INPUT);  // PT2272 drives these actively too

  // Hold both sensors in reset a little longer to guarantee a clean,
  // fully-discharged start (50ms, up from 10ms — some breakout boards
  // have their own onboard regulator with a slower power-down curve)
  delay(50);

  // Preventive bus recovery BEFORE the first Wire.begin() — clears any
  // stuck-SDA condition left over from a previous power cycle, so every
  // boot starts from a guaranteed-clean bus without needing the web
  // button pressed manually.
  recoverI2CBus(21, 22);

  Wire.begin(21, 22);
  Wire.setClock(100000);   // 100kHz — reliable with typical wiring/resistors

  initI2CDevices();

  // Boot connectivity summary — same result the web app shows
  Serial.println("----------------------------------------");
  Serial.printf("[BOOT CHECK] Sensor #1 (0x30): %s\n",
    tof1Ok ? "OK" : "ERROR - not responding, check wiring/power");
  Serial.printf("[BOOT CHECK] Sensor #2 (0x31): %s\n",
    !SENSOR2_ATTACHED ? "DETACHED (not in this build)"
                      : (tof2Ok ? "OK" : "ERROR - not responding, check wiring/power"));
  Serial.printf("[BOOT CHECK] Water sensor: %s\n",
    WATER_ATTACHED ? "attached" : "DETACHED — test mode locked ON (50%)");
  Serial.printf("[BOOT CHECK] OLED: %s\n",
    !OLED_ATTACHED ? "DETACHED (not in this build)" : "attached (see [OLED] line above)");
  Serial.println("----------------------------------------");

  loadSettings();
  readWaterLevel();

  WiFi.mode(WIFI_STA);
  // Lower TX power before connecting — full power (~20dBm default) draws
  // a sharp current spike right at radio startup, which is a classic
  // brownout trigger when the supply is already loaded by every other
  // module running simultaneously. 11dBm is still plenty for typical
  // home WiFi range at close-to-moderate distance.
  WiFi.setTxPower(WIFI_POWER_11dBm);
  delay(50);  // let the rail settle after the mode-set current draw too
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.printf("Connecting to %s", WIFI_SSID);
  int tries=0;
  while (WiFi.status()!=WL_CONNECTED && tries<30) { delay(500); Serial.print("."); tries++; }
  if (WiFi.status()==WL_CONNECTED)
    Serial.printf("\n✓ http://%s\n", WiFi.localIP().toString().c_str());
  else
    Serial.println("\n✗ WiFi failed — check credentials/signal");

  server.on("/",         handleRoot);
  server.on("/status",   handleStatus);
  server.on("/pump",     handlePumpCmd);
  server.on("/settings", handleSettings);
  server.on("/toggle",   handleToggle);
  server.on("/restart",  handleRestart);
  server.on("/i2crecover", handleI2CRecover);
  server.begin();
  Serial.println("[Web] Server ready");

  esp_task_wdt_config_t wdtCfg = {
    .timeout_ms    = WDT_TIMEOUT_SEC * 1000,
    .idle_core_mask = 0,
    .trigger_panic  = true
  };
  esp_task_wdt_init(&wdtCfg);
  esp_task_wdt_add(NULL);
  Serial.printf("[WDT] Armed (%ds)\n", WDT_TIMEOUT_SEC);
}

// ── Loop ──────────────────────────────────────────────────────
void loop() {
  esp_task_wdt_reset();
  server.handleClient();
  unsigned long now = millis();

  if (now - lastWaterRead >= WATER_SAMPLE_MS) {
    lastWaterRead = now;
    readWaterLevel();
  }

  // Refresh cached laser distances (runs even with system OFF, for debugging)
  if (now - lastDistRead >= DIST_SAMPLE_MS) {
    lastDistRead = now;
    readDistances();
  }

  // Periodic Serial distance report (mirrors the web app debug card)
  if (now - lastDistPrint >= DIST_PRINT_MS) {
    lastDistPrint = now;
    char d1[12], d2[12];
    if (!tof1Ok)               strcpy(d1, "ERR");
    else if (lastDist1Cm >= 0) snprintf(d1, sizeof(d1), "%ldcm", lastDist1Cm);
    else                       strcpy(d1, "--");
    if (!SENSOR2_ATTACHED)     strcpy(d2, "DET");
    else if (!tof2Ok)          strcpy(d2, "ERR");
    else if (lastDist2Cm >= 0) snprintf(d2, sizeof(d2), "%ldcm", lastDist2Cm);
    else                       strcpy(d2, "--");
    Serial.printf("[Dist] S1=%s  S2=%s\n", d1, d2);
  }

  // 433MHz remote — hold-to-confirm on all 4 buttons every loop,
  // regardless of system state, so C/D (maintenance functions) always
  // work even with the pump master switch off. A/B are individually
  // gated on systemEnabled below since they control the pump.
  // Requiring a sustained 3s hold (not just a clean edge) is what
  // filters out the random triggers — a brief noise glitch on the
  // D-pin can't hold HIGH that long, but a real press easily does.
  if (rfEnabled) {
    for (int i = 0; i < 4; i++) {
      bool raw = (digitalRead(rfPins[i]) == HIGH);
      if (raw != rfCandidate[i]) {
        rfCandidate[i] = raw;
        rfCandidateSince[i] = now;
        if (!raw) rfFired[i] = false;   // released — armed for next hold
      }
      bool pressed = raw && !rfFired[i] && (now - rfCandidateSince[i] >= RF_HOLD_MS);
      if (!pressed) continue;
      rfFired[i] = true;   // fire once per hold session, not every loop

      switch (i) {
        case 0:  // A — timed start/stop, universal override (like touch)
          if (!systemEnabled) break;
          if (pumpRunning) {
            Serial.println("[RF-A] Stop");
            if (pumpSource==SRC_PIR) lastLaserEndTime=now;
            pumpOFF();
          } else {
            Serial.println("[RF-A] Trigger (timed)");
            if (hasWater()) startPump(SRC_REMOTE, remoteDurationMs);
            else            triggerBlocked(SRC_REMOTE);
          }
          break;
        case 1:  // B — infinite start/stop, universal override
          if (!systemEnabled) break;
          if (pumpRunning) {
            Serial.println("[RF-B] Stop");
            if (pumpSource==SRC_PIR) lastLaserEndTime=now;
            pumpOFF();
          } else {
            Serial.println("[RF-B] Trigger (infinite)");
            if (hasWater()) startPumpForever(SRC_REMOTE);
            else            triggerBlocked(SRC_REMOTE);
          }
          break;
        case 2:  // C — manual I2C bus recovery, works regardless of system state
          Serial.println("[RF-C] Manual I2C bus recovery triggered");
          Wire.end();
          recoverI2CBus(21, 22);
          Wire.begin(21, 22);
          Wire.setClock(100000);
          delay(50);
          initI2CDevices();
          break;
        case 3:  // D — restart board, works regardless of system state
          Serial.println("[RF-D] Restart triggered");
          pumpOFF();
          delay(300);
          ESP.restart();
          break;
      }
    }
  }

  if (!systemEnabled) {
    if (pumpRunning) { Serial.println("[System] Master OFF"); pumpOFF(); }
    laserCandidate = false;
    touchCandidate = false;
    if (now-lastOledUpdate >= OLED_REFRESH_MS) { lastOledUpdate=now; updateOled(); }
    delay(50); return;
  }

  if (tankLowError && (now-emptyErrorTime > EMPTY_BLINK_MS)) tankLowError=false;

  if (pumpRunning && !pumpContinuous && (now-pumpStartTime >= pumpDuration)) {
    bool wasLaser = (pumpSource==SRC_PIR);
    bool underSafetyCap = (now - laserFirstTriggerTime) < MAX_LASER_RUNTIME_MS;
    if (wasLaser && catStillPresent() && hasWater() && underSafetyCap) {
      pumpStartTime = now;   // renew: same duration, fresh countdown
      Serial.println("[Pump] Cat still in range — renewing timer for another round");
    } else {
      if (wasLaser && !underSafetyCap) {
        Serial.println("[Pump] Safety cap reached (5 min) — stopping regardless of sensor state");
      } else {
        Serial.println("[Pump] Timer done");
      }
      if (wasLaser) lastLaserEndTime=now;
      pumpOFF();
      if (wasLaser && !hasWater()) triggerBlocked(SRC_PIR);
    }
  }

  if (pumpRunning && pumpContinuous && !hasWater()) {
    Serial.printf("[Pump] Continuous — water low (%d%%)\n", waterPct);
    pumpOFF(); triggerBlocked(SRC_NONE);
  }

  if (now-lastOledUpdate >= OLED_REFRESH_MS) { lastOledUpdate=now; updateOled(); }

  // Touch: universal manual override. Checked BEFORE the pumpRunning
  // gate below so a touch can stop an already-running pump regardless
  // of what triggered it (laser or app), not just start new runs.
  if (touchStable() && (now-lastTouchTime > TOUCH_DEBOUNCE_MS)) {
    lastTouchTime = now;
    if (pumpRunning) {
      Serial.println("[Touch] Stop");
      if (pumpSource==SRC_PIR) lastLaserEndTime=now;
      pumpOFF();
    } else {
      Serial.println("[Touch] Trigger");
      if (hasWater()) startPump(SRC_TOUCH, touchDurationMs);
      else            triggerBlocked(SRC_TOUCH);
    }
    delay(50); return;
  }

  if (pumpRunning) { delay(50); return; }

  if (laserStable() && (now-lastLaserEndTime > LASER_LOCKOUT_MS)) {
    Serial.printf("[Laser] Triggered (d1=%dcm d2=%dcm)\n", triggerDistance1Cm, triggerDistance2Cm);
    laserFirstTriggerTime = now;
    if (hasWater()) startPump(SRC_PIR, pirDurationMs);
    else            triggerBlocked(SRC_PIR);
  }

  delay(50);
}
