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
<link rel="apple-touch-icon" href="data:image/png;base64,/9j/4AAQSkZJRgABAQAAAQABAAD/2wBDAA0JCgsKCA0LCgsODg0PEyAVExISEyccHhcgLikxMC4pLSwzOko+MzZGNywtQFdBRkxOUlNSMj5aYVpQYEpRUk//2wBDAQ4ODhMREyYVFSZPNS01T09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT0//wAARCADAAMADASIAAhEBAxEB/8QAGwAAAQUBAQAAAAAAAAAAAAAAAAIDBAUGAQf/xAA9EAACAQIEAwYEBQMCBQUAAAABAhEAAwQSITEFQVEGEyJhcYEykaHwFCNCsdFSweEVFjNicpLxU1SCk6L/xAAZAQADAQEBAAAAAAAAAAAAAAAAAgMBBAX/xAArEQACAgAEBAUFAQEAAAAAAAAAAQIRAxIhMRNRYdEEQXGRsSIyQsHhFCP/2gAMAwEAAhEDEQA/APNqKKKYwKKKKACiiigAoq/4f2Xv3VW9xG4MJZOyn422MRy0nz02q9w1rh3D1VcBhUN1dr10Sx5TO4n2GtU4dazdEHj26w1fx7mXwvZ7iuJgrhHtrmyk3fBHnB1j0FWdnskEI/H8Rs2iDJRNZX1MRz5GrS7isRiiB3hCmfCHAkHkRTVsMCQFC5ZIbKInXSjNhrZX6hkxpbyS9P6N2ez/AANbmfvcTeC/oYwG9wB+9PHhXAQwBwFwdSbjQP8A9UI7G6AcrysaaiPenLbIinL8QMRmgN/c8qONyivYP8/OT9xocL4CSw/AXNBP/EbX08VN3Oz3BLh70XMTYV9kB0HzBP1p8KVfcu5PxcgDyA0+/OpFxFVu8a6EUtEz15dOtHG5xXsH+flJ+5S4jseSW/A4+25zfBcEQvqJk+wqrxXZ7iuGkthHuLmyg2vHPnA1j1Fa4GbYRLZJt/FJEkeUcqkpdZRCOyneGWQB0GutGfDe6r0DJjR2kn6/w80or0PG4HAY6Uxti0bjLPf2hDEiRod9oiZ9NKznEeyuJshruAf8TaEnLs676Rz5ec8qOHesHZqx6dYir49zP0UUVMsFFFFABRRRQAUUUUAFFFTuE8Lv8VxXc2RlRdblwjRB/PQVqTk6RkpKKt7DeA4fiuI3xawtotqAzR4U8yeWxrY8P4dhOCqBbC4nFsT+ay/Dodt484319KdVbGBwgwWABWyPiuq3iLdf8/2FR3JcBgA2Uw0keL+DVHJYekdXz7EFGWNrPSPLn69hd67eumLsuytpt1GwnemCxF0ZUeBoSDE9fWnO9UFgMpAjUaz5E7UhQwt+JSs7wQQKi227Z0JJKkLJDKCDM+EEDY/fzrli2/8A6mVRsMsac995pIi2lwi4FKgaSZH96V4A2YAura6+I6jeKw0TmZAoHiYgROkail3lEAi2GlgSN80+XrzNCMWjLmO4Bk7eeuldtllcEIFk5sqmZ9hzoAULarbUJ4j0Mkn0PzpwZbcMGcnRQpmWA+96TbJzHu8qjTWJg+f3ypSg3EZj4GEjXp8qAHA6M4a74TObM3LmBp5UhTlMvdLo8jK/L5/tvQuV4YvAD5mBBbWDG2k6/Sl2yXuMubNnnUdSN9P70Ad1KpkZSxHIQTE9Kkpea2iDIF1CwSeknfemAGVjrlgQTEQI2+kU5bEvbNy4paJVZJIB39j50JtaoxpSVMh8U4ZguNEgDuMTm0vKk5ogEHr01rF4/h+K4dfNrFWiupCtHhfzB57it8w7zXJk1ykRAJ32A28qTireHxtk4LGqzI+obmjdQTqN+nXlV1NYmktHz7nO4SwdYax5cvTsec0VO4twu/wrFdzeGZG1t3ANHH89RUGptOLpl4yUla2CiiisNCiiigCVw3A3eI423hrIPiPiYCci82PpW4CWuG4YYPBL3VtT4nYfGecnmf8AxoKZ4Xw9eC8Ny+I4q+FN2CPD/wAo15SRPM+1KKm4ysNwsRl0JHPoKrJ8NZVu9+xzxXGlnf2rbr17CLzEJnYKuZYhRr8zHWo5JAnvHIIOYAwDpy019qkEgofEuWTl15+RFRWe2qm3PiUkxqI9qg3R0pWKZ4HgOYsBoBMnYfc09csX7SEvYeGMjXal3eI4fA8M71Lah20gjXyiqjH9o7t24cuXKDt7VLO3simRLcl98tslkBQ5tY10/wA9KbxWLt2rbM9xVcaqJnU71XWMahzO0ByNcon5dKVhbKYzEBJ1I0nn502czILucaBgLYzkfqJgD061HxPGcQBmItAn4cs6ffnV9a7PJiUZUi2ygHMFnXpWV4tg8Thcd+FvWzniQRqHHUUKVg40WGF4xduMve5WzaatlirbD3xfkt4c2pEiG9DuKyQwl9RLWnVeZy7VY2MPeQgZmYDQDetboKTNPaCQO7tgMZ1HhkE/MVKVQrHOPCoJnMI94I9ao+G49XLYdjDhZ8Rg6VbuwGbKis7a6kNPoBrtWiNUOT3mdDAU759STp7f3pYdLl6F01KktpOk/L0pu5mYszG5lzCRl8LeQjl50tsQh8IXIXIGQ6z9/wAUAdv3AqFCysNB8JgDn+wqFjL6/hzcaIPiLRA/zUq7dDu7/l5QI8PiVumnKqrHugw5ughyT4CDuDvWgNYTHWscP9K4nZY4e6QLFyZZG5f46bbbZ3iWBu8OxtzDXgfCfCxEZ15MPWuYi4XvHO4LbGOXlWlt2f8AcXAslxnbG4QHu3LDx+R9YAn0M71eL4iyvdbdjmmuDLOvte/Tr3MjRRRUjoCtB2RwAvYx8deX8nDfDI0L8uXLfrMVn69CweFHDuEYbChMtyA90GNWO8kb66D0FUw9Lm/IhjtusNefx5nMTdN25r3mYgHKrddoNIXLkFtbaiRqCI0mDHntXWUoRIgkQJ0A15a02XjS3BnQkga9IqTbbtl0klSE4ljL+GQg9So29RVFexfd4gtpp9B0q8uzlZlBKnfX4uU67/tWL4ghtYy7bJJhvpStWMnQ7icZdxrhWMKuw5AUq1YAYI24Oppq3a7pkV113b5VNsKt59DEAyKV6DrUYdCjBlAUSTApdvFvgWtvaBFxeZ5DXl71Pwlu3cW4pIJBE9KiccsvYCW2Hwmcw8+tYnbo2q1N3wTFi6M7kKUtq5PUlaTxxLdw2rihWuBpEamDvVZ2YvWMVw66zNkvLbjTfSod3GXXe73rORIWCdN6RL6hnrEliw7MxtKFs6ZkDchodBz0qux2CbBhr+GjVfhMjTr61bOVy6gqWGyjQ+R68uVcu2muQCGGeROU/KatuQMnmawy3yCxHiBrWcNvrdtrdQmbg0kDX0HpNZviVo4Nit0EFAAgA3Bq07OsHwATKoPeFMxMQCNprBnqW4KhCiI0FZidz0Gs0qYKj4SRIAaPPkJ6Uh7aMoy2jltkHaDt+/P5UuEFpzkyKB7ketaKM32DLF34DGswJ5nSqfiTG4qsNF2BJ3jTSpV/EQ7h28CnQDn7moV4G5BdWPn1o2NSszxDm8QFPTatN2ZuPhwWIMTLCPqNfWobYe3bUkW9T1rmD4g+HuwEBA0KkUJu7QOKaaY52twC2sYmPsAGzifiK7BxvsI139ZrP1tbbrxnhGKwLWybyKblrUTm5RIga6ehNYqr4mtTXmc+C2rw35fHkWnZvDfieO4ZSHyo3eEryy6ifKYHvW0xFy298kEEEFc2YRGx+vKqDsThznxeLyuSiC2kDRp1I8zovzq5vKe6nMZg582mvX6US0w0uepkPqxpPkq/Y2toqQ1sKvLxQPf79KSczAOq28x8JCiQNOZHzpToCWMqyrqpMyR8vlTTD4mgZvMfp6e3lUToGLxBEFQWUTJH3FZi+Bc4i9xhIBEc5rS4xibcgDxjWG3Gvz3rK33CvmmSTOnKg1DmKvo185REbV3C3lVncg6iABUEksc1OG4QoUcudK0Oi24bc7u2VOlx/hEb+dO8WvXGw9yxcTMRqCogQOdVIusyZmuEMTtU3C5rjG1m8SgkmdDNTqnY12qO9nrzo90Lcg5duoqxt94FCXFM5ixIaPQVQWbt3A37qqBLAoZFaDBs91c5MzyOpAA3n2pq1sW/poscIQVcLBj+ldTI59KfLMLrrKbCQo1+4pnCjMsENkeSS8aLqCfIwelSGt3EK21t2wreMoRMecA05Mh4zBWMUhJnJAKtsB9/WncJh7eDwy2bRlGGYmDIbf8AipCgBSe7iI0IzCOnSddqWTmLMgDMNlPh9+kb0AN2mdnnRgwzklZPpNN4pzZtE3JLLr4tRHtz5c6UxOYgQDud/kY1momPVGAQFpLxlzHT1mgEQrFl8SxubTPdr6c6k/gGhi7GV3PU/wAVxcYlu8LeHQso0BJmR68hSOIY13w4QsEzHxECptsskqGL9vLaz5xcMQAKqyWDmD4gamlh3UCVA1jzqIAe/kjnTRFZZ8Hxf4bH2rjEhGbKfFAg6SfIb+1V3aTDfhuO4lQHyu3eAtzzamPKZHtUrug1tFG4HXan+19trtrh+PIbNdtZXIHhB3Hpu3yrow3mw2uWpy4iy40Zc01+yw7L23Ts3cLKQL11mSDuIAJ0/wCk/KpgZksAKIIGUgrB1imeBz/tjBQNc7RBIPxNtT4VCVDk2+Q015/F161uN+K6IXw/5PqxN4hkVbnJJbMdemuwn61HdgLZIDkwYUTG/wDPSnSlsAIGgM2XfWRy85pBu96S7MQ3MeXtzqJ0EDiV653LnNGnhG/rPKspfC5wEzH+9anEGyAqPcyoQBE/TeqnHPhLd1RYBuH9UiAvkKyzUioKsu6keooB2AFWVzHO6LbhbS7khdWpnF2LIw1u/Yu52PxrEFf5os3YYJDqoVTm5xtS7TMHIXQgQeVPWMRatYYLlW4T8UjUe9NYksFDjRWO1L0G6ky5auYm0MURvAMLoSNJq9wVnLbALGFWYiJ56VS8Hc3VFs3QqIZYNMR1rTWglq3HeFcxywY38o3oSFkxy0V7sWwpkr4WZRAPOen70tENss1x0DxlWN6SqIuZsVmXSVAEjWQJ8486SUCuQbmoXQjc+ubUUwoo3rpIZ2AUAZSslfU1221t3dnyEbaA6+f30Fc7w3FJGqh4ABJLBukV1tAZw7NcJkBSPmY9frQA1dAEHuUBaCw5eXpTYIe4zeIMg3Gw1jQmlX2R5dwEAiBG3y2399KjMBaZ7xLzBJBM+lD2NW5Bx138MxFsKrE8jJ9TUK03fXM91iTymo+KuM911aPEdTTtkKq7mkoeyQ5yqQGmd4pi1JMHeY9qWSMrQd+ZrltZJBkEiCa0CbYVriCInmfpU3j1lj2Vtd2kCzeDOCfhBzf3YfOo+DuFEQhQZ3/arLjKley2MBbMCyET0zLVfDvWS6Mh4pfTF9V2F8CWey+EIMFS5B/+TU6zflwzFMpB3gLJHP1neofZZmvdnXW5cIFm8wQgkQIBMx/1H7FTEZSocaBv1Ax5Aaa+VUxvxfREvD/kurApbVmzKhgktzjznkT0qJirFzFMLZutZs5TmNs6uQBA6DzqYGysUchbY1AzAxPOeX+aaLtEPGWTsu8bR8t4qJ0GNxa27V4qgzNERJOtJZHt4YC5mW4rggHTKD9itBwnhTJx5Ti8i27b52NwwIO0datO2XCuHvhTjbGKyXEBBtOuUuAYJE7wf3qTxEpKJVRbVmJtWTeUMSWM7eZE1Z8O7P379/IxKE2yYYfQ1Ydm8TYwrXUsZHvnKUNyVK8toMg+XSthYwBRLZtA3LxWGcnXbU67UksRqWWhowtWeW38G+GaYgqxVgdYIqMwa4wJMT15VdcT0x95IBVrhX113qsuKGuosE8iBVUxGi87N4UQ11lQ65davUKgIQrtqBIB19/eoPBLIs8OYCQAZJC8iNtdzqKsLzC2AxYvmGYAIPlt5CmEe426d45t99OaSoLag+n3vSrZa2Dba38QmFjf+a6LguCbl2EU+EjXN961y5efvGe0uipM5SQdNgRznnQYOO6wmXKJYZlBABE7jlTZZWCnLMGLc9fXrXbpZkym5knwmRoG35eXPemEtlf0yy6nM0CPKgDlw+Aoo8BObxcx5TPSq7GXiMMbcjNJJI0HyqwuL3SkgTczSSYMDePP6VR8Rus5EECRJjStAhKMzE6etLNwLs2+2lX/AGTwODvjEX8UMzKuRFYSpkdKhcVwOH/FFVS5Y1iYkDz1pW9aHrQrVuBnEkGnVBEFTzqvxNu5h7zWnEMhjp70/hMRmJVxuIoNTL3hoByAr9dKseOCOy2LggjMseQzLpUPhYQWQGMMdjyp/tUz2ezyIvh728q3BvIgn91FU8P90n0ZDxX2xXVdyF2Jvy+MwbS2dA6qdV00PzkfKrvuylue6Az/AKgvl0/blWP7N4n8Nx3DMS+V27sheebQT5TB9q2j2hbxFwAC0NIZYWRpt6VSWuGny0JQ+nGkuav9EZQ1nMUzqm+ZmiPc/t9KRdm5cJLE9FMwQT1MeulLJbIXW5ItsQNBrrofLf6Vy+rMS90CQIMGYn7/AHqJ0Fgtl8VwxMQFZzbLWzlAYx1I5isvxPhGMxt+1bwVlriOf0A/UnatDwPiP+mXnUh+6c/DzX2I3/mrlk4JjUe662lLfEyXMp9xXPOThK6LxWaNGMfsjxMY2xFi5cEKDewxkDqJGxr0JOHfhsNI8IFseEtMRynp/eq09pOF8Jw4XDWbzhAfCsQY0neszxnt7fxqtbw9o2Lc7fqI86RucxklErOKItvGO+XIzMTlmYnnVRasPiMSIgAkxPMVy9jvxF4vfJIP9NXPZ/DF7iYgqAskDUa6cqvFNE5NFxggLOHS0t0WtI8R0PtG+/Oi5CW/E2eRIDLKwYpVohlW6rXO7FvLJU6AHSZ1pxrgChT3eUDwrMyTvr0iPnTkhBIt2+8ARmXYTJ+X3603eIDqLXxA6BWknpI3jWnu+y4Wb0AGMoKgFZ0mmYshie7W2Vg5sswdeQkTH7+1AHXWXUqyArqisdCI1OnP6elIuZlJtgZUnM0PvA+mx0pw9ygAklYBgEkKSOR+96btd1mIRlZyIYM0zvp9a0CJjO8W1mUsUCyq/CAY5jbzqBatI5uPeEgNCkydfT7FWj4W5i7y2sKlzNcMgaEHlJ8qteIcIHDsIrWbYBsBSzgaMedZJ0jYq2VWE4diSAMJdBe6RmVGhvuNfSmMThypUJedlUkBxsxG8T56Vfrxe1YNi2xAN25Kt3kBBmzEsu+bkDzp5MCnE7tt7eVUDEKiR4ZMx8qhGctcyos4ryM3fS3iFWzjLHeyNCRBHoaq73CVtp3uG70qG1zAaDlrWqxmCuWI760bJuvCWokhRzY+fSr/AIRgsNdtW2MKSpDjcN5mmc6BRsyWBwptWUUnMIjf4TUHtrfKtg8FmeUQu2vhM6D30b51qr9rBjiVy3gQDYBmJBHtHKsD2kxP4njuJYF8qN3YDcsuhjymT7104WmE5c9DkxmpY0Y8k3+irr0LB4pOIcIsYsvmfKLd46SGG+mw119CK89rQdk8ctvFvgL7fk4oQsnRX5b6a7euWnw9bg/MnjpqsReXx5l4r5Xd0AgxDLC6xzArqMUVUUKCYMKSzAeg5/zV0OArctZ7F1fEBqw1BG/1qLd4JjkZ1UBreuXLrPrUnFp0y6kpK0VZLm0Y1I6Hn7wJ9KjNYJYsLYBI1EaecedT7mCxaj89GVTtlBGUAdPPXnUe8zd84gLPwymgPSedKaVt/Ah0hLRfk5ZisVVY7Cph8O2UAgfv61pot3Li+EKTqcp1JPT1EVWcasG5hS2YnxAR0+/Og0zSk5cnU1uMBhQOH2LAgGZIQTrAOvyrNYfCC4yqySeQSPrWusszXSltCSBBAUGT/O2nlQax3Ke57xVCkSYiCPbnzpBtEYhiyr3ZG5M5hvGWeVKtq123mEssFY2K689P/OlNoe6QsLoXm7GfERrM8vfagUM6NYFsd34SNGjfeD101jenTcRbyiShjWFEesj/ADTdoEhiLcvyYgGfLQGPauMgQMchnT4pkiI585n6UAKuMp0GVV1jLy8wpG9M3FBcM7TbTnBJ189/eaVcV4LNnugQ4HhGXlodNelcgOEz3Co3gqQJ9DtWgaLszg1uPcxUaj8tN9Oo1q7xuD7+zctqRDgDTcGo/ZVFfhSEEZQzbcqvhbGQ6VGb1Hjoee8Z7NvYtYcgPduPcIYKksdJG3pTOCv4zAE4PJ3ZuCZc5CBpInkTtXoBwiC+L6aXACsnmDVRi+z5xeJe6zlZ66ilbsqq8itxN60cO9y1YuFQCWVrRQAQIGu5GpkVWYS3iMTi7QRnW3JY5WgEQauk4bbt4jue8ZWyzrosVLw2A/BcPvYm4AMqlbYjroD9aTChVRjqbiTSWaRj8TiV4bwW/iicl0qUtmBJY7aHfXX0Brz+tB2tx4u4xMBYb8nDCGg6F+exjTb1ms/XoT+moLy+fM4MK5XiP8vjyCiiikKnp3ZLjq8QwuW4SLtsAXp2mNGHLWD6fvpc1eLcNx13h2Nt4myT4T4lBjOvNT616pwbi2H4jhEuWXlToJ0IP9JHI10N8RX5rfucq/4yyv7Xt06di2LEiCNDUW9gcHfUi5h0M8wIPzFPUTNROiyvvcCwd8HLKZtCDqKhYnsu72Dbs3EclY8RPTznnWgWetSbKzWZUwzNGGtdlsXg+8dbTEj4DEj6Vy2lyyoU22Vj42EEGR9a9KtJ4R59KU9m2/x20b/qUGaxwQcQ8zCR4rZOiwFAygR/bb/FJW5bbKGyxJYnIXB3kb/St7iuDYC8GJwwWSDK6bVSYrs5hiALN50IBC5tY96zKxlJGbItMBKiGY6QZ6z5ctulOyWGmqjdmbbrJJ+4qx/0O+qsUul9wSX+LlPrUN+HYjDwgw65WYIRkjSNwR50oxGcAEtBzEgyIBIkyTHU/vSbgU2wGZYH/NmI06culdylIcq8CP0jxT108xQ3ikFEneG566zvtRYFh2W7Q2OHcQPD8Sctm9BRwIVT6cgfuK9FUhlkEEGsv2W4Vhf9PXE27VvvL8tcYASTtUu/bv8ADLxFrEXBac+EHUDyFRm1YyVl2SA0V3IGBB1B3FRMBixfXI5/MG/mOtTdBrSLU12tBgYOwrhzbkjadYrFdu+0qYSz3VgyxzLZy7Fo1Y8oE+/odLjtR2iw3DsHdUuAoGVyNST/AEqOZ/b9vGeJY67xHG3MTeJ8R8KkzkXko9K7cGHCWd7vbv2OPEfHlk/Fb9u5FooorC4UUUUAFTuE8Uv8KxXfWTmRtLlsnRx/PQ1BorU3F2jJRUlT2PXuD8Ww/EcIl63clTpJ0IPMHoasq8ZwHEMVw6+LuFuldQWWfC/kRz3NehcA7T4fiCBGi3eBjuWbX1U89PlVqWJ9uj5djmuWDpLWPPl69zUJUzDjWKgWbiPqjA1Pw5gknSlqtGVtNWixVRApdNI3hmllvOptAmN3iAu1VeKuBMzE/CJqwvuAAJiqu9+Y6IdncT6DU/tTbI2JxUNu0incDX15/WiSPSnLhnfnrTRI2pWOhD2bNye8tIfUVEv8Iwd7ZWQ6iVO0iNqm+8V0UoxEweDxfDreTAYzKszkdZFM4p+NXbi/iLqXLQfVFQRH71Zg0m5ct21m44A86Xh5noDnlVszdzEccbihw+BlEUSGuW/CvnNKbtHxLCcOe/xfELbRQVARfG7eX356RXO0Ha63w7D/AIdCHuE5e5VhmjfxH9Ij5z0284x/EMVxG+buKultSVWfCnkBy2FVjhRwnc9Xy79iEsWWPpDSPPt3HOLcUv8AFcV3145UXS3bB0Qfz1NQaKKxtydsrGKiqWwUUUVhoUUUUAFFFFABRRRQBo+E9r8bg2VcVN+2IGcGHA057Ntz361uOEdqcFjcmS8hZv0TleYk6Hf203rySiqrGe0lZB+HW8HXx7HvuG4hZumFuAE/pbTflUvvudeF4XtDxXDQFxb3FzZiLvjnyk6x6GrnB9ucTaATEYfQtqbNwrA9DMn3FPeG9nQn/aO8U/T+nqN+7mnaoqEHElv6Ej3Y/wAD61jrPbvB3Lyi4b6KZlrlsQP+0k1KXtnwtSxGKSWMn8p+kdKx4d7Ne5qxq3i/Y1TGaaI8cxp1rN/704Z/7pP/AKn/AIqHf7dYJLrKnfuv9SWxB/7iDS8LnJe5vH5RfsbGm3xFq3IZhPQa155jO299wVw2GAAbRrzlpHoIg+5qnxXaHiuJkNi3trmzAWvBHlI1j1NGXDW7v0DNjS2il6/w9E4n2mwWAVg95A66ZAczzEjwjb302rFcW7X43GMyYWbFsyM5MuRrz2Xflt1rOUUPFrSCo1YCbubv49goooqRcKKKKACiiigD/9k=">
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
  width:100%;height:200px;object-fit:cover;object-position:center 30%;
  border-radius:20px 20px 0 0;display:block;
  filter:brightness(.92) saturate(1.1);
}
.hdr-text{
  background:var(--card);
  border:1px solid var(--border);border-top:none;
  border-radius:0 0 20px 20px;
  padding:14px 16px 18px;
  position:relative;
}
.hdr-text::before{
  content:'';position:absolute;top:0;left:0;right:0;height:40px;
  background:linear-gradient(to bottom,rgba(7,9,15,.7),transparent);
  pointer-events:none;
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
  <img class="cat-banner" src="data:image/jpeg;base64,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" alt="PooKooli cats">
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

const char MANIFEST_JSON[] PROGMEM = R"manifest({"name":"PooKooli Fountain","short_name":"PooKooli","start_url":"/","display":"standalone","background_color":"#07090f","theme_color":"#07090f","icons":[{"src":"data:image/png;base64,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","sizes":"192x192","type":"image/png"},{"src":"data:image/png;base64,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","sizes":"180x180","type":"image/png"}]})manifest";
void handleManifest() { server.send_P(200, "application/manifest+json", MANIFEST_JSON); }
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

  if (pumpRunning) { delay(50); return; }  // skip PIR re-trigger while pump is on

  // PIR sensor
  if (digitalRead(PIN_PIR) == HIGH && (now - lastPirEndTime > PIR_LOCKOUT_MS)) {
    Serial.println("[PIR] Motion");
    if (hasWater()) startPump(SRC_PIR, pirDurationMs);
    else            triggerBlocked(SRC_PIR);
  }

  delay(50);
}
