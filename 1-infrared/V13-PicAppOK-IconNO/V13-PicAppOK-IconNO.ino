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
<link rel="apple-touch-icon" href="data:image/png;base64,/9j/4AAQSkZJRgABAQAAAQABAAD/2wBDAAwICQsJCAwLCgsODQwOEh4UEhEREiUbHBYeLCcuLisnKyoxN0Y7MTRCNCorPVM+QkhKTk9OLztWXFVMW0ZNTkv/2wBDAQ0ODhIQEiQUFCRLMisyS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0v/wAARCADAAMADASIAAhEBAxEB/8QAHAAAAgMBAQEBAAAAAAAAAAAABAUAAwYHAgEI/8QAPxAAAgEDAwEGBAMGBAYCAwAAAQIDAAQRBRIhMQYTQVFhcRQigZEyocEHFSNCsdFSYuHwFjNDgqLxJHJTY5L/xAAaAQACAwEBAAAAAAAAAAAAAAACAwABBAUG/8QALBEAAgICAgEEAAYBBQAAAAAAAQIAAxEhEjFBBBMiUQUjYXGRsTJCUsHR8P/aAAwDAQACEQMRAD8A5VUqVKkklSpUqSSVKlMNG0S/1y57iwgMhH43PCIPMnwqSiQNmL6aaP2d1XWjmws5JI84Mp+VB/3HiumdnP2dabpwSbUcX9z1wwxEp9F/m+v2raLEqoqqAqqMBQMAD0FNFf3ENf8A7ZzDTf2XHAbU9QA847Zc/wDk39q0dl2E7P2oGbEzsP5p5GbP0GB+VawxV4kCRKWchVAySfAU1VT6mVrLD2YFaaPp1sB3Gn2kWP8ADAo/SjVjVRhVAHoMV9iZZEV42VkYZDKcg1aBVnUEZMH3xPIYsqzAZI6+lUXGkaddA/EafaS//eBT+lXI8IuSFUB3JBPmR/6/KittRsS1z4mWvewPZ68B/wDgm3Y/zQSMuPocj8qzOp/sqIBbS9RB8o7lcf8Akv8AaunFa+baAqpjBY6+Z+ftY7N6tohzf2UkcecCVfmQ/wDcOKVV+lSmVIIBBGCCODWN7R/s80vUw0tkBp9z/wDrX+G3uvh9PsaA1/Uet4/1TjlSmWuaBqGhT91fwFVb8Eq8o/sf060tpRGI8EEZElSpUqS5KlSpUkkqVKlSSSpUrY9jeyovGS/1FM2/WKE/9T1b/L6ePt1NELnAi7LFrXk0p7Jdi5tZKXV7ugseq44eX/6+Q9ftnw6vp1jb6fbJb2kKwwJ+FFH5nzPqa+Wy8DjAAwBTCILkAkZPQZ61r4CsanOaxrjk9T1F0ohADS3UYL5twtmxGyAYUfOrA5yD4Z6V7t76RZu4u4HRgOG/x+fH9jQlcjIhK2DgxkAGHBB9qU6ssq3sWx5xFLE0bCJN3iCc/Sm0KxRwho9ixAZ+Uce9eoZY51LRtuAJXIpasUOY1kDjEVabY/BidVLd08m6MN1AwP1ott/CR471+Ez4ep9qJmKRRtJIQqKMkmq7BTte6kGHk4VT/IvgP1NWXJ+RlLWAeMTarE9tte3BZoCGQeLbev35+9ObeWO4gjmiOY5FDKfQ0NegMc+VDaM/wtw9kx/hSEyQeniy/r96Y3yTPkRa4D4+40G1phEPxlS2PSg7eedpSs0a7CflZeo9686pb3iXUV7ZZYqhjljHVlzkEeor3pzNdQt31vNBtIxvG0mhAAXlCOS2Ov8AmFMmKHlXijGFBXNxDCjl5ogVBOC4BJ8h60KZJluABuLNStYbuB4LmJJoXHzI4yDXKu1XY+TTN93p+6W0HLIeWiH6j18PHzrrFvdR38DSIrKVOGVuo4zQlzHitJqVxxbuZBc1R5L1OD1K2XbDsuIA+oafHiMczQqOF/zL6eY8Pbpjaw2VtW2DOtVatq8lkqVKlLjZKlSitMsJdSvorWHhpDyx6KPEn2FWAScCUSAMmOOyGgfvS5+JuFzaQtjB/wCo3l7ef+tdStIsYoLTLOGztora3XbHGNqjxPqfUmidN+P1BrtLQCOB8xF5FwQB1x5E8/lXVSsVJj+ZxLLTdZnx4EcwpgDigo9Ou2crcObmJJGeEGTaoBOSD4+NHWyGOJIzI0hVQC7dSaLTpSS5XqNCAxRd2t+ZVeKNYkAwRBOxPvg17F3PAvc36C6tujOv409Tjr/WmU88cGN7YJoe/tA6/EIMNwH2/wAwP9qgfOAwkKkZKncL08x7MxzCeNzz5e/v51dYwR2am1Q9CzjjoCeBSyNfhmTaxRuQUPQmnNtMJ4ww9iPI0i0EfsZopYNryIBfZvLsQIw7uBlMi+LORkD2A5+ooxyUG0jGKQ9mrkTXl3Ox5kmkb88D8hThnmvFMkMiRx5IUMmd2DjPWisQqQp6EpHDgsO5TL82aW38MogM8JAkhcMno3h9D0pqIncsAArLgEsDt+nnQVyHUzQzKodVDKV6MPA+nQ0ys7xE2A4yY2srlLy0huY/wSoGHp6VYGDDKkEdOKT9lHzp00X8sVzIq+x5/Wib66NsVihAUEbQB/L5n9KQ1f5hQTQLR7YcykXpv7yW0XdGiZ3+ZHT6c14n0exf5khYMpyCjkYq6O/t44i5iCStxsjGSQOmTVZuL82sk6W8R28pFzuYeVO+QPx0In4kb2ZPh1hUhFxuOWPix8zQk8WaB7+/vb2G4hHdZdVkhXI2r4lgR5fpTW4khhI76VYlIJ3N5DrTMFDEHD9RVLDXMe2nZ7913PxdqmLOY42j/pt5ex8PtXUILtLvcqqwZQC3yELySOCfahNUs4by1e1uQDFcKRjIycHGR6g/0o7E9xcHuDTYaX5DrzOKVKK1XT5dLv5rSf8AFGeG8GHgR7iha5ZGDgzuAgjIkrc9ibAW1qbp1/jXHC8chP8AU8/QVj9NtDfX0NuOO8bBPkPE/autaFBIkjTwkww26HdIBxGpBA/KtvpKxuw+P7nO9fadVDz3+0e6TJYnuI4kd7kL3hnzhUYr0PsD9/WqbRpBCkcrhypLZA4yepoZXWW2iljhkhtxuWLc+TKfFm+nSroW2DJIGPE1p49mY+egIZLdx2q5l3bipZVVSS2McenXxpjC4eNWXlWAI9qWmeG3RZZZEjVeVZjivq63YoFzK20jhhExB+uKy2Mo7mmpXbYEaGNHkRmUMwzjIq10JjKoQCeORS611zTZm2JeRh/8LnafscUTHqFjPKFjuoZJBnAVwaRyH3NHtsPEGvIpLVhJK/fWrfKSR80frnxFEWE/cTJBKeZOAfM9R+VFsFljZGAZGGCKSSLIkRU/8y1cbT5jqKev5i8TM7fltyWI45W0nWLu2bhRMxU+jHI/I1p9MvY+5EbMBg/KSfM9KA7Q6Z+8ljvbZQ0yryv/AOROuPceFJLKeVSYwS23gqeGHuK18Vvr/XzMZdqLT9GbvePFgPHk44pDqV13s7kY5G0Ef4R40LEJ5T8qhfVzjFMPhYJItiSd6z8SyDoB5CkrWtRydxz2tcuAMT32eU22kmUjmZ2lx79P6CqxmeQuRvx8oHmf/dXXtwLa2Yjqq/Ko+wodoGC2lpGxDMMyEeGf9mqGyWPmWTgBB4nzurhGZrVBcXIbDuR/DRSDwnIyRTSximitEjuZDLIM5Y9SM8Z9auiiSCFYYgFRBhRXvpSHs5ampK+MpkU0s1WyS8jUMBvjbfGxHRqaydKXSrcd7I3eKUH4F24GPU+YNXX3Asi6V4Vu0LwpBcSRBGVHJXgnGM4zn7jGKHkiiiZ2WBTJKwJfJyp5yfr5fWj541Z1kKKZUBCsQRjP+uD9KWuzx4SdkQsP4ZcdVxjPqc5rYkyOSTMl290sXViL2Nf41tw2PGM/2PP1Nc9rslzsljZXAaNwQwPiCOa5JqVobC/ntm57tyAfMeB+2Ky+srwQ48zZ6C7kDWfEfdibTdLcXTD8IEa+55P5D866lo1q02lJbMCIrmcs5A6ouM8+XFYjsrbCDQ4DjDyFpD9TgfkBW902UroXfKyt3aSx7GOAOc/rWgKUoUCZWYWepcnxA7mcXdzIYsiGMfw0P8q8Dj+tWwc4z9KEsESR+6d3R+671FEZO9c4J/I0UXklullM8jJ3YUxt0yOhH08sdTTWwPiIpd/JvMs1LTrLVLRodQgWaMfMM5BU+YI5Brm2pzzKy21rFhYRsLHIyPIA9K6esvdxs54Cgk846VxrU9Sn1CeRgzLEzsdqnAGTmuZ6oDInX9CTgwiS+bu0inkMkanIjbkZ8fY+1UfvZlVUhUqq9PMHzph2Z7MS61NtaQKM/MDkEZ6H1Hr7VupuxFtaRkRxZcxH/uYc1znsVddzqKhPnETdktdv7dooMkwyEkCQn8RHH0zRV12zdZ5IyIllGUkkPIYjwArLtqDaZdrvXc9qzNjkYPP+lKI9029yflRS7sT1Y801GYdGLdEbsZm7sv2gSoBvtomhj/EA+049KK/4v03Utzz2UPy9G747m9AQv9TXN5LV1h3x5O8AkjjFeLZ5Vwu4x9ecnmmC112DEt6epxgrOwaZfaPdd2UjdO8OIzPko58lbJBPp1ovVNah00MmxmdBzwFRR6kkflXOdDtF1iJIrRE/eBfYTnKbccsw9B49cgY8a18j97o4g1Axy3FqSjyE9cEjOT1PrRm+xl3FL6SlG1/EDuu0st4xWNY0PBB27gfIdTQD9rb6CbKz75M7d23PND3jrKSLdQkIGFRF5PFAx2pWeSTZ/wAlgrNjhc/7/pQB2PZj/aRegJqP+Mr5JUSR497DG3bxnPApRf8AavUbq5ZPiGj5HHQUocd7cO4JGzjp0xmq3ysqxyqC7DcB4nONo+/FTJl8QPE0dh2kvolLG+KQqOe9XcB69f8A3Tq27U9+mfiYZCxwO8t2iAPllSf6VjJrkQtHAqq8KfjUj8fmab6BaWi3CtHLsjnG9VzkZHNByKnuEUVhsTR2WsrfyyRfDzRunVtpaNgRkENjHNEwQw3FwyTsQu3K4bByPAe+Tx416OO74xySTjxPnQ2cTxtgnawOF68V2KQTXknc89eVW4gDWYDfK8bMZpY5GYkho+PHjIwMH0rB9uLJo57a72kd6pQ5GOnQ/UH8q6U94Y7qOG3MHxJb5nkj/iw+Hsfasl27t+90+4HetMbZwQ54zg7Tj05o7cvUVx1uDTiu8Pns4/mHafD3VrBCo/AioB9AK0mm9zpclxZX022S4X5o1RmKYB544ORg5HTxpHEF7kMG+bdjb5Dzplpli8pF8A0y2yOsimQl2G35Sc9fEfQU24fHHiK9Pts+YNbLbvLLdWiywIXaNUJyWX/N489aYQUKYzAiqwwXZnAZcHacYOfaiITUOxKHc+a3CJ9MaJtxWXjAON2Pbn6CsR+5ILaZ1mZRAzE7x8oGPL/WmOt608V3P3d4u1CAISCMjxIIoW3wlw01zH8ZaP8APJsXdsXx+1ee9Q7M5M9R6atUrAjrT+1+jaPDHb6dYX10CwXcNoDOfAc8mtPpPa/StdQSQM6vCGMkUq4eMdMkeXt51hdW0IarYAaZdw8TF4334EisOjeKsOeo5r5AkfZKSO6uS0rxWrxMwYbpnfAAUHnaAOWxjoOtZQ6t8R3n+o8rvJ6iztqbWTVpTY7CsyYBHTJPJ+gFLUhEYGnqrGedwvHJ68/kKpkjluLx7kDYqkFIx0Az4D/fWmNjpsFzPZ91cM0twZUkxn5XAyBu9QT9jTyRWmT4gj5tqaBezc2qBktGi7mHgIOXfFZnUtIkW4MKoQ4HOfDFeNGs5JbyUR28lrJaoxklVyChA4PvnNaxHvGWT94vKZVTBeSLLE46A+NA1gU4G4SpzGTqK+wmorps+ozPCpeGAD8OMPuAAH50TZtPdDY0jusrlio5ySf6eFfb6C2fs8dRhUW8yuEdQfxL5/nVPZh7q5vA1jEVMaO7SuvBAUnAHieKfy5KBE8QrExzpVndRa5aC5tmNscrJgZ2epPvQ9pYloe1FvOu2SO5Cpz+LHK4/L70JJb3G7RZpNYv3vL8mRpUkYLEAm7gDjqfsDTeeCfUNMuXd8X1tdrFMUGBNuVSCR54IyPDBpa2KxwIRUjZmQgUNcyhmAEkqhvHAyCTT21t/i/2gW6BSLeGHvWAH8g5H54pVe2zQ3twqjacZUYyetP1ElithZWswhvbu1+Iubtly0UeeFX1zRswUZMoLy0INe6de3V7NItmUtWc7FaPadvn/wC6TajbyKJIoJMeQB5UDxoi2a+OlT6lb6neLcQyHcjuzbwGAwQeM8+HjVV98RYahKupxsoZ/lkQYVj9uPahDhiceIXHjNT2YvjcaWkMpPxEWd/+bnqKOkDFWOScHwUcA+Z96zWg3gXUo0UBVcFcCtOVLNjBJcbRgZJ56D6jwrtekflX+08966vhbrzBpGtwLlmHeXEwVVcjOzI+b7ZFL9Zto5NOlhTAZ4GVlAOFOMDk9c4z9aYXYuVcW93YLalkEhIk3Eg/KBjw4Bz60MqM1tCZPxGMA/Qlf0rWoDDvuYnypxjY/wC8z3YuvcOUhiabIYPLLtwviVHmOvjkcYqyyvDBIxaTbHMDHIc9AfH6H9aVaPdFrS0nUgP3aOCRnkAfrWkgt01iOQRIqM+FkaVAWOSOQ2QM+oGaFyAMnow6wSeK9ifIbWYLJahQk8MpYK3SQEdQx/pUP/OY5BQ4AATbtwMEdefei49WcC3tZopI5IpAJG7wglBkZOBnpzSq7kW3eZY5O9RCQjAYyKVyIyX1He2CQqbzOda3LJbX1x86tKJGU5PGM1LLXXXlhKjYwRGQAw+tW9pCUvpHP4ZMFt5HLfSk8MKt80ZAIPVCxxXFIDT0QJWM9S1uV0LY7iU/gUZJHqD4UDLFeX7SXTuzlQWcp820euKfwX+lwmNXtYu7XHeBZWDk+OX2HPsOBWh0PWOzFrMHt0NvLIcM7XTExjw2/L1P69aADiNCESWOzMZBdRW+hP8Aw2LyuNtwW46HKkeGT/Sr+zWvnSLqZbhRJa3AXcJFJTg8MB/Q9RiiO2MlnD2lmvbSzQ20wXadvyb8fMy+HXz9aGtzBqm43UyQN4hyVLA9dpwV8uDV6Ik2DOm6XfWupIrFLYuvJ5OGI8cZ5pV2k1JCgldmdIyQ2xAFQeYyeT9KQaxpcnZ22guLaWa4tyf4EmOmecEj5fqD9BSO6v7q9KvOqKRyWwMGsqenCnXUcbNfrCrqYXVgbZX3iQ8LnG0g+I9qNsLi/sHie0/htDgphcg0lieWaUPH/DO7OF54HiK2ujTwXFmlvMsbuPwl9yH2NahoRB2ZTba0kRHf2jQxhtzLBIpSMk5O1XVivtnHoKrtdWNtf3UViGNrcS94nenLF+hYk9Tg1fqmh3iEi2tXnPA/hp8q55wB40107srLqOmY7yTT7mMnYZIgctg5JGfM9fSqVEBJA2ZCxxgzN69eRC5M4GZUUAt4HHNJ9L1AahrkNxf3M8CxQmONoFBYYzgYPDDJ5FMtO7P6nrWqzaT3rQrZMRdyuCUXyAB67uSPTmre0HZZtJmRLGOe5aMbklABz0zkDkfarwCMGX0dGXNq1pA6lIprt1YSKsqJHGGHRiqZ3Eepx6Ulvru61QzJOu8udwI5w2aLstJub6TeLd4yozKkmU2+x8KOlshYrG1o6M4ILAkux9M4oQFQYUYlnLbME7M2csGrxpcDDopbk+GP9a39vEsQhulkzKVYxqBn7euM9eOazehRSW+pS39yO9Yrzx4npx44pozT30uRIu0MT3YQthQMe2OcfWur6Qcq8Ti+uPG0N+mp9uLq3mUT3c8jtEQmxIy5l6nAb8vuaBZGh394EVslyqNuUZ5wPSn19aSWFnbSBU3Ird58q4UnGBg+h8OfpWb1WT4fS7uXpsgcj7cVsRhgsOphsQ5CnsxL2HmW4sIojEs0ilokEj7V3ZyM/fz61utOsGNrNZXDNDcSDMceTtU9eD7+Ncq7EXjQ3skCsVZsSRkeDKfD6H8q6giRxX1pLctlJ9zs4b/mFl5IGfOkKxepd/8AhNLIEubI8/3E+n3klxPI853Mx5LeIHH6V71Qs9uXDKuDjGec+AoCVvgZLh2GO7dk49+Kv0+5jlPdyIZC3Vi2MemPGk/iFgACLNH4ZUSTY0yN/BLcTOCHdSMtkdD/AE/rSK6U2shRjkjoCD+v9q7UdFtrix225aOQDK4jJKn1HWsHquhSfEyAQmcfzd2RlfdDz+dcb3+LYPU7ftchruYvvZA24lufAHrRtlZNJcJ3jfjIGTztU8Fj7ZqX1p3DYt1bHirAg/QGjdF1a2t7qJriLaQcOSucjwFP5ZGRE8cHBnQ7/R7DVOzz29tsd4UKLIo4JX1Nc7gsJLV3RwRLEcHa+A2fI1u7PtDZzJ3VvKiD/CV25+lLruGxmuhcPs4Pytu4pAbjqOILbmWvhP8AJFIyiNiWQBQFHuR196ommz8ojVSPxEePFaDV1tLlAiOdo5wgyaUPbAmNFfeXO1V8R6n1o1cHUBkI3GvZqwMwErjk9M+Vb23tHaFQn4x04A/p/el3Z/Tu7jQbfAVs7K1VVHGKaRkRYOIuSCXH8ZFf/tBH55q4zfCxFbW0bcevdqqj702e2UjgChJbWaMZhwcfyk8GqGu5CQZndPk1Kyv7y4miDC6ZSyxnlCBtHXrxVuoQx35WSSCFj5yQrkflmjma5kdk7naTjkiiINOlYA3D7m9qrOeoWMbMV29ugiZI9y7upUgfTpj7ivdvpECtvZA7nxZAD9xin8dnGn8oz7V7aNFHCioE+5Rf6iOXTYjbyRhMZBNZ/RtUUXRsJz3RJIJA+dgBwoPhk1rrp1CnJwMVyTVWxqtwi5xvO3PWtFdhTqZ7KhZ3Nzd3STRMzOttA+Zooy+9mYYU9OetZTtndiLQJQCc3DKikjGRnJP/AI/nTXQ7z4Wzt7mY5mDloUIBBA4JPp5YrI/tDvmmv4bYvuMamRyOhLdPyA+9dB3xUSOjOWlWbgG7EzWnXklhfQ3UTFXjbOQAePEc+ma63Z3TLLDHOqmO4iSW2kROCNuTjOcAgk4HQ/WuOVsuy+rR3GmxWFw1x8TaTB7aRDnu06nA9D9Oay0PxJBm6+vlgjxCtfuzPftEikq3Iw2STjrmjuzctjZzAaq+1cZVckgnwobXrexktTeaYDFLESXhfcXHmSTwSTkjHhSiaQ6hGkMYVGRfxMeBWS4MWPLzNlJUKAviarUNWmsNUmuLKMSxn5l+fcR9qDuu1kurOIZbYQMDkiVVDH2Jwc1lRFdWzLFGw77JOQwIb2NERavcFxBeMQ+MF3IbA8MeZrMaRiaRZuWXsPfTFd5kDN8oVuT48Hpn0r1ZaY0mWhkid8cLIuWVvXxOKG1G/wBhxGMxFiCVYYf+2KoTtFIqESQRSMDkNjBFDwsx8YXNAflNKtlHEyvM294hgkpgewPFU3cttKwVLchj/nXH5Gpaa/aT2pSc4LDBEgzg+ftWP1CZWvneDaoB47vOPpVJWzaMp7FXYj6XUY7VztKbj0VPmP1NXdnXk1LXEMqnbGpYDOR5Vk0Yh92SD1zW2/Z8xuNRmO0cIBx41pSsJENYWE6jplsoVT6eVOoU2jFBWCHYMjFMUGBTYmfcV5ZcjGTXqvh54xmqMglYjAI2qBjwr3wK+cMWHhXkttJLH5fboaHqF3PZJxwKRdodQ+GtjiXYenHH502nlVELM+1cVzntNqBubgoJNsRyFYYOR55qEwlEU3esXEjuvevKfDc2KUwqb/WLaBmfdK6q5BBIHj4+VXPbqRnvJCF6A8AD14ph2Z7mOe+NyCAWTDIo3EYxgMeVHJ6deKOtcnEGxsDIni9ubfTNcmAVZII8xYL/AIQByQcHpz+dYXULt769muZPxSNn2HgPtTLV5DbwCDayvJyc5/Dng/X9KTUZZsYMXxXPISVdZ3L2lzHPHyUYHBJww8QceB6VTUoZc6bo80Wp2UrbmS2lhCtuIUEqw25OOCpLDI8DSDULeDTJZoDIySxMRvDcFfAjHUUr7Ma0NMuGguixsLn5ZgBkp4bl9R+Y+ldAvtHsLmxtL6C53qU/iIG2lxnIIOOh6fXNaGHvJruIVvZff+M5xILeTn4huPw5Jz+de7G0nuJUjhga7djtRVUlvYCmvabS7W1vSkasxaMSNsJkVAecBsc8Y8Ku7NRJafGs0nw7CEGKdn292wIY4bzxx7sKzBDy4maS448xF+o6Re2USmSORWYkmKRfmBxnofSlUsJCpcZDo55x0H9q0Wu62bm6gFrM9xBaxLEHlQLI4J5zj7Um75ll4AG5eSo4cA85H9aWw4k4jFPJRnuU3QePu3RvlXgHxX0oAnJyaO1Fo2dYoCQo42kdPT2/vVml6ety7CZLjBiZkMUe/wCYdM+nnRoIDmBR4bChOfAjrW7/AGaHbeXIfgkKT61iO8kgYofkGemOlaf9ndx3etMpYfxFOB1JqDuQ9TtlqSUGOBRYKgdfzoCzl3RqR81Fd7sG5kwB4g5qQZcSB1wM14dzt+TB60F+8oW7zCshA43r1HnVLXMcqLJC28YypyQCPEehFCTDCw1HbvSM5GABXm7nWC3aVzhEG7mq4cAsVHReKC1qTNi/gioSCenpVdCX5mR1e/1HU7oB1b4RWI2Jxj1z40oumtosiWQxh/mMZJ3N9KoudcnJ7zu544O82m5f5gAD4qOo9aCvLvuHX4O4+JmkPOJAyHPkOSPaqAJhkgQ+O1S6jZ4pLaIBgFEqt3khOeQAPSvmq6lb6fbxRWzSL3K5n3dZCfDHgc449ATRem3lxp1tOLq2he8gidR8g2xBuSSceHmelYPV9Tl1CQhnzGHZ+OAzHq2PsPYVsAFYB8zGSbCQeoJc3D3U7SyHLN+Q8B9BVVSpSYySpUqVJJK0fZTtIulk2t6He0ZtyMp+aF/Bh5jzH1HrnKlWrFTkSmUMMGdVu9RjvWDajbw3CTx7YpVbaGx0LAA5OOnmPak+rM//AAmLKS3htoYZTiVYj/EAwcA+ByQT6Y8BWS0zVGtGWOdO/tc/NET056r5VrrfWgtnLb2UvxFnIQRvX50I6ZB8RyPUVq5K4mcKyGYYqyDkMoPOD0IpmkTSWvfCNmCAlmHQ/XzFP9USG8S2Sym7yCMOTbtGFeHJ5GPH3HHXpR2iwRx6REr3BhhiJYxFt0c7NxggcZKnB5yOOKzikscR5uCDM5+O9uJwqK0kj8AKMlvYCtN2S1SDTbe+t2vJLG7uCsYlC5XZ/Op8VPQg+lJQ8ukX5kjEXfxllZMMdmQQevv1HNU3d29/dyXMu1XkbOFzgDwAzk4GBUHw35kPz1NB20+Dm1XdaTR3MndorTqD/HIH4uc8+H0qnsnHLFrUP8Fiz/gVSOT59Ofaq9DtGvpY7dNzyMwCxsuN/OSc+GOvpWnSSPszKktpOtzqWWWRjkxxoQOV6Z/X26jtmzC0q4nQEjkD2r2s6SxqxSds4Yjk9MefH60e2GXDS8ddvnSnSpC0EbswYuu4k8E+p8qagpjIxnzqOcyIMQK9Eyr3wlUlfmcbf5T/AC15TIlL7V2ygFWQ/K/+tGPFHnIA/vQqptRAiHAz8o8SKTjcbnULMqogUn8fU+g8KSaxci8gBE8RtX3KyE7S7A4C5PgeftXq70qa9uVN7P3cYOREPxOMc4Hh5ZPFUanNdNFLYQo9wzsTIVj3HGeFzjwHj409EGMtEO5zhZndUtreOIytfWyMw+VRyUHjgdPyoKzitbDdNpyd08cZZry4AXaMZO0YxnyJ86I1TS9N0e1+M1GVIgp+WCHDSSn/AA56e+M4rD61r9zqx7sgQ2wYssKnP3Pif94o8KniDlm8y7W9dNzF8DZs4swdzM3DSt6+gPQfU+iSpUpZJJyYYAHUlSpUqpclSpUqSSVKlSpJJXuCaS3kDxOUYeIrxUqSTT6brmmXKiPV7d4pg2Uu7fqPdfetENYNlA0IWO/tHYHvLcgNkHIZT44JJwfyrm1e4ZpYG3RSMh9DTVt+4o1/U2/caZJIs6vDeBB8vfja581bOQcZ6k+1fdM0LvrB4QsRjkuBJvg2NLGBjIX5ufIZOBz1rO2HaS6tWJeOOYNw2VALf79qdDtHotxGFkt7m3kH86txn6dftWkPW3czlLF6mh06P9yJexwaZKGkTbHJcIGdmPRCQcY4zgdcVnfgp9QeVRIZJVG9pHbaWJ6jB6nPh6Hyo627RpDC0MWoQ3ELkFknYjP3AIPqCKvXXbYkywOkMhAGyG4GGHPJJyc8nnPNC1aNjBhLY65yJpOxtwz6XF3qlGB24bOSRWillkVInC53SbCoxwCeCeeOAaw1jd20J7u2uLC3jAOJLq9UFTjghRnofTmmM3aHs7DZhJtbDzEIZO5JaPcM8gbTnOeeOaD2lHZhm1m6EfatqY02DvIkM0rnbFGATub3HSvnLW8QmvljuVGHMcZQnP4snqCfSsbdftD0iGR2giurmZBsSUIsalfT/CPZRWa1Pt9qN2jx2kcdkrdXQlpP/wCj0+gzQgog13IQ7nfU6beaxFpEDOoBAAXvXHcxj03N8zH2rD61+0NyrRWR74/4tuyEey9W92+xrAyyyTOXlkaR2OSzHJP1rzQGz6jBX9y++vbnULgz3kzzSnjcx6DyHkPQVRUqUuMkqVKlSSSpUqVJJ//Z"><link rel="icon" type="image/png" sizes="192x192" href="data:image/png;base64,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">
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
  width:100%;height:auto;
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

const char MANIFEST_JSON[] PROGMEM = R"manifest({"name":"PooKooli Fountain","short_name":"PooKooli","start_url":"/","display":"standalone","background_color":"#07090f","theme_color":"#07090f","icons":[{"src":"data:image/png;base64,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","sizes":"192x192","type":"image/png"},{"src":"data:image/png;base64,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","sizes":"180x180","type":"image/png"}]})manifest";
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
