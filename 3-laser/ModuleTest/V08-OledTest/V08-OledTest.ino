/*
 ╔══════════════════════════════════════════════════════════╗
 ║  OLED TEST — Serial + on-screen diagnostic                ║
 ║                                                          ║
 ║  Tests ONLY the 0.96" SSD1306 OLED on GPIO21/22.          ║
 ║  No sensors, no pump — isolates the display completely.   ║
 ║                                                          ║
 ║  Does an I2C scan FIRST (shows every address that         ║
 ║  responds), then tries both common OLED addresses         ║
 ║  (0x3C and 0x3D) so a wrong-address module still shows.   ║
 ╚══════════════════════════════════════════════════════════╝

 ── WIRING ───────────────────────────────────────────────────
  OLED SSD1306:  VCC→3.3V  GND→GND  SDA→GPIO21  SCL→GPIO22
 ── LIBRARIES ────────────────────────────────────────────────
  Adafruit SSD1306, Adafruit GFX
 ─────────────────────────────────────────────────────────────
*/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define OLED_WIDTH  128
#define OLED_HEIGHT  32

Adafruit_SSD1306 oled(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);
uint8_t foundAddr = 0;
bool oledOk = false;
unsigned long frame = 0;

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== OLED Test (GPIO21/22) ===");

  Wire.begin(21, 22);
  Wire.setClock(100000);
  delay(50);

  // I2C scan — shows every device that answers, whatever address it is
  Serial.println("[I2C] Scanning GPIO21(SDA)/GPIO22(SCL)...");
  int devices = 0;
  for (uint8_t a = 8; a < 120; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.printf("[I2C]  FOUND device at 0x%02X %s\n", a,
        (a==0x3C || a==0x3D) ? "(likely the OLED)" : "(?)");
      if (a==0x3C || a==0x3D) foundAddr = a;
      devices++;
    }
  }
  if (devices == 0) {
    Serial.println("[I2C]  NO devices found at all.");
    Serial.println("       -> Check VCC/GND, and SDA/SCL wiring.");
    Serial.println("       -> Try swapping SDA/SCL if wiring is uncertain.");
  } else if (foundAddr == 0) {
    Serial.println("[I2C]  Device(s) found, but none at 0x3C or 0x3D.");
    Serial.println("       -> Wrong address for an SSD1306 OLED — double");
    Serial.println("          check the module type/datasheet.");
  }

  // Try 0x3C first (most common), then 0x3D as fallback
  Serial.println("[OLED] Trying address 0x3C...");
  if (oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    oledOk = true;
    foundAddr = 0x3C;
    Serial.println("[OLED] SUCCESS at 0x3C");
  } else {
    Serial.println("[OLED] 0x3C failed. Trying 0x3D...");
    if (oled.begin(SSD1306_SWITCHCAPVCC, 0x3D)) {
      oledOk = true;
      foundAddr = 0x3D;
      Serial.println("[OLED] SUCCESS at 0x3D");
    } else {
      Serial.println("[OLED] FAILED at both 0x3C and 0x3D.");
      Serial.println("       -> begin() itself failed even though scan");
      Serial.println("          may have found something — check power");
      Serial.println("          (VCC must be steady 3.3V) and continuity.");
    }
  }

  if (!oledOk) {
    Serial.println("\n[RESULT] OLED did not initialize. No display test possible.");
    return;
  }

  // Static test pattern first — proves the panel itself lights up
  Serial.println("[OLED] Drawing test pattern...");
  oled.clearDisplay();
  oled.drawRect(0, 0, OLED_WIDTH, OLED_HEIGHT, SSD1306_WHITE);
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);
  oled.setCursor(6, 4);
  oled.print("OLED TEST OK");
  char buf[20];
  snprintf(buf, sizeof(buf), "Addr: 0x%02X", foundAddr);
  oled.setCursor(6, 16);
  oled.print(buf);
  oled.display();
  Serial.println("[RESULT] If the screen is dark now, it's a hardware");
  Serial.println("         problem (panel, power, or connector) even");
  Serial.println("         though I2C communication succeeded.");
  delay(3000);
}

void loop() {
  if (!oledOk) { delay(1000); return; }

  // Live animated counter — proves continuous updates work, not just
  // a single static frame
  frame++;
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setCursor(0, 0);
  oled.print("OLED live test");
  oled.setCursor(0, 12);
  oled.setTextSize(2);
  oled.print(frame);
  oled.display();
  Serial.printf("[OLED] frame %lu\n", frame);
  delay(500);
}
