/*
 ╔══════════════════════════════════════════════════════════╗
 ║  OLED TEST v2 — with I2C bus recovery                     ║
 ║                                                          ║
 ║  Same as the first OLED test, but BEFORE touching Wire.h, ║
 ║  it manually bit-bangs SCL to force-release a stuck SDA   ║
 ║  line — the classic I2C "bus jammed by a bad device"      ║
 ║  fix. Then scans + tries the OLED on GPIO21/22.           ║
 ║                                                          ║
 ║  If that still fails, it ALSO tries GPIO18/19 (your       ║
 ║  second I2C bus) — this tells us whether the problem is   ║
 ║  specific to GPIO21/22 or follows the OLED wherever it    ║
 ║  goes.                                                    ║
 ╚══════════════════════════════════════════════════════════╝

 ── WIRING (try one at a time as prompted by Serial) ─────────
  Bus A: OLED  VCC→3.3V  GND→GND  SDA→GPIO21  SCL→GPIO22
  Bus B: OLED  VCC→3.3V  GND→GND  SDA→GPIO18  SCL→GPIO19
 ── LIBRARIES ────────────────────────────────────────────────
  Adafruit SSD1306, Adafruit GFX
 ─────────────────────────────────────────────────────────────
*/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define OLED_WIDTH  128
#define OLED_HEIGHT  32

// ── Manual I2C bus recovery ────────────────────────────────
// Toggles SCL up to 9 times (as plain GPIO, before Wire.begin())
// to force a stuck device to release SDA, then sends a manual
// STOP condition. This is the standard fix for a jammed I2C bus.
void recoverI2CBus(int sdaPin, int sclPin) {
  Serial.printf("[Recovery] Attempting bus recovery on SDA=%d SCL=%d...\n", sdaPin, sclPin);
  pinMode(sdaPin, INPUT_PULLUP);
  pinMode(sclPin, OUTPUT);

  bool sdaStuckLow = (digitalRead(sdaPin) == LOW);
  Serial.printf("[Recovery] SDA state before recovery: %s\n", sdaStuckLow ? "LOW (stuck!)" : "HIGH (idle, ok)");

  for (int i = 0; i < 9; i++) {
    digitalWrite(sclPin, LOW);  delayMicroseconds(5);
    digitalWrite(sclPin, HIGH); delayMicroseconds(5);
    if (digitalRead(sdaPin) == HIGH) {
      Serial.printf("[Recovery] SDA released after %d clock pulses\n", i+1);
      break;
    }
  }

  // Manual STOP condition: SDA goes LOW->HIGH while SCL is HIGH
  pinMode(sdaPin, OUTPUT);
  digitalWrite(sdaPin, LOW);  delayMicroseconds(5);
  digitalWrite(sclPin, HIGH); delayMicroseconds(5);
  digitalWrite(sdaPin, HIGH); delayMicroseconds(5);

  bool sdaNowHigh = (digitalRead(sdaPin) == HIGH);
  Serial.printf("[Recovery] SDA state after recovery: %s\n", sdaNowHigh ? "HIGH (ok)" : "STILL LOW (bus may be truly damaged)");
}

// ── Scan + OLED attempt on a given bus ─────────────────────
bool tryOledOnBus(TwoWire &wire, int sdaPin, int sclPin, const char* label) {
  Serial.printf("\n=== Testing %s (SDA=%d SCL=%d) ===\n", label, sdaPin, sclPin);
  recoverI2CBus(sdaPin, sclPin);

  wire.begin(sdaPin, sclPin);
  wire.setClock(100000);
  delay(50);

  Serial.println("[I2C] Scanning...");
  int devices = 0;
  uint8_t foundAddr = 0;
  for (uint8_t a = 8; a < 120; a++) {
    wire.beginTransmission(a);
    if (wire.endTransmission() == 0) {
      Serial.printf("[I2C]  FOUND device at 0x%02X\n", a);
      if (a==0x3C || a==0x3D) foundAddr = a;
      devices++;
    }
  }
  if (devices == 0) {
    Serial.println("[I2C]  NO devices found — even after bus recovery.");
    Serial.println("       -> Points to wiring/power, not a stuck bus.");
    return false;
  }

  Adafruit_SSD1306 oled(OLED_WIDTH, OLED_HEIGHT, &wire, -1);
  uint8_t tryAddr = foundAddr ? foundAddr : 0x3C;
  if (oled.begin(SSD1306_SWITCHCAPVCC, tryAddr)) {
    Serial.printf("[OLED] SUCCESS at 0x%02X on %s!\n", tryAddr, label);
    oled.clearDisplay();
    oled.drawRect(0, 0, OLED_WIDTH, OLED_HEIGHT, SSD1306_WHITE);
    oled.setTextSize(1);
    oled.setTextColor(SSD1306_WHITE);
    oled.setCursor(6, 4);  oled.print("OLED WORKS!");
    oled.setCursor(6, 16); oled.print(label);
    oled.display();
    return true;
  } else {
    Serial.printf("[OLED] begin() FAILED on %s despite device at bus scan.\n", label);
    return false;
  }
}

TwoWire WireB = TwoWire(1);

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== OLED Test v2 — bus recovery + dual-bus check ===");

  bool okA = tryOledOnBus(Wire, 21, 22, "Bus A (GPIO21/22)");
  if (okA) {
    Serial.println("\n[RESULT] OLED works on GPIO21/22 after bus recovery!");
    Serial.println("         The bus was jammed — recovery fixed it.");
    while (1) delay(1000);
  }

  Serial.println("\n[Next] Trying second bus GPIO18/19 instead...");
  bool okB = tryOledOnBus(WireB, 18, 19, "Bus B (GPIO18/19)");
  if (okB) {
    Serial.println("\n[RESULT] OLED works on GPIO18/19 but NOT on GPIO21/22!");
    Serial.println("         -> GPIO21/22 pins or their wiring are the problem,");
    Serial.println("            not the OLED module itself.");
  } else {
    Serial.println("\n[RESULT] OLED failed on BOTH buses.");
    Serial.println("         -> Likely a power problem (check VCC is a clean");
    Serial.println("            3.3V under load) or the OLED module itself,");
    Serial.println("            even though you believe one is known-good.");
  }
  while (1) delay(1000);
}

void loop() {}
