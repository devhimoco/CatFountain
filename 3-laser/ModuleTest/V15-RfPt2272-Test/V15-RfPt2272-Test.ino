/*
 ╔══════════════════════════════════════════════════════════╗
 ║  433MHz REMOTE TEST — YK04 + PT2272-M4 (Serial only)       ║
 ║                                                          ║
 ║  Tests ONLY the RF receiver's 4 button outputs.           ║
 ║  No sensors, no pump, no OLED — isolates the remote        ║
 ║  completely so we can see exactly what the pins are doing.║
 ║                                                          ║
 ║  Prints every state change immediately, PLUS a periodic   ║
 ║  raw-state snapshot every second — so you can tell the    ║
 ║  difference between "nothing is happening" (wiring/power  ║
 ║  problem) and "pins are stuck" (decoder/pairing problem). ║
 ╚══════════════════════════════════════════════════════════╝

 ── WIRING ───────────────────────────────────────────────────
  YK04 module:  VCC→5V   GND→GND
                D0(A)→GPIO16   D1(B)→GPIO17
                D2(C)→GPIO18   D3(D)→GPIO19
  Antenna:      ~17cm straight wire on the ANT pad — range will
                be very poor without it.
 ── NO LIBRARIES NEEDED — just plain digitalRead() ─────────────
 ─────────────────────────────────────────────────────────────
*/

#define PIN_A 16
#define PIN_B 17
#define PIN_C 18
#define PIN_D 19

const int   pins[4]     = { PIN_A, PIN_B, PIN_C, PIN_D };
const char* names[4]    = { "A", "B", "C", "D" };
bool        lastState[4] = { false, false, false, false };
unsigned long lastSnapshot = 0;

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== 433MHz Remote Test (YK04 + PT2272-M4) ===");
  Serial.println("Pins: A=GPIO16  B=GPIO17  C=GPIO18  D=GPIO19");

  for (int i = 0; i < 4; i++) pinMode(pins[i], INPUT);

  Serial.println("\nInitial pin state at boot:");
  for (int i = 0; i < 4; i++) {
    bool s = digitalRead(pins[i]) == HIGH;
    lastState[i] = s;
    Serial.printf("  %s (GPIO%d): %s\n", names[i], pins[i], s ? "HIGH" : "LOW");
  }
  Serial.println("\nIf any pin already reads HIGH with no button pressed,");
  Serial.println("that's a wiring problem (pin floating or shorted) — a");
  Serial.println("PT2272 in momentary mode should idle LOW on all outputs.");
  Serial.println("\nPress remote buttons now...\n");
}

void loop() {
  unsigned long now = millis();

  // Report every state change immediately
  for (int i = 0; i < 4; i++) {
    bool cur = (digitalRead(pins[i]) == HIGH);
    if (cur != lastState[i]) {
      Serial.printf("[%s] GPIO%d -> %s\n", names[i], pins[i], cur ? "PRESSED (HIGH)" : "released (LOW)");
      lastState[i] = cur;
    }
  }

  // Periodic raw snapshot — shows the receiver is alive even if no
  // button is currently pressed, and reveals stuck-pin problems
  if (now - lastSnapshot >= 1000) {
    lastSnapshot = now;
    Serial.printf("[Snapshot] A=%d B=%d C=%d D=%d\n",
      digitalRead(PIN_A), digitalRead(PIN_B), digitalRead(PIN_C), digitalRead(PIN_D));
  }

  delay(10);
}
