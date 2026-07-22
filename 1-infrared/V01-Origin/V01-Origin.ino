// ── Cat Water Fountain · ESP32 ──────────────────────
// Touch sensor  → GPIO4  (toggle pump manually)
// PIR sensor    → GPIO5  (auto start when cat near)
// Water sensor  → GPIO34 (LOW = empty, block pump)
// Pump MOSFET   → GPIO16
// Status LED    → GPIO2

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ── Pin definitions ───────────────────────────────
const int PIN_TOUCH  = 4;
const int PIN_PIR    = 5;
const int PIN_WATER  = 34;
const int PIN_PUMP   = 16;
const int PIN_LED    = 2;

// ── BLE UUIDs ─────────────────────────────────────
#define SERVICE_UUID  "12345678-1234-1234-1234-123456789012"
#define CHAR_UUID     "87654321-4321-4321-4321-210987654321"

// ── State ─────────────────────────────────────────
bool pumpRunning   = false;
bool appCommand    = false;
bool lastTouch     = false;
unsigned long pirTimer = 0;
const unsigned long PIR_TIMEOUT = 30000; // 30s after cat leaves

BLECharacteristic* pChar;
BLEServer* pServer;
bool deviceConnected = false;

// ── BLE callbacks ─────────────────────────────────
class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* s)    { deviceConnected = true; }
  void onDisconnect(BLEServer* s) { deviceConnected = false; appCommand = false; }
};

class CharCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) {
    String val = c->getValue().c_str();
    if (val == "ON")  appCommand = true;
    if (val == "OFF") appCommand = false;
  }
};

// ── Pump control ──────────────────────────────────
void setPump(bool on) {
  bool waterOk = digitalRead(PIN_WATER);   // HIGH = water present
  bool run = on && waterOk;                 // block if tank empty
  digitalWrite(PIN_PUMP, run ? HIGH : LOW);
  digitalWrite(PIN_LED,  run ? HIGH : LOW);
  pumpRunning = run;
  // Notify app of status
  if (deviceConnected) {
    String status = run ? "PUMP:ON" : (waterOk ? "PUMP:OFF" : "PUMP:EMPTY");
    pChar->setValue(status.c_str());
    pChar->notify();
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_TOUCH, INPUT);
  pinMode(PIN_PIR,   INPUT);
  pinMode(PIN_WATER, INPUT_PULLUP);   // pull-up; float switch pulls LOW
  pinMode(PIN_PUMP,  OUTPUT);
  pinMode(PIN_LED,   OUTPUT);
  digitalWrite(PIN_PUMP, LOW);

  // ── BLE setup ─────────────────────────────────
  BLEDevice::init("CatFountain");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());
  BLEService* svc = pServer->createService(SERVICE_UUID);
  pChar = svc->createCharacteristic(CHAR_UUID,
    BLECharacteristic::PROPERTY_READ  |
    BLECharacteristic::PROPERTY_WRITE |
    BLECharacteristic::PROPERTY_NOTIFY);
  pChar->addDescriptor(new BLE2902());
  pChar->setCallbacks(new CharCallbacks());
  svc->start();
  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  BLEDevice::startAdvertising();
  Serial.println("BLE ready — device name: CatFountain");
}

void loop() {
  bool touchNow = digitalRead(PIN_TOUCH);
  bool pirNow   = digitalRead(PIN_PIR);

  // Touch: toggle on rising edge only ─────────────
  if (touchNow && !lastTouch) {
    appCommand = false;             // touch overrides app
    pirTimer   = 0;                 // cancel PIR timer
    setPump(!pumpRunning);
    delay(50);                      // debounce
  }
  lastTouch = touchNow;

  // PIR: start pump, reset 30s timer ──────────────
  if (pirNow) {
    pirTimer = millis();
    if (!pumpRunning) setPump(true);
  }
  // PIR timeout: stop pump after 30s of no motion
  if (pirTimer > 0 && (millis() - pirTimer > PIR_TIMEOUT) && !appCommand) {
    pirTimer = 0;
    setPump(false);
  }

  // App command ────────────────────────────────────
  if (appCommand && !pumpRunning) setPump(true);
  if (!appCommand && pumpRunning && pirTimer == 0) setPump(false);

  delay(100);
}