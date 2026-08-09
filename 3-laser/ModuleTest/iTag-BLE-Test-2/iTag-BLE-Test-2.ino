/*
 ── iTag BLE Test Sketch ─────────────────────────────────────
 Standalone — no pump, no WiFi, no OLED. Just BLE, so we can
 confirm the iTag connects and figure out what a button press
 actually looks like on YOUR unit before touching the fountain
 code again.

 WHAT IT DOES:
  1. Scans for 5 seconds and prints EVERY BLE device it sees
     (name + address + RSSI). This tells us your iTag's real
     advertised name — it may not be exactly "iTag".
  2. If it sees a device matching BLE_TAG_NAME, it tries to
     connect, lists ALL services and characteristics the tag
     actually exposes (so we can see if FFE0/FFE1 is even
     right for your unit), subscribes to notify on any
     notifiable characteristic it finds, and prints raw bytes
     whenever one fires — press the button and watch here.
  3. If it disconnects, it goes back to scanning automatically.

 HOW TO USE:
  1. Flash this to your ESP32, open Serial Monitor at 115200.
  2. Watch the first scan — find your iTag's name in the list
     and set BLE_TAG_NAME to match EXACTLY (case-sensitive).
     Re-flash if you had to change it.
  3. Once connected, press the iTag button repeatedly and watch
     for "[BLE] NOTIFY from <uuid>: <bytes>" lines. Note WHICH
     characteristic UUID fires and what byte(s) it sends — send
     that back to me and I'll wire the real fountain code to
     match your exact unit instead of guessing FFE0/FFE1.

 LIBRARY: NimBLE-Arduino (by h2zero) — same one as before.
 BOARD:   ESP32 Dev Module — Partition Scheme doesn't matter much
          here since this sketch is tiny on its own.
 ─────────────────────────────────────────────────────────────
*/

#include <NimBLEDevice.h>

#define BLE_TAG_NAME  "iTAG"   // confirmed from your scan: advertises as "iTAG"

NimBLEScan* scan = nullptr;
NimBLEClient* client = nullptr;
const NimBLEAdvertisedDevice* targetDevice = nullptr;
bool connected = false;
bool scanning = false;
unsigned long lastAttempt = 0;

void notifyCallback(NimBLERemoteCharacteristic* chr, uint8_t* data, size_t len, bool isNotify) {
  Serial.printf("[BLE] NOTIFY from %s:", chr->getUUID().toString().c_str());
  for (size_t i = 0; i < len; i++) Serial.printf(" %02X", data[i]);
  Serial.println();
}

class ClientCB : public NimBLEClientCallbacks {
  void onDisconnect(NimBLEClient* c, int reason) {
    Serial.printf("[BLE] Disconnected (reason %d)\n", reason);
    connected = false;
  }
};

class ScanCB : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice* dev) override {
    Serial.printf("[Scan] seen: name=\"%s\"  addr=%s  rssi=%d\n",
      dev->haveName() ? dev->getName().c_str() : "(no name)",
      dev->getAddress().toString().c_str(),
      dev->getRSSI());
    if (dev->haveName() && dev->getName() == BLE_TAG_NAME) {
      Serial.println("[Scan] *** MATCH — stopping scan, will connect ***");
      NimBLEDevice::getScan()->stop();
      targetDevice = dev;
    }
  }
};

void startScan() {
  if (scanning || connected) return;
  scanning = true;
  Serial.println("[BLE] Scanning (5s)...");
  scan->setScanCallbacks(new ScanCB(), false);
  scan->setActiveScan(true);
  scan->start(5000, false);
  scanning = false;
}

void tryConnect() {
  if (!targetDevice) return;
  if (!client) {
    client = NimBLEDevice::createClient();
    client->setClientCallbacks(new ClientCB(), false);
  }
  Serial.println("[BLE] Connecting...");
  if (!client->connect(targetDevice)) {
    Serial.println("[BLE] Connect FAILED — will retry");
    return;
  }
  Serial.println("[BLE] Connected. Discovering services...");

  auto services = client->getServices(true);
  bool subscribedAny = false;
  for (auto svc : services) {
    Serial.printf("  Service: %s\n", svc->getUUID().toString().c_str());
    auto chars = svc->getCharacteristics(true);
    for (auto chr : chars) {
      Serial.printf("    Char: %s  (notify=%d write=%d read=%d)\n",
        chr->getUUID().toString().c_str(),
        chr->canNotify(), chr->canWrite(), chr->canRead());
      if (chr->canNotify()) {
        chr->subscribe(true, notifyCallback);
        subscribedAny = true;
        Serial.println("      -> subscribed");
      }
    }
  }
  if (!subscribedAny) Serial.println("[BLE] WARNING: no notifiable characteristic found on this tag.");
  connected = true;
  Serial.println("[BLE] Ready — press the button now.");
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== iTag BLE Test ===");
  NimBLEDevice::init("");
  NimBLEDevice::setPower(3);
  scan = NimBLEDevice::getScan();
  startScan();
}

void loop() {
  if (!connected) {
    unsigned long now = millis();
    if (now - lastAttempt >= 4000) {
      lastAttempt = now;
      if (targetDevice) tryConnect();
      else              startScan();
    }
  }
  delay(50);
}
