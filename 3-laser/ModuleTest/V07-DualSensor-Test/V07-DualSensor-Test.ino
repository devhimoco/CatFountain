/*
 * Dual VL53L0X Test Sketch for ESP32 DevKit V4
 * -------------------------------------------------------------
 * Two VL53L0X (CJVL53L0XV2 / GY-VL53L0X) sensors sharing ONE I2C bus.
 * Since both sensors boot up at the same default address (0x29),
 * we use XSHUT to bring them up one at a time and assign each
 * a unique I2C address before use.
 *
 * Wiring:
 *   Both sensors:
 *     VIN   -> 3V3
 *     GND   -> GND
 *     SDA   -> GPIO 21   (shared)
 *     SCL   -> GPIO 22   (shared)
 *
 *   Sensor #1:
 *     XSHUT -> GPIO 32
 *
 *   Sensor #2:
 *     XSHUT -> GPIO 33
 *
 * Library required (install via Arduino Library Manager):
 *   "Adafruit VL53L0X" by Adafruit (pulls in Adafruit BusIO)
 */

#include <Wire.h>
#include "Adafruit_VL53L0X.h"

#define SDA_PIN   21
#define SCL_PIN   22

#define XSHUT_1   32
#define XSHUT_2   33

#define ADDR_1    0x29  // new address for sensor 1
#define ADDR_2    0x31   // new address for sensor 2

Adafruit_VL53L0X sensor1 = Adafruit_VL53L0X();
Adafruit_VL53L0X sensor2 = Adafruit_VL53L0X();

VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  pinMode(XSHUT_1, OUTPUT);
  pinMode(XSHUT_2, OUTPUT);

  // Hold both sensors in reset
  digitalWrite(XSHUT_1, LOW);
  digitalWrite(XSHUT_2, LOW);
  delay(10);

  Wire.begin(SDA_PIN, SCL_PIN);

  // --- Bring up sensor 1 only, assign it a new address ---
  digitalWrite(XSHUT_1, HIGH);
  delay(10);
  Serial.println("Initializing sensor 1...");
  if (!sensor1.begin(ADDR_1)) {
    Serial.println("Failed to initialize sensor 1. Check wiring!");
    while (1) { delay(1000); }
  }
  Serial.println("Sensor 1 OK at address 0x30");

  // --- Bring up sensor 2, assign it a new address ---
  digitalWrite(XSHUT_2, HIGH);
  delay(10);
  Serial.println("Initializing sensor 2...");
  if (!sensor2.begin(ADDR_2)) {
    Serial.println("Failed to initialize sensor 2. Check wiring!");
    while (1) { delay(1000); }
  }
  Serial.println("Sensor 2 OK at address 0x31");

  Serial.println("Both sensors initialized successfully.");
  Serial.println("----------------------------------------");
}

void loop() {
  sensor1.rangingTest(&measure1, false);
  sensor2.rangingTest(&measure2, false);

  Serial.print("Sensor 1: ");
  if (measure1.RangeStatus != 4) {
    Serial.print(measure1.RangeMilliMeter);
    Serial.print(" mm");
  } else {
    Serial.print("out of range");
  }

  Serial.print("   |   Sensor 2: ");
  if (measure2.RangeStatus != 4) {
    Serial.print(measure2.RangeMilliMeter);
    Serial.print(" mm");
  } else {
    Serial.print("out of range");
  }
  Serial.println();

  delay(200);
}
