/*
 * VL53L0X (CJVL53L0XV2 / GY-VL53L0X) Test Sketch for ESP32 DevKit V4
 * -------------------------------------------------------------
 * Wiring:
 *   VIN   -> 3V3 (sensor board is 3.3V logic; most breakouts have a regulator for 5V too, but 3V3 is safest)
 *   GND   -> GND
 *   SDA   -> GPIO 21
 *   SCL   -> GPIO 22
 *   XSHUT -> GPIO 32
 *   GPIO1 (interrupt) -> not used here
 *
 * Library required (install via Arduino Library Manager):
 *   "Adafruit VL53L0X" by Adafruit
 *   (it depends on "Adafruit BusIO")
 *
 * If you have multiple VL53L0X sensors, XSHUT lets you power them up
 * one at a time to assign each a different I2C address (example included below, commented out).
 */

#include <Wire.h>
#include "Adafruit_VL53L0X.h"

#define SDA_PIN     21
#define SCL_PIN     22
#define XSHUT_PIN   32

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  // Configure XSHUT pin and power the sensor on
  pinMode(XSHUT_PIN, OUTPUT);
  digitalWrite(XSHUT_PIN, LOW);   // hold sensor in reset
  delay(10);
  digitalWrite(XSHUT_PIN, HIGH);  // release reset / power up
  delay(10);

  // Start I2C on the specified pins
  Wire.begin(SDA_PIN, SCL_PIN);

  Serial.println("Initializing VL53L0X...");

  if (!lox.begin()) {
    Serial.println("Failed to detect/initialize VL53L0X sensor. Check wiring!");
    while (1) { delay(1000); }
  }

  Serial.println("VL53L0X sensor initialized successfully.");
}

void loop() {
  VL53L0X_RangingMeasurementData_t measure;

  lox.rangingTest(&measure, false); // pass 'true' for debug data on Serial

  if (measure.RangeStatus != 4) {  // 4 = out of range / invalid reading
    Serial.print("Distance (mm): ");
    Serial.println(measure.RangeMilliMeter);
  } else {
    Serial.println("Out of range");
  }

  delay(200);
}

/*
 * ---- Multiple sensors example (uncomment / adapt if needed) ----
 *
 * #define XSHUT_1  32
 * #define XSHUT_2  33
 *
 * Adafruit_VL53L0X sensor1 = Adafruit_VL53L0X();
 * Adafruit_VL53L0X sensor2 = Adafruit_VL53L0X();
 *
 * void setup() {
 *   Serial.begin(115200);
 *   Wire.begin(SDA_PIN, SCL_PIN);
 *
 *   pinMode(XSHUT_1, OUTPUT);
 *   pinMode(XSHUT_2, OUTPUT);
 *   digitalWrite(XSHUT_1, LOW);
 *   digitalWrite(XSHUT_2, LOW);
 *   delay(10);
 *
 *   // Bring up sensor 1, assign new address
 *   digitalWrite(XSHUT_1, HIGH);
 *   delay(10);
 *   sensor1.begin(0x30);
 *
 *   // Bring up sensor 2, assign new address
 *   digitalWrite(XSHUT_2, HIGH);
 *   delay(10);
 *   sensor2.begin(0x31);
 * }
 */
