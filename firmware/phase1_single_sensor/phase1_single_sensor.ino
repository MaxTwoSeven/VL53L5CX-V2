/**
 * Phase 1 — Single VL53L5CX Sensor Verification
 * ===============================================
 * Target: Raspberry Pi Pico (Arduino-Pico core — earlephilhower)
 * Library: SparkFun VL53L5CX Library
 *
 * Wiring:
 *   Pico 3V3  (Pin 36) → VIN  [Sensor A]
 *   Pico GND  (Pin 38) → GND  [Sensor A]
 *   Pico GP4  (Pin  6) → SDA  [Sensor A]
 *   Pico GP5  (Pin  7) → SCL  [Sensor A]
 *   Pico GP2  (Pin  4) → LPn  [Sensor A]
 *
 * LED:  3 blinks = starting  |  solid = ranging active
 */

#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h>

#define LPN_A_PIN    2    // GP2, physical pin 4
#define SDA_PIN      4    // GP4, physical pin 6
#define SCL_PIN      5    // GP5, physical pin 7
#define CENTER_ZONE  5    // Upper-center zone in 4x4 grid
#define VALID_STATUS 5    // ST ULD: 5 = valid target

SparkFun_VL53L5CX sensor;
VL53L5CX_ResultsData results;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_BUILTIN, HIGH); delay(100);
    digitalWrite(LED_BUILTIN, LOW);  delay(100);
  }

  Serial.begin(115200);
  delay(3000);  // Allow Serial Monitor to connect (no while(!Serial) — causes DTR reset loop)

  Serial.println("Phase 1 — Single Sensor Verification");
  Serial.println("--------------------------------------");
  Serial.flush();

  pinMode(LPN_A_PIN, OUTPUT);
  digitalWrite(LPN_A_PIN, LOW);
  delay(10);
  digitalWrite(LPN_A_PIN, HIGH);
  delay(100);

  Wire.setSDA(SDA_PIN);
  Wire.setSCL(SCL_PIN);
  Wire.begin();
  Wire.setClock(400000);

  Serial.println("Initializing sensor (~2s)...");
  Serial.flush();
  if (!sensor.begin(0x29, Wire)) {
    Serial.println("ERROR: sensor.begin() failed. Check wiring.");
    Serial.flush();
    while (true) {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      delay(200);
    }
  }

  sensor.setResolution(4 * 4);
  sensor.startRanging();

  digitalWrite(LED_BUILTIN, HIGH);  // solid = ready

  Serial.println("Sensor ready. Readings:");
  Serial.println("  Zone layout: 4x4 grid. Center zone index = 5.");
  Serial.println("------------------------------------------");
  Serial.flush();
}

void loop() {
  if (sensor.isDataReady()) {
    sensor.getRangingData(&results);

    int16_t dist_mm = results.distance_mm[CENTER_ZONE];
    uint8_t status  = results.target_status[CENTER_ZONE];

    Serial.print("Distance: ");
    Serial.print(dist_mm);
    Serial.print(" mm  [");
    Serial.print(status == VALID_STATUS ? "OK" : "invalid");
    Serial.println("]");
    Serial.flush();
  }

  delay(50);
}
