/**
 * Phase 1 — Single VL53L5CX Sensor Verification
 * ===============================================
 * Target: Raspberry Pi Pico (Arduino-Pico core)
 * Library: stm32duino/VL53L5CX
 *
 * Wiring:
 *   Pico 3V3  (Pin 36) → VIN  [Sensor A]
 *   Pico GND  (Pin 38) → GND  [Sensor A]
 *   Pico GP4  (Pin  6) → SDA  [Sensor A]
 *   Pico GP5  (Pin  7) → SCL  [Sensor A]
 *   Pico GP2  (Pin  4) → LPn  [Sensor A]
 *
 * Expected output: continuous distance readings from center zone in mm.
 * Move your hand toward/away from the sensor to verify tracking.
 *
 * Install library via Arduino IDE Library Manager:
 *   Search: "VL53L5CX" → install "VL53L5CX by STMicroelectronics"
 */

#include <Wire.h>
#include <vl53l5cx_api.h>

#define LPN_A_PIN   2    // GP2 → LPn Sensor A
#define SDA_PIN     4    // GP4
#define SCL_PIN     5    // GP5
#define CENTER_ZONE 5    // Upper-center zone in 4x4 grid
#define VALID_STATUS 5   // ST ULD driver: 5 = valid target

VL53L5CX_Configuration sensor;
VL53L5CX_ResultsData    results;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("Phase 1 — Single Sensor Verification");
  Serial.println("--------------------------------------");

  // Enable Sensor A via LPn
  pinMode(LPN_A_PIN, OUTPUT);
  digitalWrite(LPN_A_PIN, LOW);
  delay(10);
  digitalWrite(LPN_A_PIN, HIGH);
  delay(100);  // Required boot delay

  Wire.setSDA(SDA_PIN);
  Wire.setSCL(SCL_PIN);
  Wire.begin();
  Wire.setClock(1000000);  // 1 MHz for fast firmware upload

  sensor.platform.address = VL53L5CX_DEFAULT_I2C_ADDRESS;  // 0x29

  Serial.println("Initializing sensor (uploading ~86KB firmware, ~2s)...");
  if (vl53l5cx_init(&sensor) != VL53L5CX_STATUS_OK) {
    Serial.println("ERROR: Sensor A not found. Check wiring and power.");
    while (true) delay(1000);
  }

  // 4x4 = 16 zones, up to 60Hz
  vl53l5cx_set_resolution(&sensor, VL53L5CX_RESOLUTION_4X4);
  vl53l5cx_start_ranging(&sensor);

  Serial.println("Sensor A ready. Readings:");
  Serial.println("  Zone layout: 4x4 grid. Index [5] = upper-center zone.");
  Serial.println("------------------------------------------");
}

void loop() {
  uint8_t ready = 0;
  vl53l5cx_check_data_ready(&sensor, &ready);

  if (ready) {
    vl53l5cx_get_ranging_data(&sensor, &results);

    int16_t  dist_mm = results.distance_mm[CENTER_ZONE];
    uint8_t  status  = results.target_status[CENTER_ZONE];
    bool     valid   = (status == VALID_STATUS);

    Serial.print("  Distance: ");
    Serial.print(dist_mm);
    Serial.print(" mm  [");
    Serial.print(valid ? "OK" : "invalid");
    Serial.println("]");
  }

  delay(50);
}
