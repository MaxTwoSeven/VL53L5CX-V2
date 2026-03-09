/**
 * Phase 2 — Dual VL53L5CX Sensor Verification
 * =============================================
 * Target: Raspberry Pi Pico (Arduino-Pico core)
 * Library: stm32duino/VL53L5CX
 *
 * Wiring (adds Sensor B to Phase 1 setup):
 *   Pico 3V3  (Pin 36) → VIN  [Sensor A]  [Sensor B]
 *   Pico GND  (Pin 38) → GND  [Sensor A]  [Sensor B]
 *   Pico GP4  (Pin  6) → SDA  [Sensor A]  [Sensor B]  (shared bus)
 *   Pico GP5  (Pin  7) → SCL  [Sensor A]  [Sensor B]  (shared bus)
 *   Pico GP2  (Pin  4) → LPn  [Sensor A]
 *   Pico GP3  (Pin  5) → LPn  [Sensor B]
 *
 * Pull-up note: if both breakout boards have onboard pull-up resistors,
 * cut the jumper on one board to keep combined pull-up ~4.7kΩ on SDA/SCL.
 *
 * Address init result:
 *   Sensor A → 0x2A  (chest, reassigned at boot)
 *   Sensor B → 0x29  (lean, default)
 *
 * Expected output: interleaved distance readings from both sensors.
 */

#include <Wire.h>
#include <vl53l5cx_api.h>

#define LPN_A_PIN    2
#define LPN_B_PIN    3
#define SDA_PIN      4
#define SCL_PIN      5
#define CENTER_ZONE  5
#define VALID_STATUS 5

#define ADDR_DEFAULT  0x29
#define ADDR_SENSOR_A 0x2A
#define ADDR_SENSOR_B 0x29

VL53L5CX_Configuration sensor_a;
VL53L5CX_Configuration sensor_b;
VL53L5CX_ResultsData   results_a;
VL53L5CX_ResultsData   results_b;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("Phase 2 — Dual Sensor Verification");
  Serial.println("------------------------------------");

  pinMode(LPN_A_PIN, OUTPUT);
  pinMode(LPN_B_PIN, OUTPUT);

  Wire.setSDA(SDA_PIN);
  Wire.setSCL(SCL_PIN);
  Wire.begin();
  Wire.setClock(1000000);

  // ── Address initialization sequence ────────────────────────────────────────

  // Step 1: Disable both sensors
  Serial.println("Disabling both sensors...");
  digitalWrite(LPN_A_PIN, LOW);
  digitalWrite(LPN_B_PIN, LOW);
  delay(10);

  // Step 2: Boot Sensor A at default 0x29, reassign to 0x2A
  Serial.println("Booting Sensor A...");
  digitalWrite(LPN_A_PIN, HIGH);
  delay(100);

  sensor_a.platform.address = ADDR_DEFAULT;
  if (vl53l5cx_init(&sensor_a) != VL53L5CX_STATUS_OK) {
    Serial.println("ERROR: Sensor A not found.");
    while (true) delay(1000);
  }
  vl53l5cx_set_i2c_address(&sensor_a, ADDR_SENSOR_A << 1);
  Serial.print("  Sensor A reassigned to 0x");
  Serial.println(ADDR_SENSOR_A, HEX);
  delay(100);

  // Step 3: Boot Sensor B at default 0x29
  Serial.println("Booting Sensor B...");
  digitalWrite(LPN_B_PIN, HIGH);
  delay(100);

  sensor_b.platform.address = ADDR_DEFAULT;
  if (vl53l5cx_init(&sensor_b) != VL53L5CX_STATUS_OK) {
    Serial.println("ERROR: Sensor B not found.");
    while (true) delay(1000);
  }
  Serial.print("  Sensor B at 0x");
  Serial.println(ADDR_SENSOR_B, HEX);
  delay(100);

  Serial.println("Both sensors confirmed on bus.");

  // ── Configure and start ranging ─────────────────────────────────────────────
  vl53l5cx_set_resolution(&sensor_a, VL53L5CX_RESOLUTION_4X4);
  vl53l5cx_set_resolution(&sensor_b, VL53L5CX_RESOLUTION_4X4);
  vl53l5cx_start_ranging(&sensor_a);
  vl53l5cx_start_ranging(&sensor_b);

  Serial.println("Reading from both sensors:");
  Serial.println("--------------------------------------------------");
}

void loop() {
  uint8_t ready_a = 0, ready_b = 0;
  vl53l5cx_check_data_ready(&sensor_a, &ready_a);
  vl53l5cx_check_data_ready(&sensor_b, &ready_b);

  if (ready_a) {
    vl53l5cx_get_ranging_data(&sensor_a, &results_a);
    int16_t mm  = results_a.distance_mm[CENTER_ZONE];
    uint8_t st  = results_a.target_status[CENTER_ZONE];
    Serial.print("  A (chest, 0x2A): ");
    Serial.print(mm);
    Serial.print(" mm  [");
    Serial.print(st == VALID_STATUS ? "OK" : "invalid");
    Serial.println("]");
  }

  if (ready_b) {
    vl53l5cx_get_ranging_data(&sensor_b, &results_b);
    int16_t mm  = results_b.distance_mm[CENTER_ZONE];
    uint8_t st  = results_b.target_status[CENTER_ZONE];
    Serial.print("  B (lean,  0x29): ");
    Serial.print(mm);
    Serial.print(" mm  [");
    Serial.print(st == VALID_STATUS ? "OK" : "invalid");
    Serial.println("]");
  }

  delay(50);
}
