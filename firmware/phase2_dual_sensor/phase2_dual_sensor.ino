/**
 * Phase 2 — Dual VL53L5CX Sensor Verification
 * =============================================
 * Target: Raspberry Pi Pico (Arduino-Pico core — earlephilhower)
 * Library: SparkFun VL53L5CX Library
 *
 * Wiring:
 *   Pico 3V3  (Pin 36) → VIN  [Sensor A]  [Sensor B]
 *   Pico GND  (Pin 38) → GND  [Sensor A]  [Sensor B]
 *   Pico GP4  (Pin  6) → SDA  [Sensor A]  [Sensor B]  (shared bus)
 *   Pico GP5  (Pin  7) → SCL  [Sensor A]  [Sensor B]  (shared bus)
 *   Pico GP2  (Pin  4) → LPn  [Sensor A]
 *   Pico GP3  (Pin  5) → LPn  [Sensor B]
 *
 * Pull-up note: most breakout boards have onboard pull-ups. With two boards
 * on the same bus the combined resistance may drop too low (~2.35kΩ). If
 * readings are erratic, cut the pull-up jumper on one board.
 *
 * Address assignment after boot:
 *   Sensor A → 0x2A  (chest)
 *   Sensor B → 0x29  (lean, default)
 *
 * LED codes:
 *   3 short blinks                = setup() started
 *   1 long blink after each step  = sensor confirmed
 *   fast blink forever            = sensor not found on I2C
 *   solid ON                      = both sensors ranging
 */

#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h>

#define LPN_A_PIN    2
#define LPN_B_PIN    3
#define SDA_PIN      4
#define SCL_PIN      5
#define CENTER_ZONE  5
#define VALID_STATUS 5

SparkFun_VL53L5CX sensor_a;
SparkFun_VL53L5CX sensor_b;
VL53L5CX_ResultsData results_a;
VL53L5CX_ResultsData results_b;

void blink(int n, int onMs, int offMs) {
  for (int i = 0; i < n; i++) {
    digitalWrite(LED_BUILTIN, HIGH); delay(onMs);
    digitalWrite(LED_BUILTIN, LOW);  delay(offMs);
  }
}

void panicFast(const char* msg) {
  Serial.println(msg); Serial.flush();
  while (true) blink(1, 80, 80);
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  blink(3, 100, 100);  // alive

  Serial.begin(115200);
  delay(3000);  // allow Serial Monitor to connect; no while(!Serial) — causes DTR reset loop

  Serial.println("Phase 2 — Dual Sensor Verification");
  Serial.println("------------------------------------");
  Serial.flush();

  pinMode(LPN_A_PIN, OUTPUT);
  pinMode(LPN_B_PIN, OUTPUT);

  Wire.setSDA(SDA_PIN);
  Wire.setSCL(SCL_PIN);
  Wire.begin();
  Wire.setClock(400000);
  Serial.println("  I2C started on GP4/GP5 @ 400kHz");
  Serial.flush();

  // ── Step 1: both sensors off ──────────────────────────────────────────────
  Serial.println("  [1] Disabling both sensors...");
  Serial.flush();
  digitalWrite(LPN_A_PIN, LOW);
  digitalWrite(LPN_B_PIN, LOW);
  delay(10);

  // ── Step 2: boot Sensor A at 0x29, reassign to 0x2A ──────────────────────
  Serial.println("  [2] Booting Sensor A (firmware upload ~2s)...");
  Serial.flush();
  digitalWrite(LPN_A_PIN, HIGH);
  delay(100);

  if (!sensor_a.begin(0x29, Wire)) {
    panicFast("  ERROR: Sensor A not found at 0x29. Check LPn_A wiring (GP2, pin 4).");
  }
  sensor_a.setAddress(0x2A);
  blink(1, 400, 200);  // A confirmed
  Serial.println("  [2] Sensor A → 0x2A  OK");
  Serial.flush();
  delay(50);

  // ── Step 3: boot Sensor B at default 0x29 ────────────────────────────────
  Serial.println("  [3] Booting Sensor B (firmware upload ~2s)...");
  Serial.flush();
  digitalWrite(LPN_B_PIN, HIGH);
  delay(100);

  if (!sensor_b.begin(0x29, Wire)) {
    panicFast("  ERROR: Sensor B not found at 0x29. Check LPn_B wiring (GP3, pin 5).");
  }
  blink(1, 400, 200);  // B confirmed
  Serial.println("  [3] Sensor B → 0x29  OK");
  Serial.flush();
  delay(50);

  // ── Step 4: configure and start ranging ───────────────────────────────────
  sensor_a.setResolution(4 * 4);
  sensor_b.setResolution(4 * 4);
  sensor_a.setRangingFrequency(60);
  sensor_b.setRangingFrequency(60);
  sensor_a.startRanging();
  sensor_b.startRanging();

  digitalWrite(LED_BUILTIN, HIGH);  // solid = both sensors ranging

  Serial.println("  Both sensors ranging.");
  Serial.println();
  Serial.println("  Sensor A (chest 0x2A) | Sensor B (lean 0x29)");
  Serial.println("  -----------------------------------------------");
  Serial.flush();
}

void loop() {
  bool gotA = sensor_a.isDataReady();
  bool gotB = sensor_b.isDataReady();

  if (gotA) {
    sensor_a.getRangingData(&results_a);
    int16_t mm = results_a.distance_mm[CENTER_ZONE];
    uint8_t st = results_a.target_status[CENTER_ZONE];
    Serial.print("  A (chest 0x2A): ");
    Serial.print(mm);
    Serial.print(" mm  [");
    Serial.print(st == VALID_STATUS ? "OK" : "invalid");
    Serial.print("]");
  }

  if (gotB) {
    sensor_b.getRangingData(&results_b);
    int16_t mm = results_b.distance_mm[CENTER_ZONE];
    uint8_t st = results_b.target_status[CENTER_ZONE];
    if (gotA) Serial.print("   ");
    Serial.print("  B (lean  0x29): ");
    Serial.print(mm);
    Serial.print(" mm  [");
    Serial.print(st == VALID_STATUS ? "OK" : "invalid");
    Serial.print("]");
  }

  if (gotA || gotB) {
    Serial.println();
    Serial.flush();
  }

  delay(50);
}
