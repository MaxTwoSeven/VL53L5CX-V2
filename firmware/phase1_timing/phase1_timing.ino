/**
 * Phase 1 — Real-Time Throughput Monitor
 * ========================================
 * Target: Raspberry Pi Pico (Arduino-Pico core — earlephilhower)
 * Library: SparkFun VL53L5CX Library
 *
 * Shows live distance readings alongside timing stats:
 *   - Instantaneous interval between readings (ms)
 *   - Rolling average Hz over the last 16 readings
 *   - Min / max interval observed since boot
 *   - A live ASCII bar scaled to 700mm range
 *
 * Wiring (same as Phase 1):
 *   Pico 3V3  (Pin 36) → VIN
 *   Pico GND  (Pin 38) → GND
 *   Pico GP4  (Pin  6) → SDA
 *   Pico GP5  (Pin  7) → SCL
 *   Pico GP2  (Pin  4) → LPn
 */

#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h>

#define LPN_A_PIN    2
#define SDA_PIN      4
#define SCL_PIN      5
// #define CENTER_ZONE  5   // center zone index for 4x4 mode
#define CENTER_ZONE  27      // center zone index for 8x8 mode (row 3, col 3)
#define VALID_STATUS 5

#define ROLLING_N    16     // window size for rolling average Hz
#define BAR_WIDTH    40     // characters wide for ASCII bar
#define MAX_MM       700    // bar scale ceiling

SparkFun_VL53L5CX sensor;
VL53L5CX_ResultsData results;

// ── Timing state ─────────────────────────────────────────────────────────────
uint32_t intervals[ROLLING_N] = {0};
uint8_t  intervalIdx = 0;
uint32_t lastReadingUs = 0;
uint32_t minIntervalMs = UINT32_MAX;
uint32_t maxIntervalMs = 0;
uint32_t totalReadings = 0;

// ── ASCII bar ─────────────────────────────────────────────────────────────────
void printBar(int16_t mm) {
  int filled = constrain((int)map(mm, 0, MAX_MM, 0, BAR_WIDTH), 0, BAR_WIDTH);
  Serial.print("  [");
  for (int i = 0; i < BAR_WIDTH; i++) Serial.print(i < filled ? '#' : ' ');
  Serial.print("] ");
  Serial.print(mm);
  Serial.println(" mm");
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_BUILTIN, HIGH); delay(100);
    digitalWrite(LED_BUILTIN, LOW);  delay(100);
  }

  Serial.begin(115200);
  delay(3000);

  Serial.println("Phase 1 — Throughput Monitor");
  Serial.println("------------------------------");
  Serial.flush();

  pinMode(LPN_A_PIN, OUTPUT);
  digitalWrite(LPN_A_PIN, LOW);  delay(10);
  digitalWrite(LPN_A_PIN, HIGH); delay(100);

  Wire.setSDA(SDA_PIN);
  Wire.setSCL(SCL_PIN);
  Wire.begin();
  Wire.setClock(400000);

  Serial.println("Initializing sensor (~2s)...");
  Serial.flush();
  if (!sensor.begin(0x29, Wire)) {
    Serial.println("ERROR: sensor not found. Check wiring.");
    Serial.flush();
    while (true) { digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); delay(200); }
  }

  // Try the fastest resolution and ranging frequency
  // sensor.setResolution(4 * 4);     // 4x4 = 16 zones, up to 60Hz
  // sensor.setRangingFrequency(60);

  sensor.setResolution(8 * 8);        // 8x8 = 64 zones, up to 15Hz
  sensor.setRangingFrequency(15);     // request 15Hz (sensor max for 8x8)
  sensor.startRanging();

  digitalWrite(LED_BUILTIN, HIGH);

  Serial.println("Running. Columns: reading# | interval ms | rolling Hz | dist mm | bar");
  Serial.println("------------------------------------------------------------------------");
  Serial.flush();

  lastReadingUs = micros();
}

void loop() {
  if (!sensor.isDataReady()) return;

  uint32_t nowUs       = micros();
  uint32_t intervalUs  = nowUs - lastReadingUs;
  uint32_t intervalMs  = intervalUs / 1000;
  lastReadingUs        = nowUs;

  sensor.getRangingData(&results);
  totalReadings++;

  // Update min/max (skip first reading — interval is from boot)
  if (totalReadings > 1) {
    if (intervalMs < minIntervalMs) minIntervalMs = intervalMs;
    if (intervalMs > maxIntervalMs) maxIntervalMs = intervalMs;
  }

  // Rolling average
  intervals[intervalIdx % ROLLING_N] = intervalUs;
  intervalIdx++;
  uint8_t  samples    = min((uint32_t)ROLLING_N, totalReadings - 1);
  float    avgUs      = 0;
  for (uint8_t i = 0; i < samples; i++) avgUs += intervals[i];
  if (samples > 0) avgUs /= samples;
  float    rollingHz  = (avgUs > 0) ? 1000000.0f / avgUs : 0;

  int16_t dist_mm = results.distance_mm[CENTER_ZONE];
  uint8_t status  = results.target_status[CENTER_ZONE];
  bool    valid   = (status == VALID_STATUS);

  // ── Print one line per reading ─────────────────────────────────────────────
  Serial.print("#");
  Serial.print(totalReadings);
  Serial.print("  ");

  Serial.print(intervalMs);
  Serial.print("ms  ");

  Serial.print(rollingHz, 1);
  Serial.print("Hz  ");

  Serial.print(valid ? "  " : "? ");  // flag invalid readings
  printBar(valid ? dist_mm : 0);

  // ── Print summary every 60 readings ──────────────────────────────────────
  if (totalReadings % 60 == 0) {
    Serial.println();
    Serial.print("  >> ");
    Serial.print(totalReadings);
    Serial.print(" readings  |  rolling ");
    Serial.print(rollingHz, 1);
    Serial.print(" Hz  |  interval min/max: ");
    Serial.print(minIntervalMs);
    Serial.print(" / ");
    Serial.print(maxIntervalMs);
    Serial.println(" ms");
    Serial.println();
    Serial.flush();
  }
}
