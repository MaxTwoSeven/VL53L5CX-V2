/**
 * Phase 3 — Raw Serial Validator
 * ================================
 * Target:  Raspberry Pi Pico (Arduino-Pico / earlephilhower)
 * Library: SparkFun VL53L5CX Library
 *
 * Plain-text output — NO ANSI codes.
 * Open with Arduino IDE Serial Monitor at 115200 baud.
 * No "screen" required.
 *
 * Output format (one line per frame pair):
 *   #1234 | A: 412mm 9z  B: 398mm 7z | depth:291mm lean:+10mm | jX:133 jY:156
 *
 * Wiring (same as all Phase 2/3 sketches):
 *   GP2 → LPn A    GP3 → LPn B
 *   GP4 → SDA      GP5 → SCL
 *   GP6 → INT A    GP7 → INT B
 */

// ── Grid mode ─────────────────────────────────────────────────────────────────
// #define USE_8X8   // uncomment for 8x8 @ 15 Hz

#ifdef USE_8X8
  #define GRID_SIZE   8
  #define NUM_ZONES  64
  #define RANGING_HZ 15
#else
  #define GRID_SIZE   4
  #define NUM_ZONES  16
  #define RANGING_HZ 60
#endif

// ── Position tuning (same as phase3_position) ─────────────────────────────────
#define MIN_VALID_MM    50
#define MAX_VALID_MM   800
#define MIN_DEPTH_MM   200
#define MAX_DEPTH_MM   700
#define MAX_LEAN_MM    200
#define COS45         0.7071f

// ── Hardware ──────────────────────────────────────────────────────────────────
#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h>

// Struct must be declared before any function definitions so the Arduino IDE's
// auto-generated forward declarations can reference the type correctly.
struct SensorReading {
  float dist_mm;
  int   valid_count;
};

#define LPN_A_PIN    2
#define LPN_B_PIN    3
#define SDA_PIN      4
#define SCL_PIN      5
#define INT_A_PIN    6
#define INT_B_PIN    7
#define VALID_STATUS 5

SparkFun_VL53L5CX    sensor_a;
SparkFun_VL53L5CX    sensor_b;
VL53L5CX_ResultsData results_a;
VL53L5CX_ResultsData results_b;

// ── State ─────────────────────────────────────────────────────────────────────
uint32_t frameCount   = 0;
uint32_t lastReadA_ms = 0;
uint32_t lastReadB_ms = 0;

// ── Utilities ─────────────────────────────────────────────────────────────────
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

float mapF(float x, float inA, float inB, float outA, float outB) {
  if (inA == inB) return outA;
  return outA + (x - inA) * (outB - outA) / (inB - inA);
}

int mapClamp(float x, float inA, float inB, int outA, int outB) {
  int v = (int)mapF(x, inA, inB, (float)outA, (float)outB);
  int lo = min(outA, outB), hi = max(outA, outB);
  return v < lo ? lo : (v > hi ? hi : v);
}

// ── Position calculation ───────────────────────────────────────────────────────
SensorReading computeReading(VL53L5CX_ResultsData& data) {
  SensorReading r = {-1.0f, 0};
  float dSum = 0, dW = 0;
  for (int z = 0; z < NUM_ZONES; z++) {
    if (data.target_status[z] != VALID_STATUS) continue;
    int16_t mm = data.distance_mm[z];
    if (mm < MIN_VALID_MM || mm > MAX_VALID_MM) continue;
    float w = 1.0f / (float)mm;
    dSum += mm * w;
    dW   += w;
    r.valid_count++;
  }
  if (dW > 0) r.dist_mm = dSum / dW;
  return r;
}

// ── Wire recovery helper ───────────────────────────────────────────────────────
void recoverWire() {
  Wire.setTimeout(0);      // disable timeout during reset sequence
  delay(5); Wire.end(); delay(5);
  Wire.setSDA(SDA_PIN); Wire.setSCL(SCL_PIN);
  Wire.begin(); Wire.setClock(100000);
  Wire.setTimeout(100);    // restore 100ms timeout for ranging
}

// ── Full hang watchdog ────────────────────────────────────────────────────────
// If NO frame at all arrives from either sensor for 3 seconds, the I2C bus is
// likely stuck (loose wire mid-transaction). Reset Wire hardware and re-init
// both sensors from scratch.
static uint32_t lastAnyFrame_ms = 0;

void watchdogHang() {
  if (millis() - lastAnyFrame_ms < 3000) return;
  if (millis() < 8000) return;   // skip during boot grace period

  Serial.println("  [hang watchdog] No frames for 3s — full bus + sensor recovery");
  recoverWire();
  delay(50);

  // Re-init both sensors
  digitalWrite(LPN_A_PIN, LOW);
  digitalWrite(LPN_B_PIN, LOW);
  delay(10);

  digitalWrite(LPN_A_PIN, HIGH); delay(100);
  if (sensor_a.begin(0x29, Wire)) {
    sensor_a.setAddress(0x2A);
    sensor_a.setResolution(GRID_SIZE * GRID_SIZE);
    sensor_a.setRangingFrequency(RANGING_HZ);
    sensor_a.startRanging();
    Serial.println("  [hang watchdog] Sensor A re-init OK");
  } else {
    Serial.println("  [hang watchdog] Sensor A re-init FAILED");
  }

  digitalWrite(LPN_B_PIN, HIGH); delay(100);
  if (sensor_b.begin(0x29, Wire)) {
    sensor_b.setResolution(GRID_SIZE * GRID_SIZE);
    sensor_b.setRangingFrequency(RANGING_HZ);
    sensor_b.startRanging();
    Serial.println("  [hang watchdog] Sensor B re-init OK");
  } else {
    Serial.println("  [hang watchdog] Sensor B re-init FAILED");
  }

  lastAnyFrame_ms = millis();
  lastReadA_ms    = millis();
  lastReadB_ms    = millis();
}

// ── Sensor B stale watchdog ───────────────────────────────────────────────────
static bool     bIsolated = false;
static uint32_t bRetryAt  = 0;

void watchdogB() {
  if (!bIsolated && (millis() > 5000) && (millis() - lastReadB_ms > 5000)) {
    Serial.println("  [watchdog] Sensor B stale >5s — isolating + recovering bus");
    digitalWrite(LPN_B_PIN, LOW);
    bIsolated = true;
    bRetryAt  = millis() + 30000;
    recoverWire();
  }
  if (bIsolated && millis() >= bRetryAt) {
    Serial.println("  [watchdog] Attempting Sensor B recovery...");
    digitalWrite(LPN_B_PIN, HIGH); delay(100);
    if (sensor_b.begin(0x29, Wire)) {
      sensor_b.setResolution(GRID_SIZE * GRID_SIZE);
      sensor_b.setRangingFrequency(RANGING_HZ);
      sensor_b.startRanging();
      lastReadB_ms = millis();
      bIsolated = false;
      Serial.println("  [watchdog] Sensor B recovered.");
    } else {
      Serial.println("  [watchdog] Sensor B recovery failed — retry in 30s");
      digitalWrite(LPN_B_PIN, LOW);
      recoverWire();
      bRetryAt = millis() + 30000;
    }
  }
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  blink(3, 100, 100);

  Serial.begin(115200);
  delay(3000);

  Serial.println("========================================");
  Serial.println("  Phase 3 Raw Validator");
  Serial.print  ("  Mode: "); Serial.print(GRID_SIZE); Serial.print("x");
  Serial.print  (GRID_SIZE); Serial.print(" @ "); Serial.print(RANGING_HZ);
  Serial.println("Hz");
  Serial.println("========================================");
  Serial.flush();

  pinMode(INT_A_PIN, INPUT_PULLUP);
  pinMode(INT_B_PIN, INPUT_PULLUP);
  pinMode(LPN_A_PIN, OUTPUT);
  pinMode(LPN_B_PIN, OUTPUT);

  Wire.setSDA(SDA_PIN);
  Wire.setSCL(SCL_PIN);
  Wire.begin();
  Wire.setClock(100000);   // 100kHz — safe with dual-board pull-ups (~2.35kΩ combined)

  digitalWrite(LPN_A_PIN, LOW);
  digitalWrite(LPN_B_PIN, LOW);
  delay(10);

  Serial.print("  Booting A... "); Serial.flush();
  digitalWrite(LPN_A_PIN, HIGH); delay(100);
  if (!sensor_a.begin(0x29, Wire)) panicFast("ERROR: Sensor A not found.");
  sensor_a.setAddress(0x2A);
  Serial.println("OK (0x2A)");
  blink(1, 400, 100);
  delay(50);

  Serial.print("  Booting B... "); Serial.flush();
  digitalWrite(LPN_B_PIN, HIGH); delay(100);
  bool bOk = sensor_b.begin(0x29, Wire);
  if (!bOk) {
    Serial.println("FAILED — continuing with A only");
  } else {
    Serial.println("OK (0x29)");
    blink(1, 400, 100);
  }
  delay(50);

  sensor_a.setResolution(GRID_SIZE * GRID_SIZE);
  sensor_b.setResolution(GRID_SIZE * GRID_SIZE);
  sensor_a.setRangingFrequency(RANGING_HZ);
  sensor_b.setRangingFrequency(RANGING_HZ);
  sensor_a.startRanging();
  sensor_b.startRanging();

  // Set I2C timeout AFTER begin() — firmware upload inside begin() takes
  // several seconds and must not be interrupted. 100ms per transaction is
  // safe for ranging at 100kHz (~3ms per 32-byte read). If a wire loses
  // contact mid-transaction, Wire returns an error instead of hanging forever.
  Wire.setTimeout(100);

  lastReadA_ms    = millis();
  lastReadB_ms    = millis();
  lastAnyFrame_ms = millis();

  digitalWrite(LED_BUILTIN, HIGH);

  Serial.println();
  Serial.println("  #frame | A: dist  zones  B: dist  zones | depth    lean   | jX  jY");
  Serial.println("  -------+--------------------------------+------------------+--------");
  Serial.flush();
}

// ── Loop ──────────────────────────────────────────────────────────────────────
void loop() {
  delay(8);

  bool readyA = (digitalRead(INT_A_PIN) == LOW) || sensor_a.isDataReady();
  bool readyB = (digitalRead(INT_B_PIN) == LOW) || sensor_b.isDataReady();

  bool updatedA = false, updatedB = false;

  if (readyA) {
    sensor_a.getRangingData(&results_a);
    lastReadA_ms    = millis();
    lastAnyFrame_ms = millis();
    frameCount++;
    updatedA = true;
  }
  if (readyB) {
    if (updatedA) delay(2);
    sensor_b.getRangingData(&results_b);
    lastReadB_ms    = millis();
    lastAnyFrame_ms = millis();
    updatedB = true;
  }

  watchdogHang();
  watchdogB();

  // Print one line whenever A has new data (B may or may not be fresh)
  if (updatedA) {
    SensorReading ra = computeReading(results_a);
    SensorReading rb = computeReading(results_b);

    // Depth and lean (same algorithm as phase3_position)
    bool haveA = (ra.valid_count > 0);
    bool haveB = (rb.valid_count > 0);

    float depth_mm = 0, lean_mm = 0;
    int   joyX = 128, joyY = 128;

    if (haveA || haveB) {
      float slant = haveA && haveB ? (ra.dist_mm + rb.dist_mm) * 0.5f
                  : haveA           ?  ra.dist_mm
                  :                    rb.dist_mm;
      depth_mm = slant * COS45;

      if (haveA && haveB)
        lean_mm = (ra.dist_mm - rb.dist_mm) * COS45;

      joyX = mapClamp(lean_mm,  -MAX_LEAN_MM, MAX_LEAN_MM, 0, 255);
      joyY = mapClamp(slant,     MAX_DEPTH_MM, MIN_DEPTH_MM, 0, 255);
    }

    // Freshness flags
    uint32_t now = millis();
    char flagA = (now - lastReadA_ms < 500) ? ' ' : '!';
    char flagB = updatedB                   ? ' '
               : (now - lastReadB_ms < 500) ? '~' : '?';

    // Print formatted line
    char line[128];
    char distA[12], distB[12];

    if (haveA) snprintf(distA, sizeof(distA), "%4.0fmm", ra.dist_mm);
    else        snprintf(distA, sizeof(distA), "  --  ");

    if (haveB) snprintf(distB, sizeof(distB), "%4.0fmm", rb.dist_mm);
    else        snprintf(distB, sizeof(distB), "  --  ");

    snprintf(line, sizeof(line),
             "  #%-5lu | A%c%s %2dz  B%c%s %2dz | %4.0fmm %+6.0fmm | %3d %3d",
             (unsigned long)frameCount,
             flagA, distA, ra.valid_count,
             flagB, distB, rb.valid_count,
             depth_mm, lean_mm,
             joyX, joyY);

    Serial.println(line);

    // ── Serial Plotter output ──────────────────────────────────────────────
    // Guard with availableForWrite() so toggling a trace in the plotter
    // (which briefly stalls the IDE's serial reader) never blocks the loop.
    if (Serial.availableForWrite() > 64) {
      char plot[80];
      snprintf(plot, sizeof(plot), "MIN:0 MAX:255 Depth:%d Lean:%d DistA:%d DistB:%d",
               joyY, joyX,
               haveA ? mapClamp(ra.dist_mm, MIN_DEPTH_MM, MAX_DEPTH_MM, 0, 255) : 128,
               haveB ? mapClamp(rb.dist_mm, MIN_DEPTH_MM, MAX_DEPTH_MM, 0, 255) : 128);
      Serial.println(plot);
    }
  }
}
