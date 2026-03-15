/**
 * Phase 3 — Rider Position Mapping
 * ==================================
 * Target:  Raspberry Pi Pico (Arduino-Pico / earlephilhower)
 * Library: SparkFun VL53L5CX Library
 *
 * Fuses data from two VL53L5CX sensors in a V-geometry mount to compute
 * the rider's 2D position: depth (forward / back) and lean (left / right).
 *
 * ── Algorithm ──────────────────────────────────────────────────────────────
 *   Both sensors face the rider from opposite 45° angles.
 *   Each reports a slant distance to the nearest part of the rider's torso.
 *
 *     depth  =  (dist_A + dist_B) / 2  ×  cos(45°)   [perpendicular F/B]
 *     lean   =  (dist_A − dist_B)      ×  cos(45°)   [lateral L/R diff]
 *
 *   A secondary lean signal is derived from the column centroid of each
 *   sensor's zone grid.  Its weight is tunable (CENTROID_WEIGHT).
 *
 * ── Sensor geometry (top-down) ─────────────────────────────────────────────
 *
 *     [A]  ↘ 45°  ──── 300mm ────  45° ↙  [B]
 *                    ↘        ↙
 *                      [Rider]
 *
 *   Sensor A = LEFT  (chest label, address 0x2A)
 *   Sensor B = RIGHT (lean  label, address 0x29)
 *
 *   Rider leans LEFT  → dist_A decreases, dist_B increases → lean < 0
 *   Rider leans RIGHT → dist_A increases, dist_B decreases → lean > 0
 *
 * ── Wiring (same as phase2_depthmap) ───────────────────────────────────────
 *   Pico GP2 (pin  4) → LPn [Sensor A]
 *   Pico GP3 (pin  5) → LPn [Sensor B]
 *   Pico GP4 (pin  6) → SDA (shared)
 *   Pico GP5 (pin  7) → SCL (shared)
 *   Pico GP6 (pin  9) → INT [Sensor A]
 *   Pico GP7 (pin 10) → INT [Sensor B]
 *
 * ⚠️  Use macOS Terminal for ANSI colors.
 *       screen /dev/cu.usbmodem<PORT> 115200
 *     Ctrl-A  then  \  then  Y  to quit.
 *
 * ── Calibration workflow ───────────────────────────────────────────────────
 *   1. Sit in the neutral (center, mid-depth) position.
 *      Watch "dist A / dist B" — both should read similarly.
 *   2. Lean all the way forward.  Note "dist A" and "dist B" min values
 *      → set MIN_DEPTH_MM to that slant reading (or slightly lower).
 *   3. Lean all the way back.  Note the slant max values
 *      → set MAX_DEPTH_MM accordingly.
 *   4. Lean hard left then hard right.  Note the "lean" readout peaks
 *      → set MAX_LEAN_MM to that absolute value (or slightly higher).
 *   5. Adjust CENTROID_WEIGHT (0–1) to blend in the zone-grid lean signal.
 *      Start at 0 (distance-only) and increase if lean feels sluggish.
 */

// ── Mode ──────────────────────────────────────────────────────────────────────
#define USE_8X8   // uncomment for 8x8 @ 15 Hz

// ── Derived grid constants ────────────────────────────────────────────────────
#ifdef USE_8X8
  #define GRID_SIZE   8
  #define NUM_ZONES  64
  #define RANGING_HZ 15
#else
  #define GRID_SIZE   4
  #define NUM_ZONES  16
  #define RANGING_HZ 60
#endif

// ── Position tuning ───────────────────────────────────────────────────────────
//
// All distances below are SLANT mm (what the sensor hardware reports).
// perpendicular mm = slant × cos(45°) = slant × 0.7071
//
// Example: rider at 300mm perp → slant ≈ 424mm
//          rider at 500mm perp → slant ≈ 707mm
//
// Start here, then calibrate per the instructions above.

#define MIN_VALID_MM         50    // exclude glare / noise (hard min)
#define MAX_VALID_MM        800    // exclude background (static ~930mm readings)

#define MIN_DEPTH_MM        200    // slant at rider's closest forward lean
#define MAX_DEPTH_MM        700    // slant at rider's farthest back lean
#define MAX_LEAN_MM         200    // ±slant differential at full left / right lean

// Centroid contribution: 0.0 = distance differential only (safe default).
// Increase toward 1.0 after calibrating centroid direction for your mount.
#define CENTROID_WEIGHT    0.0f
#define CENTROID_MM_PER_COL 40.0f  // approximate mm of lean per column shift
#define COS45              0.7071f  // cos(45°) — slant → perpendicular conversion

// ── Hardware ──────────────────────────────────────────────────────────────────
#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h>

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

// ── ANSI helpers ──────────────────────────────────────────────────────────────
#define A_RESET "\033[0m"
#define A_BOLD  "\033[1m"
#define A_CLEAR "\033[2J\033[H"
#define A_HOME  "\033[H"
#define A_HIDE  "\033[?25l"
#define EOLN    "\033[K\r\n"

// ── Position data types ───────────────────────────────────────────────────────

struct SensorReading {
  float dist_mm;       // inverse-distance-weighted average of valid zones (slant)
  float col_centroid;  // weighted column centroid: 0.0 = left, GRID_SIZE-1 = right
  int   valid_count;   // number of zones that passed MIN/MAX_VALID_MM filter
};

struct RiderPos {
  float depth_mm;  // perpendicular depth, forward (small = close to sensors)
  float lean_mm;   // lateral lean: negative = left (A-side), positive = right (B-side)
  int   joyX;      // 0–255 lean  (0=full left, 128=center, 255=full right)
  int   joyY;      // 0–255 depth (0=full back, 255=full forward / closest)
  bool  valid;
};

// ── Hz / staleness tracking ───────────────────────────────────────────────────
uint32_t frameCount   = 0;
uint32_t lastFrameUs  = 0;
float    rollingHz    = 0.0f;
uint32_t hzBuf[8]     = {0};
uint8_t  hzIdx        = 0;
uint32_t lastReadA_ms = 0;
uint32_t lastReadB_ms = 0;

void updateHz() {
  uint32_t now = micros();
  if (lastFrameUs > 0) {
    hzBuf[hzIdx++ % 8] = now - lastFrameUs;
    float avg = 0;
    for (int i = 0; i < 8; i++) avg += hzBuf[i];
    avg /= 8.0f;
    rollingHz = (avg > 0) ? 1000000.0f / avg : 0.0f;
  }
  lastFrameUs = now;
}

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

void fmtAge(uint32_t age_ms, char* buf, size_t len) {
  if      (age_ms > 1000) snprintf(buf, len, "\033[1;91m[STALE %4lums]\033[0m  ", (unsigned long)age_ms);
  else if (age_ms > 300)  snprintf(buf, len, "\033[93m[%4lums ago]  \033[0m  ",   (unsigned long)age_ms);
  else                    snprintf(buf, len, "\033[92m[live]       \033[0m  ");
}

// ── Position computation ──────────────────────────────────────────────────────

SensorReading computeReading(VL53L5CX_ResultsData& data) {
  SensorReading r = {-1.0f, (GRID_SIZE - 1) * 0.5f, 0};
  float dSum = 0, dW = 0, cSum = 0, cW = 0;

  for (int z = 0; z < NUM_ZONES; z++) {
    if (data.target_status[z] != VALID_STATUS) continue;
    int16_t mm = data.distance_mm[z];
    if (mm < MIN_VALID_MM || mm > MAX_VALID_MM) continue;

    int   col = z % GRID_SIZE;
    float w   = 1.0f / (float)mm;  // inverse-distance weight: closer = more weight

    dSum += mm * w;  dW += w;
    cSum += col * w; cW += w;
    r.valid_count++;
  }

  if (dW > 0) {
    r.dist_mm      = dSum / dW;
    r.col_centroid = (cW > 0) ? cSum / cW : (GRID_SIZE - 1) * 0.5f;
  }
  return r;
}

RiderPos computePosition(SensorReading& ra, SensorReading& rb) {
  RiderPos p = {0.0f, 0.0f, 128, 128, false};

  bool haveA = (ra.valid_count > 0);
  bool haveB = (rb.valid_count > 0);
  if (!haveA && !haveB) return p;
  p.valid = true;

  // ── Depth ──
  // Average both sensors when both are valid; fall back to whichever is alive.
  float slant = haveA && haveB ? (ra.dist_mm + rb.dist_mm) * 0.5f
              : haveA           ?  ra.dist_mm
              :                    rb.dist_mm;
  p.depth_mm = slant * COS45;

  // ── Lean — primary: distance differential ──
  // Sensor A = LEFT.  A closer (smaller dist_A) → rider leaning LEFT → lean < 0.
  // Sensor B = RIGHT. B closer (smaller dist_B) → rider leaning RIGHT → lean > 0.
  float lean_dist = 0.0f;
  if (haveA && haveB)
    lean_dist = (ra.dist_mm - rb.dist_mm) * COS45;
  // dist_A < dist_B → lean_dist < 0 → leaning LEFT ✓

  // ── Lean — secondary: zone column centroid ──
  // CENTROID_WEIGHT is 0.0 by default; uncomment and calibrate for your mount.
  // Expected directions (adjust sign if your sensor is flipped):
  //   Sensor A (left, pointing right): higher col_centroid → rider shifted right → +lean
  //   Sensor B (right, pointing left): lower  col_centroid → rider shifted right → +lean
  float lean_centroid = 0.0f;
  if (CENTROID_WEIGHT > 0.0f && haveA && haveB) {
    float center = (GRID_SIZE - 1) * 0.5f;
    float cA =  (ra.col_centroid - center) * CENTROID_MM_PER_COL;
    float cB = -(rb.col_centroid - center) * CENTROID_MM_PER_COL;
    lean_centroid = (cA + cB) * 0.5f;
  }

  p.lean_mm = lean_dist    * (1.0f - CENTROID_WEIGHT)
            + lean_centroid *         CENTROID_WEIGHT;

  // ── Joystick mapping ──
  // Joy X: 0 = full left lean, 128 = center, 255 = full right lean
  p.joyX = mapClamp(p.lean_mm, -MAX_LEAN_MM, MAX_LEAN_MM, 0, 255);
  // Joy Y: 0 = full back (far), 255 = full forward (close to sensors)
  p.joyY = mapClamp(slant, MAX_DEPTH_MM, MIN_DEPTH_MM, 0, 255);

  return p;
}

// ── Zone display (same palette as phase2_depthmap) ────────────────────────────

const char* zoneColor(int16_t mm, uint8_t status) {
  if (status != VALID_STATUS) return "\033[38;5;236m";
  if (mm <  150) return "\033[1;97m";
  if (mm <  300) return "\033[1;91m";
  if (mm <  500) return "\033[38;5;208m";
  if (mm <  700) return "\033[93m";
  if (mm <  900) return "\033[92m";
  if (mm < 1200) return "\033[96m";
  if (mm < 1800) return "\033[94m";
  return "\033[34m";
}

const char* zoneChar(int16_t mm, uint8_t status) {
  if (status != VALID_STATUS) return "\xc2\xb7\xc2\xb7";        // ··
  if (mm <  300) return "\xe2\x96\x88\xe2\x96\x88";             // ██
  if (mm <  600) return "\xe2\x96\x93\xe2\x96\x93";             // ▓▓
  if (mm <  900) return "\xe2\x96\x92\xe2\x96\x92";             // ▒▒
  if (mm < 1400) return "\xe2\x96\x91\xe2\x96\x91";             // ░░
  return "  ";
}

void printZone(int16_t mm, uint8_t status) {
  Serial.print(zoneColor(mm, status));
  Serial.print(zoneChar(mm, status));
  Serial.print(A_RESET);
}

// ── Position indicator box ────────────────────────────────────────────────────
// A fixed character grid.  The ◆ marker moves with the rider's position.
//   F (top)    = rider closest / most forward
//   B (bottom) = rider farthest / most backward
//   L (left)   = lean left      R (right) = lean right

#define BOX_W  24   // inner character columns
#define BOX_H   7   // inner character rows

void drawPositionBox(RiderPos& pos, SensorReading& ra, SensorReading& rb) {
  // Map rider position onto box grid
  int bx = BOX_W / 2, by = BOX_H / 2;   // default: center (no target)
  if (pos.valid) {
    bx = mapClamp(pos.lean_mm,  -MAX_LEAN_MM, MAX_LEAN_MM, 0, BOX_W - 1);
    by = mapClamp(pos.depth_mm,  MIN_DEPTH_MM * COS45, MAX_DEPTH_MM * COS45, 0, BOX_H - 1);
    // Y: small depth (close) → top row (by=0); large depth (far) → bottom row
  }

  char ann[56];   // right-side annotation buffer

  // ── Top border ──
  Serial.print("  F \xe2\x94\x8c");                   // F ┌
  for (int i = 0; i < BOX_W; i++) Serial.print("\xe2\x94\x80");  // ─
  Serial.print("\xe2\x94\x90" EOLN);                  // ┐

  // ── Rows ──
  for (int row = 0; row < BOX_H; row++) {
    Serial.print("    \xe2\x94\x82");                  // │
    for (int col = 0; col < BOX_W; col++) {
      if (row == by && col == bx)
        Serial.print(pos.valid
          ? "\033[1;93m\xe2\x97\x86\033[0m"           // ◆ amber when valid
          : "\033[38;5;240m\xc2\xb7\033[0m");          // · grey when no target
      else
        Serial.print(" ");
    }
    Serial.print("\xe2\x94\x82");                      // │

    // Right-side numeric readout
    switch (row) {
      case 0:
        Serial.print("   \033[91m" A_BOLD "A\033[0m (left)   ");
        if (ra.valid_count > 0)
          snprintf(ann, sizeof(ann), "slant %4.0fmm  %d zones", ra.dist_mm, ra.valid_count);
        else
          snprintf(ann, sizeof(ann), "no valid zones");
        Serial.print(ann);
        break;
      case 1:
        Serial.print("   \033[94m" A_BOLD "B\033[0m (right)  ");
        if (rb.valid_count > 0)
          snprintf(ann, sizeof(ann), "slant %4.0fmm  %d zones", rb.dist_mm, rb.valid_count);
        else
          snprintf(ann, sizeof(ann), "no valid zones");
        Serial.print(ann);
        break;
      case 3:
        if (pos.valid)
          snprintf(ann, sizeof(ann), "   depth  %4.0fmm   lean %+.0fmm",
                   pos.depth_mm, pos.lean_mm);
        else
          snprintf(ann, sizeof(ann), "   depth    --    lean    --");
        Serial.print(ann);
        break;
      case 4:
        snprintf(ann, sizeof(ann), "   Joy X (lean) :  %3d / 255", pos.joyX);
        Serial.print(ann);
        break;
      case 5:
        snprintf(ann, sizeof(ann), "   Joy Y (depth):  %3d / 255", pos.joyY);
        Serial.print(ann);
        break;
    }
    Serial.print(EOLN);
  }

  // ── Bottom border ──
  Serial.print("  B \xe2\x94\x94");                   // B └
  for (int i = 0; i < BOX_W; i++) Serial.print("\xe2\x94\x80");  // ─
  Serial.print("\xe2\x94\x98" EOLN);                  // ┘

  // ── L / R labels ──
  Serial.print("    L");
  for (int i = 0; i < BOX_W - 1; i++) Serial.print(" ");
  Serial.print("R" EOLN);
}

// ── Full screen redraw ────────────────────────────────────────────────────────
void redraw(RiderPos& pos, SensorReading& ra, SensorReading& rb) {
  Serial.print(A_HOME);

  // ── Title bar ──
  char tbuf[80];
  snprintf(tbuf, sizeof(tbuf),
           "  Phase 3 — Rider Position  %dx%d  frame #%lu  %.1f Hz",
           GRID_SIZE, GRID_SIZE, (unsigned long)frameCount, rollingHz);
  Serial.print(A_BOLD); Serial.print(tbuf); Serial.print(A_RESET); Serial.print(EOLN);
  Serial.print(EOLN);

  // ── Sensor freshness headers ──
  uint32_t now_ms = millis();
  char stA[52], stB[52];
  fmtAge(now_ms - lastReadA_ms, stA, sizeof(stA));
  fmtAge(now_ms - lastReadB_ms, stB, sizeof(stB));

  Serial.print("  ");
  Serial.print(A_BOLD "\033[91m" "Sensor A  left  (0x2A)" A_RESET "  ");
  Serial.print(stA);
  Serial.print("      ");
  Serial.print(A_BOLD "\033[94m" "Sensor B  right (0x29)" A_RESET "  ");
  Serial.print(stB);
  Serial.print(EOLN);
  Serial.print(EOLN);

  // ── Raw zone grids (compact — no per-zone label) ──
  for (int row = 0; row < GRID_SIZE; row++) {
    Serial.print("  ");
    for (int col = 0; col < GRID_SIZE; col++)
      printZone(results_a.distance_mm[row * GRID_SIZE + col],
                results_a.target_status[row * GRID_SIZE + col]);
    Serial.print("      ");
    for (int col = 0; col < GRID_SIZE; col++)
      printZone(results_b.distance_mm[row * GRID_SIZE + col],
                results_b.target_status[row * GRID_SIZE + col]);
    Serial.print(EOLN);
  }
  Serial.print(EOLN);

  // ── Position panel ──
  Serial.print("  " A_BOLD "── Position ──────────────────────────────────────────────────────────" A_RESET EOLN);
  Serial.print(EOLN);

  if (!pos.valid)
    Serial.print("  \033[38;5;240m[no valid zones — move into sensor range]\033[0m" EOLN EOLN);

  drawPositionBox(pos, ra, rb);

  // Erase any leftover content below (handles terminal resize)
  Serial.print("\033[J");
  Serial.flush();
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  blink(3, 100, 100);

  Serial.begin(115200);
  delay(3000);

  Serial.print(A_HIDE A_CLEAR);
  Serial.println(A_BOLD "Phase 3 — Rider Position Mapping" A_RESET);
  Serial.flush();

  pinMode(INT_A_PIN, INPUT_PULLUP);
  pinMode(INT_B_PIN, INPUT_PULLUP);
  pinMode(LPN_A_PIN, OUTPUT);
  pinMode(LPN_B_PIN, OUTPUT);

  Wire.setSDA(SDA_PIN);
  Wire.setSCL(SCL_PIN);
  Wire.begin();
  Wire.setClock(400000);

  digitalWrite(LPN_A_PIN, LOW);
  digitalWrite(LPN_B_PIN, LOW);
  delay(10);

  Serial.println("  Booting Sensor A..."); Serial.flush();
  digitalWrite(LPN_A_PIN, HIGH); delay(100);
  if (!sensor_a.begin(0x29, Wire)) panicFast("  ERROR: Sensor A not found.");
  sensor_a.setAddress(0x2A);
  blink(1, 400, 100);
  Serial.println("  Sensor A \xe2\x86\x92 0x2A  OK"); Serial.flush();
  delay(50);

  Serial.println("  Booting Sensor B..."); Serial.flush();
  digitalWrite(LPN_B_PIN, HIGH); delay(100);
  if (!sensor_b.begin(0x29, Wire)) panicFast("  ERROR: Sensor B not found.");
  blink(1, 400, 100);
  Serial.println("  Sensor B \xe2\x86\x92 0x29  OK"); Serial.flush();
  delay(50);

  sensor_a.setResolution(GRID_SIZE * GRID_SIZE);
  sensor_b.setResolution(GRID_SIZE * GRID_SIZE);
  sensor_a.setRangingFrequency(RANGING_HZ);
  sensor_b.setRangingFrequency(RANGING_HZ);
  sensor_a.startRanging();
  sensor_b.startRanging();

  // Anchor the watchdog clock to NOW so the 3 s stale window starts from
  // the moment ranging begins — not from millis()=0 at declaration, which
  // would cause the watchdog to fire immediately on the first loop pass.
  lastReadA_ms = millis();
  lastReadB_ms = millis();

  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("  Both sensors ranging.\n"); Serial.flush();
}

// ── Sensor B watchdog ─────────────────────────────────────────────────────────
static bool     bIsolated = false;
static uint32_t bRetryAt  = 0;

void watchdogB() {
  if (!bIsolated && (millis() > 5000) && (millis() - lastReadB_ms > 3000)) {
    digitalWrite(LPN_B_PIN, LOW);
    bIsolated = true;
    bRetryAt  = millis() + 30000;
  }
  if (bIsolated && millis() >= bRetryAt) {
    digitalWrite(LPN_B_PIN, HIGH); delay(100);
    if (sensor_b.begin(0x29, Wire)) {
      sensor_b.setResolution(GRID_SIZE * GRID_SIZE);
      sensor_b.setRangingFrequency(RANGING_HZ);
      sensor_b.startRanging();
      lastReadB_ms = millis();
      bIsolated = false;
    } else {
      digitalWrite(LPN_B_PIN, LOW);
      bRetryAt = millis() + 30000;
    }
  }
}

// ── Loop ──────────────────────────────────────────────────────────────────────
static uint32_t   lastFullClear_ms = 0;
static SensorReading ra_last = {-1.0f, 0.0f, 0};
static SensorReading rb_last = {-1.0f, 0.0f, 0};
static RiderPos      pos_last = {0.0f, 0.0f, 128, 128, false};

void loop() {
  // Guaranteed full redraw every 5 s (catches late-connecting terminals)
  if (millis() - lastFullClear_ms >= 5000) {
    Serial.print(A_CLEAR);
    lastFullClear_ms = millis();
    redraw(pos_last, ra_last, rb_last);
  }

  delay(8);

  // INT pin = fast path (zero I2C overhead); isDataReady() = fallback
  bool readyA = (digitalRead(INT_A_PIN) == LOW) || sensor_a.isDataReady();
  bool readyB = (digitalRead(INT_B_PIN) == LOW) || sensor_b.isDataReady();

  bool updated = false;

  if (readyA) {
    sensor_a.getRangingData(&results_a);
    lastReadA_ms = millis();
    frameCount++;
    updateHz();
    updated = true;
  }
  if (readyB) {
    if (updated) delay(2);
    sensor_b.getRangingData(&results_b);
    lastReadB_ms = millis();
    updated = true;
  }

  watchdogB();

  if (updated) {
    ra_last  = computeReading(results_a);
    rb_last  = computeReading(results_b);
    pos_last = computePosition(ra_last, rb_last);
    redraw(pos_last, ra_last, rb_last);
  }
}
