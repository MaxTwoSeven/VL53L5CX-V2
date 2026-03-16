/**
 * Phase 2 — Dual Sensor Live Depth Map
 * ======================================
 * Target: Raspberry Pi Pico (Arduino-Pico core — earlephilhower)
 * Library: SparkFun VL53L5CX Library
 *
 * Both sensors share one I2C bus. LPN pins assign unique addresses at boot.
 * INT pins are read directly each loop pass to gate getRangingData() calls.
 * isDataReady() is used as a fallback if the INT pin isn't wired / stays HIGH.
 *
 * Wiring:
 *   Pico GP2 (pin  4) → LPn [Sensor A]
 *   Pico GP3 (pin  5) → LPn [Sensor B]
 *   Pico GP4 (pin  6) → SDA (shared)
 *   Pico GP5 (pin  7) → SCL (shared)
 *   Pico GP6 (pin  9) → INT [Sensor A]  (optional — pulls up internally)
 *   Pico GP7 (pin 10) → INT [Sensor B]  (optional — pulls up internally)
 *
 * ⚠️  Use macOS Terminal for ANSI colors — Arduino Serial Monitor won't render them.
 *       screen /dev/cu.usbmodem<PORT> 115200
 *     Ctrl-A then K to quit.
 *
 * ── Mode selection ────────────────────────────────────────────────────────────
 */

// #define USE_8X8   // ← uncomment for 8x8 @ 15Hz (requires solid pull-ups on both boards)

// ── Derived config ────────────────────────────────────────────────────────────
#ifdef USE_8X8
  #define GRID_SIZE   8
  #define NUM_ZONES   64
  #define RANGING_HZ  15
  #define CENTER_ZONE 27
#else
  #define GRID_SIZE   4
  #define NUM_ZONES   16
  #define RANGING_HZ  60   // up to 60 Hz for 4x4
  #define CENTER_ZONE 5
#endif

#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h>

#define LPN_A_PIN    2
#define LPN_B_PIN    3
#define SDA_PIN      4
#define SCL_PIN      5
#define INT_A_PIN    6   // GP6, physical pin 9
#define INT_B_PIN    7   // GP7, physical pin 10
#define VALID_STATUS 5

SparkFun_VL53L5CX sensor_a;
SparkFun_VL53L5CX sensor_b;
VL53L5CX_ResultsData results_a;
VL53L5CX_ResultsData results_b;

// INT pins are active-low open-drain outputs from the sensor.
// Read them directly — LOW means new data is waiting on that sensor.

// ── ANSI helpers ──────────────────────────────────────────────────────────────
#define A_RESET  "\033[0m"
#define A_BOLD   "\033[1m"
#define A_CLEAR  "\033[2J\033[H"
#define A_HOME   "\033[H"
#define A_HIDE   "\033[?25l"

const char* zoneColor(int16_t mm, uint8_t status) {
  if (status != VALID_STATUS) return "\033[38;5;236m";
  if (mm <  150)  return "\033[1;97m";
  if (mm <  300)  return "\033[1;91m";
  if (mm <  500)  return "\033[38;5;208m";
  if (mm <  700)  return "\033[93m";
  if (mm <  900)  return "\033[92m";
  if (mm < 1200)  return "\033[96m";
  if (mm < 1800)  return "\033[94m";
  return           "\033[34m";
}

const char* zoneChar(int16_t mm, uint8_t status) {
  if (status != VALID_STATUS) return "\xc2\xb7\xc2\xb7"; // ··
  if (mm <  300)  return "\xe2\x96\x88\xe2\x96\x88";     // ██
  if (mm <  600)  return "\xe2\x96\x93\xe2\x96\x93";     // ▓▓
  if (mm <  900)  return "\xe2\x96\x92\xe2\x96\x92";     // ▒▒
  if (mm < 1400)  return "\xe2\x96\x91\xe2\x96\x91";     // ░░
  return          "  ";
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

void printZone(int16_t mm, uint8_t status) {
  Serial.print(zoneColor(mm, status));
  Serial.print(zoneChar(mm, status));
  Serial.print(A_RESET);
}

void printLabel(int16_t mm, uint8_t status) {
  Serial.print(zoneColor(mm, status));
  if (status == VALID_STATUS) {
    char buf[8];
    snprintf(buf, sizeof(buf), "%4dmm", mm);
    Serial.print(buf);
  } else {
    Serial.print("  -- ");
  }
  Serial.print(A_RESET);
}

// ── Hz tracking & staleness ───────────────────────────────────────────────────
uint32_t frameCount  = 0;
uint32_t lastFrameUs = 0;
float    rollingHz   = 0;
uint32_t hzBuf[8]   = {0};
uint8_t  hzIdx       = 0;

uint32_t lastReadA_ms = 0;   // millis() of most recent successful Sensor A read
uint32_t lastReadB_ms = 0;   // millis() of most recent successful Sensor B read

void updateHz() {
  uint32_t now = micros();
  if (lastFrameUs > 0) {
    hzBuf[hzIdx++ % 8] = now - lastFrameUs;
    float avg = 0;
    for (int i = 0; i < 8; i++) avg += hzBuf[i];
    avg /= 8.0f;
    rollingHz = avg > 0 ? 1000000.0f / avg : 0;
  }
  lastFrameUs = now;
}

// ── EOL helper: erase to end of line, then newline ────────────────────────────
#define EOLN  "\033[K\r\n"

// Fixed-width sensor status string (always 16 chars visible, padded with spaces)
void fmtAge(uint32_t age_ms, char* buf, size_t len) {
  if (age_ms > 1000)
    snprintf(buf, len, "\033[1;91m[STALE %4lums]\033[0m  ", (unsigned long)age_ms);
  else if (age_ms > 300)
    snprintf(buf, len, "\033[93m[%4lums ago]  \033[0m  ", (unsigned long)age_ms);
  else
    snprintf(buf, len, "\033[92m[live]       \033[0m  ");
}

// ── Draw ──────────────────────────────────────────────────────────────────────
void redraw() {
  Serial.print(A_HOME);

  // ── Title ──
  char tbuf[64];
  snprintf(tbuf, sizeof(tbuf), "  Dual VL53L5CX %dx%d  frame #%lu  %.1f Hz",
           GRID_SIZE, GRID_SIZE, (unsigned long)frameCount, rollingHz);
  Serial.print(A_BOLD); Serial.print(tbuf); Serial.print(A_RESET); Serial.print(EOLN);
  Serial.print(EOLN);

  // ── Sensor headers with fixed-width status ──
  uint32_t now_ms = millis();
  char stA[48], stB[48];
  fmtAge(now_ms - lastReadA_ms, stA, sizeof(stA));
  fmtAge(now_ms - lastReadB_ms, stB, sizeof(stB));

  Serial.print("  ");
  Serial.print(A_BOLD "\033[91m" "Sensor A  chest (0x2A)" A_RESET "  ");
  Serial.print(stA);
  Serial.print("      ");
  Serial.print(A_BOLD "\033[94m" "Sensor B  lean  (0x29)" A_RESET "  ");
  Serial.print(stB);
  Serial.print(EOLN);
  Serial.print(EOLN);

  // ── Grid rows ──
  for (int row = 0; row < GRID_SIZE; row++) {
    Serial.print("  ");
    for (int col = 0; col < GRID_SIZE; col++) {
      int zone = row * GRID_SIZE + col;
      printZone(results_a.distance_mm[zone], results_a.target_status[zone]);
    }
    int labelZone = row * GRID_SIZE + (GRID_SIZE / 2 - 1);
    Serial.print("  ");
    printLabel(results_a.distance_mm[labelZone], results_a.target_status[labelZone]);
    Serial.print("      ");
    for (int col = 0; col < GRID_SIZE; col++) {
      int zone = row * GRID_SIZE + col;
      printZone(results_b.distance_mm[zone], results_b.target_status[zone]);
    }
    Serial.print("  ");
    printLabel(results_b.distance_mm[labelZone], results_b.target_status[labelZone]);
    Serial.print(EOLN);
  }

  // ── Legend ──
  Serial.print(EOLN);
  Serial.print("  ");
  const char* colors[] = {"\033[1;97m","\033[1;91m","\033[38;5;208m","\033[93m",
                           "\033[92m",  "\033[96m",  "\033[94m",       "\033[34m",
                           "\033[38;5;236m"};
  const char* labels[] = {"<150 ","<300 ","<500 ","<700 ",
                           "<900 ","<1200","<1800"," far ","none "};
  for (int i = 0; i < 9; i++) {
    Serial.print(colors[i]); Serial.print(labels[i]); Serial.print(A_RESET); Serial.print(" ");
  }
  Serial.print(EOLN);

  // ── Center zone readout ──
  Serial.print(EOLN);
  Serial.print("  A center["); Serial.print(CENTER_ZONE); Serial.print("]:  ");
  printLabel(results_a.distance_mm[CENTER_ZONE], results_a.target_status[CENTER_ZONE]);
  Serial.print("        B center["); Serial.print(CENTER_ZONE); Serial.print("]:  ");
  printLabel(results_b.distance_mm[CENTER_ZONE], results_b.target_status[CENTER_ZONE]);
  Serial.print(EOLN);

  // Erase any leftover lines below this frame (handles mode switches / resize)
  Serial.print("\033[J");
  // No flush — flush blocks until the terminal drains the buffer.
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  blink(3, 100, 100);

  Serial.begin(115200);
  delay(3000);

  Serial.print(A_HIDE);
  Serial.print(A_CLEAR);
  Serial.println(A_BOLD "Phase 2 — Dual Sensor Depth Map (INT-driven)" A_RESET);
  Serial.flush();

  // INT pins — active low, open-drain output from sensor; pull up on Pico side
  pinMode(INT_A_PIN, INPUT_PULLUP);
  pinMode(INT_B_PIN, INPUT_PULLUP);

  pinMode(LPN_A_PIN, OUTPUT);
  pinMode(LPN_B_PIN, OUTPUT);

  Wire.setSDA(SDA_PIN);
  Wire.setSCL(SCL_PIN);
  Wire.begin();
  Wire.setClock(400000);
  // No Wire timeout — getRangingData() is only called when INT pin is LOW
  // (hardware-confirmed ready), so I2C reads are always fast and clean.

  digitalWrite(LPN_A_PIN, LOW);
  digitalWrite(LPN_B_PIN, LOW);
  delay(10);

  Serial.println("  Booting Sensor A (~2s)..."); Serial.flush();
  digitalWrite(LPN_A_PIN, HIGH);
  delay(100);
  if (!sensor_a.begin(0x29, Wire)) panicFast("  ERROR: Sensor A not found.");
  sensor_a.setAddress(0x2A);
  blink(1, 400, 100);
  Serial.println("  Sensor A → 0x2A  OK"); Serial.flush();
  delay(50);

  Serial.println("  Booting Sensor B (~2s)..."); Serial.flush();
  digitalWrite(LPN_B_PIN, HIGH);
  delay(100);
  if (!sensor_b.begin(0x29, Wire)) panicFast("  ERROR: Sensor B not found.");
  blink(1, 400, 100);
  Serial.println("  Sensor B → 0x29  OK"); Serial.flush();
  delay(50);

  sensor_a.setResolution(GRID_SIZE * GRID_SIZE);
  sensor_b.setResolution(GRID_SIZE * GRID_SIZE);
  sensor_a.setRangingFrequency(RANGING_HZ);
  sensor_b.setRangingFrequency(RANGING_HZ);
  sensor_a.startRanging();
  sensor_b.startRanging();

  // No ISRs needed — INT pins are read directly in the loop

  // Anchor watchdog clock to NOW so the 3 s stale window starts from when
  // ranging begins, not from millis()=0 at declaration (which would cause
  // the watchdog to fire immediately on the first loop pass).
  lastReadA_ms = millis();
  lastReadB_ms = millis();

  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("  Both sensors ranging (INT-driven).\n"); Serial.flush();
}

// ── Loop ──────────────────────────────────────────────────────────────────────
// Readiness is determined by the INT pins ONLY — no isDataReady() I2C calls.
// isDataReady() was blocking indefinitely when the bus was marginal, starving
// the entire loop. INT pins are hardware-asserted by the sensor (active-low,
// open-drain) and carry zero I2C overhead.
//
// The force-redraw fires at the TOP of the loop so the display updates even
// when no sensor data arrives, and even if getRangingData() takes time.

static uint32_t lastFullClear_ms = 0;

void loop() {
  // ── Force-redraw — guaranteed output every 5 s; skip if buffer is full ───
  if (millis() - lastFullClear_ms >= 5000) {
    lastFullClear_ms = millis();
    if (Serial.availableForWrite() > 0) {
      Serial.print(A_CLEAR);
      redraw();
    }
  }

  delay(8);

  // INT pin is the fast path; isDataReady() is the fallback for boards where
  // INT doesn't assert reliably.  Short-circuit means isDataReady() is only
  // called over I2C when the INT pin is NOT already LOW.
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

  // ── Sensor B bus-isolation watchdog ──────────────────────────────────────
  static bool     bIsolated = false;
  static uint32_t bRetryAt  = 0;

  auto recoverWire = []() {
    delay(5);
    Wire.end();
    delay(5);
    Wire.setSDA(SDA_PIN);
    Wire.setSCL(SCL_PIN);
    Wire.begin();
    Wire.setClock(400000);
  };

  if (!bIsolated && (millis() > 5000) && (millis() - lastReadB_ms > 5000)) {
    digitalWrite(LPN_B_PIN, LOW);
    bIsolated = true;
    bRetryAt  = millis() + 30000;
    recoverWire();   // clear any stuck-bus state before Sensor A resumes
  }

  if (bIsolated && millis() >= bRetryAt) {
    digitalWrite(LPN_B_PIN, HIGH);
    delay(100);
    if (sensor_b.begin(0x29, Wire)) {
      sensor_b.setResolution(GRID_SIZE * GRID_SIZE);
      sensor_b.setRangingFrequency(RANGING_HZ);
      sensor_b.startRanging();
      lastReadB_ms = millis();
      bIsolated = false;
      bRetryAt  = 0;
    } else {
      digitalWrite(LPN_B_PIN, LOW);
      recoverWire();
      bRetryAt = millis() + 30000;
    }
  }

  if (updated && Serial.availableForWrite() > 0) redraw();
}
