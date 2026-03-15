/**
 * Phase 1 — Live Depth Map (4x4 / 8x8)
 * ======================================
 * Target: Raspberry Pi Pico (Arduino-Pico core — earlephilhower)
 * Library: SparkFun VL53L5CX Library
 *
 * Renders a full-grid color depth map in the terminal.
 * Each zone is drawn as a colored block — color encodes distance.
 *
 * ⚠️  Arduino IDE's built-in Serial Monitor does NOT render ANSI colors.
 *     For colors, open macOS Terminal and run:
 *       screen /dev/cu.usbmodem<YOUR_PORT> 115200
 *     Press Ctrl-A then K to quit screen.
 *
 * ── Mode selection ────────────────────────────────────────────────────────────
 *   Comment / uncomment the line below to switch modes:
 */

#define USE_8X8   // ← comment this out for 4x4 @ 60Hz

// ── Derived config ────────────────────────────────────────────────────────────
#ifdef USE_8X8
  #define GRID_SIZE    8
  #define NUM_ZONES    64
  #define RANGING_HZ   15
  #define CENTER_ZONE  27   // row 3, col 3
#else
  #define GRID_SIZE    4
  #define NUM_ZONES    16
  #define RANGING_HZ   60
  #define CENTER_ZONE  5    // row 1, col 1
#endif

#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h>

#define LPN_A_PIN    2
#define SDA_PIN      4
#define SCL_PIN      5
#define VALID_STATUS 5

SparkFun_VL53L5CX sensor;
VL53L5CX_ResultsData results;

// ── ANSI helpers ──────────────────────────────────────────────────────────────
#define A_RESET   "\033[0m"
#define A_BOLD    "\033[1m"
#define A_CLEAR   "\033[2J\033[H"   // clear screen + cursor home
#define A_HIDE    "\033[?25l"       // hide cursor (cleaner redraws)

// Distance → ANSI color  (nearest → hottest, farthest → coolest)
const char* zoneColor(int16_t mm, uint8_t status) {
  if (status != VALID_STATUS) return "\033[38;5;236m";  // dark gray  = no target
  if (mm <  150)  return "\033[1;97m";                  // bold white = wall / collision
  if (mm <  300)  return "\033[1;91m";                  // bold red
  if (mm <  500)  return "\033[38;5;208m";              // orange
  if (mm <  700)  return "\033[93m";                    // bright yellow
  if (mm <  900)  return "\033[92m";                    // bright green
  if (mm < 1200)  return "\033[96m";                    // bright cyan
  if (mm < 1800)  return "\033[94m";                    // bright blue
  return           "\033[34m";                           // dark blue   = far
}

// Distance → display character  (closer = denser block)
const char* zoneChar(int16_t mm, uint8_t status) {
  if (status != VALID_STATUS) return "\xc2\xb7\xc2\xb7"; // ·· (UTF-8 middle dot x2)
  if (mm <  300)  return "\xe2\x96\x88\xe2\x96\x88";     // ██ full block
  if (mm <  600)  return "\xe2\x96\x93\xe2\x96\x93";     // ▓▓ dark shade
  if (mm <  900)  return "\xe2\x96\x92\xe2\x96\x92";     // ▒▒ medium shade
  if (mm < 1400)  return "\xe2\x96\x91\xe2\x96\x91";     // ░░ light shade
  return          "  ";                                   //    space = very far
}

// ── Legend ────────────────────────────────────────────────────────────────────
void printLegend() {
  Serial.println();
  Serial.print("  ");
  struct { const char* col; const char* label; } bands[] = {
    {"\033[1;97m",        " <150 "},
    {"\033[1;91m",        " <300 "},
    {"\033[38;5;208m",    " <500 "},
    {"\033[93m",          " <700 "},
    {"\033[92m",          " <900 "},
    {"\033[96m",          "<1200 "},
    {"\033[94m",          "<1800 "},
    {"\033[34m",          " far  "},
    {"\033[38;5;236m",    " none "},
  };
  for (auto& b : bands) {
    Serial.print(b.col);
    Serial.print(b.label);
    Serial.print(A_RESET);
    Serial.print(" ");
  }
  Serial.println(A_RESET);
}

// ── Stats for center zone ─────────────────────────────────────────────────────
uint32_t frameCount   = 0;
uint32_t lastFrameUs  = 0;
float    rollingHz    = 0;
uint32_t hzIntervals[8] = {0};
uint8_t  hzIdx        = 0;

void updateHz() {
  uint32_t now = micros();
  if (lastFrameUs > 0) {
    hzIntervals[hzIdx++ % 8] = now - lastFrameUs;
    float avg = 0;
    for (int i = 0; i < 8; i++) avg += hzIntervals[i];
    avg /= 8.0f;
    rollingHz = (avg > 0) ? 1000000.0f / avg : 0;
  }
  lastFrameUs = now;
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_BUILTIN, HIGH); delay(100);
    digitalWrite(LED_BUILTIN, LOW);  delay(100);
  }

  Serial.begin(115200);
  delay(3000);

  Serial.print(A_HIDE);   // hide cursor
  Serial.print(A_CLEAR);
  Serial.println(A_BOLD "Phase 1 — Depth Map" A_RESET);
  Serial.print("  Mode: ");
  Serial.print(GRID_SIZE); Serial.print("x"); Serial.print(GRID_SIZE);
  Serial.print("  |  Max rate: "); Serial.print(RANGING_HZ); Serial.println("Hz");
  Serial.println("  Initializing sensor (~2s)...");
  Serial.flush();

  pinMode(LPN_A_PIN, OUTPUT);
  digitalWrite(LPN_A_PIN, LOW);  delay(10);
  digitalWrite(LPN_A_PIN, HIGH); delay(100);

  Wire.setSDA(SDA_PIN);
  Wire.setSCL(SCL_PIN);
  Wire.begin();
  Wire.setClock(400000);

  if (!sensor.begin(0x29, Wire)) {
    Serial.println("  ERROR: sensor not found. Check wiring.");
    Serial.flush();
    while (true) { digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); delay(200); }
  }

  sensor.setResolution(GRID_SIZE * GRID_SIZE);
  sensor.setRangingFrequency(RANGING_HZ);
  sensor.startRanging();

  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("  Sensor ready.\n");
  Serial.flush();
}

// ── Loop ──────────────────────────────────────────────────────────────────────
void loop() {
  if (!sensor.isDataReady()) return;

  sensor.getRangingData(&results);
  frameCount++;
  updateHz();

  // Cursor home (redraws in-place without flicker)
  Serial.print("\033[H");

  // ── Header ────────────────────────────────────────────────────────────────
  Serial.print(A_BOLD);
  Serial.print("  VL53L5CX  ");
  Serial.print(GRID_SIZE); Serial.print("x"); Serial.print(GRID_SIZE);
  Serial.print("   frame #"); Serial.print(frameCount);
  Serial.print("   ");
  Serial.print(rollingHz, 1);
  Serial.print(" Hz");
  Serial.println("          " A_RESET);  // trailing spaces erase old chars
  Serial.println();

  // ── Grid ──────────────────────────────────────────────────────────────────
  for (int row = 0; row < GRID_SIZE; row++) {
    Serial.print("  ");
    for (int col = 0; col < GRID_SIZE; col++) {
      int     zone = row * GRID_SIZE + col;
      int16_t mm   = results.distance_mm[zone];
      uint8_t st   = results.target_status[zone];

      Serial.print(zoneColor(mm, st));
      Serial.print(zoneChar(mm, st));
      Serial.print(A_RESET);
    }

    // Numeric readout for the center column on each row
    int centerCol  = GRID_SIZE / 2 - 1;
    int labelZone  = row * GRID_SIZE + centerCol;
    int16_t lmm    = results.distance_mm[labelZone];
    uint8_t lst    = results.target_status[labelZone];

    Serial.print("  ");
    Serial.print(zoneColor(lmm, lst));
    if (lst == VALID_STATUS) {
      char buf[10];
      snprintf(buf, sizeof(buf), "%4dmm", lmm);
      Serial.print(buf);
    } else {
      Serial.print("  -- ");
    }
    Serial.println(A_RESET);
  }

  printLegend();

  // ── Center zone summary ───────────────────────────────────────────────────
  int16_t cmm = results.distance_mm[CENTER_ZONE];
  uint8_t cst = results.target_status[CENTER_ZONE];
  Serial.println();
  Serial.print("  Center zone [");
  Serial.print(CENTER_ZONE);
  Serial.print("]:  ");
  Serial.print(zoneColor(cmm, cst));
  Serial.print(A_BOLD);
  if (cst == VALID_STATUS) {
    Serial.print(cmm); Serial.print(" mm");
  } else {
    Serial.print("no target");
  }
  Serial.println(A_RESET "        ");
  Serial.flush();
}
