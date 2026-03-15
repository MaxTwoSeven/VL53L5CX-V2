/**
 * VL53L5CX-V2 — Production Firmware
 * ====================================
 * Target: Raspberry Pi Pico (Arduino-Pico core — earlephilhower)
 * Library: SparkFun VL53L5CX Library
 *
 * Reads two VL53L5CX sensors and emits newline-delimited JSON frames
 * over USB serial at ~20Hz. Mission Control (Electron app) connects
 * to this serial port and feeds data into the Live Inputs panel.
 *
 * Wiring:
 *   Pico 3V3  (Pin 36) → VIN  [Sensor A]  [Sensor B]
 *   Pico GND  (Pin 38) → GND  [Sensor A]  [Sensor B]
 *   Pico GP4  (Pin  6) → SDA  [Sensor A]  [Sensor B]  (shared I2C bus)
 *   Pico GP5  (Pin  7) → SCL  [Sensor A]  [Sensor B]  (shared I2C bus)
 *   Pico GP2  (Pin  4) → LPn  [Sensor A = chest]
 *   Pico GP3  (Pin  5) → LPn  [Sensor B = lean]
 *
 * Output JSON format (matches Mission Control inputStore.ts):
 *   {"tof":[{"id":"tof-chest","mm":354,"min":100,"max":700},
 *           {"id":"tof-lean","mm":310,"min":100,"max":700}]}
 *
 * Each frame is one line terminated with \n at ~20Hz.
 * Distances are cosine-corrected for 45° mount angle.
 */

#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h>
#include <math.h>

// ── Pin assignments ───────────────────────────────────────────────────────────
#define LPN_A_PIN   2
#define LPN_B_PIN   3
#define SDA_PIN     4
#define SCL_PIN     5

// ── Ranging config ────────────────────────────────────────────────────────────
#define CENTER_ZONE       5      // Upper-center zone in 4x4 grid
#define VALID_STATUS      5      // ST ULD driver: 5 = valid target
#define FRAME_INTERVAL_MS 50     // 20Hz output

// ── Range limits (mm) after cosine correction ─────────────────────────────────
#define MIN_MM 100
#define MAX_MM 700

// ── Cosine correction for 45° mount ──────────────────────────────────────────
const float COS_45 = cos(45.0 * M_PI / 180.0);  // 0.7071

SparkFun_VL53L5CX sensor_a;
SparkFun_VL53L5CX sensor_b;
VL53L5CX_ResultsData results_a;
VL53L5CX_ResultsData results_b;

int last_mm_a = MIN_MM;
int last_mm_b = MIN_MM;
unsigned long last_frame = 0;

int clamp(int value, int lo, int hi) {
  return max(lo, min(hi, value));
}

void initSensors() {
  pinMode(LPN_A_PIN, OUTPUT);
  pinMode(LPN_B_PIN, OUTPUT);

  Wire.setSDA(SDA_PIN);
  Wire.setSCL(SCL_PIN);
  Wire.begin();
  Wire.setClock(400000);

  Serial.println("# Disabling both sensors...");
  digitalWrite(LPN_A_PIN, LOW);
  digitalWrite(LPN_B_PIN, LOW);
  delay(10);

  Serial.println("# Booting Sensor A (chest)...");
  digitalWrite(LPN_A_PIN, HIGH);
  delay(100);

  if (!sensor_a.begin(0x29, Wire)) {
    Serial.println("# ERROR: Sensor A not found.");
    while (true) delay(1000);
  }
  sensor_a.setAddress(0x2A);
  Serial.println("# Sensor A reassigned to 0x2A");
  delay(100);

  Serial.println("# Booting Sensor B (lean)...");
  digitalWrite(LPN_B_PIN, HIGH);
  delay(100);

  if (!sensor_b.begin(0x29, Wire)) {
    Serial.println("# ERROR: Sensor B not found.");
    while (true) delay(1000);
  }
  Serial.println("# Sensor B at 0x29");
  delay(100);

  sensor_a.setResolution(4 * 4);
  sensor_b.setResolution(4 * 4);
  sensor_a.startRanging();
  sensor_b.startRanging();

  Serial.println("# Both sensors ranging. Streaming JSON...");
}

void setup() {
  Serial.begin(115200);
  delay(3000);  // no while(!Serial) — causes DTR reset loop on RP2040
  Serial.println("# VL53L5CX-V2 firmware starting...");
  initSensors();
  last_frame = millis();
}

void loop() {
  if (sensor_a.isDataReady()) {
    sensor_a.getRangingData(&results_a);
    uint8_t st = results_a.target_status[CENTER_ZONE];
    if (st == VALID_STATUS) {
      int corrected = (int)(results_a.distance_mm[CENTER_ZONE] * COS_45);
      last_mm_a = clamp(corrected, MIN_MM, MAX_MM);
    }
  }

  if (sensor_b.isDataReady()) {
    sensor_b.getRangingData(&results_b);
    uint8_t st = results_b.target_status[CENTER_ZONE];
    if (st == VALID_STATUS) {
      int corrected = (int)(results_b.distance_mm[CENTER_ZONE] * COS_45);
      last_mm_b = clamp(corrected, MIN_MM, MAX_MM);
    }
  }

  if (millis() - last_frame >= FRAME_INTERVAL_MS) {
    Serial.print("{\"tof\":[");
    Serial.print("{\"id\":\"tof-chest\",\"mm\":");
    Serial.print(last_mm_a);
    Serial.print(",\"min\":");
    Serial.print(MIN_MM);
    Serial.print(",\"max\":");
    Serial.print(MAX_MM);
    Serial.print("},{\"id\":\"tof-lean\",\"mm\":");
    Serial.print(last_mm_b);
    Serial.print(",\"min\":");
    Serial.print(MIN_MM);
    Serial.print(",\"max\":");
    Serial.print(MAX_MM);
    Serial.println("}]}");
    last_frame = millis();
  }
}
