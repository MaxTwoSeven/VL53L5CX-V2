/**
 * Sensor B Isolation Test
 * ========================
 * Tests Sensor B ONLY. Sensor A is held in reset (LPN_A LOW) for the
 * entire run — it never touches the I2C bus.
 *
 * Sensor B is NOT given an address change — it stays at the factory
 * default 0x29. This avoids any dependency on a prior address-assign step.
 *
 * Wiring (unchanged from Phase 2/3):
 *   GP2 → LPn A    GP3 → LPn B
 *   GP4 → SDA      GP5 → SCL
 *   GP6 → INT A    GP7 → INT B  (INT not used here — polling fallback)
 *
 * Open Arduino Serial Monitor at 115200 baud.
 * Expected output every ~16 ms:
 *   #1  dist:398mm  zones:7/16  status:OK
 *
 * If you see "FAILED" the sensor is either not powered, LPN_B (GP3) is
 * not connected, or the SDA/SCL lines have a problem.
 */

#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h>

#define LPN_A_PIN    2
#define LPN_B_PIN    3
#define SDA_PIN      4
#define SCL_PIN      5
#define INT_B_PIN    7
#define VALID_STATUS 5
#define NUM_ZONES   16   // 4x4

SparkFun_VL53L5CX    sensor_b;
VL53L5CX_ResultsData results_b;

uint32_t frameCount = 0;

void blink(int n, int onMs, int offMs) {
  for (int i = 0; i < n; i++) {
    digitalWrite(LED_BUILTIN, HIGH); delay(onMs);
    digitalWrite(LED_BUILTIN, LOW);  delay(offMs);
  }
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  blink(3, 100, 100);

  Serial.begin(115200);
  delay(3000);

  Serial.println("========================================");
  Serial.println("  Sensor B Isolation Test  (4x4 @ 60Hz)");
  Serial.println("  Sensor A held in reset — not on bus");
  Serial.println("========================================");
  Serial.flush();

  pinMode(INT_B_PIN, INPUT_PULLUP);
  pinMode(LPN_A_PIN, OUTPUT);
  pinMode(LPN_B_PIN, OUTPUT);

  // A stays LOW the entire time
  digitalWrite(LPN_A_PIN, LOW);
  digitalWrite(LPN_B_PIN, LOW);
  delay(10);

  Wire.setSDA(SDA_PIN);
  Wire.setSCL(SCL_PIN);
  Wire.begin();
  Wire.setClock(400000);

  Serial.print("  Booting Sensor B... "); Serial.flush();
  digitalWrite(LPN_B_PIN, HIGH);
  delay(100);

  Wire.setTimeout(200);
  bool ok = sensor_b.begin(0x29, Wire);
  Wire.setTimeout(0);

  if (!ok) {
    Serial.println("FAILED");
    Serial.println();
    Serial.println("  Checklist:");
    Serial.println("    [ ] LPN_B (GP3) connected to sensor XSHUT/LPN pad?");
    Serial.println("    [ ] 3.3V and GND connected to sensor?");
    Serial.println("    [ ] SDA (GP4) and SCL (GP5) connected?");
    Serial.println("    [ ] Sensor has onboard pull-ups OR external ~4.7k on SDA/SCL?");
    Serial.flush();
    while (true) blink(3, 80, 80);   // rapid triple-blink = error
  }

  // B keeps its factory address 0x29 — no setAddress() call needed
  Serial.println("OK  (address 0x29)");

  sensor_b.setResolution(16);   // 4x4
  sensor_b.setRangingFrequency(60);
  sensor_b.startRanging();

  digitalWrite(LED_BUILTIN, HIGH);   // solid = ranging

  Serial.println();
  Serial.println("  #frame  dist_mm  valid_zones  center_status");
  Serial.println("  ------  -------  -----------  -------------");
  Serial.flush();
}

void loop() {
  bool ready = (digitalRead(INT_B_PIN) == LOW) || sensor_b.isDataReady();
  if (!ready) { delay(5); return; }

  sensor_b.getRangingData(&results_b);
  frameCount++;

  // Count valid zones and compute simple average of valid readings
  int   validCount = 0;
  float sum = 0;
  for (int z = 0; z < NUM_ZONES; z++) {
    if (results_b.target_status[z] == VALID_STATUS) {
      sum += results_b.distance_mm[z];
      validCount++;
    }
  }
  float avg = (validCount > 0) ? sum / validCount : 0;

  // Center zone (zone 5 for 4x4)
  int16_t cDist   = results_b.distance_mm[5];
  uint8_t cStatus = results_b.target_status[5];

  char line[80];
  snprintf(line, sizeof(line),
           "  #%-5lu  %4.0fmm   %2d/16        %s",
           (unsigned long)frameCount,
           avg,
           validCount,
           (cStatus == VALID_STATUS) ? "valid" : "---  ");
  Serial.println(line);
}
