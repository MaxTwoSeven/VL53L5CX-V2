/**
 * Sensor A Isolation Test
 * ========================
 * Tests Sensor A ONLY. Sensor B is held in reset (LPN_B LOW) for the
 * entire run — it never touches the I2C bus.
 *
 * Wiring (unchanged from Phase 2/3):
 *   GP2 → LPn A    GP3 → LPn B
 *   GP4 → SDA      GP5 → SCL
 *   GP6 → INT A    GP7 → INT B  (INT not used here — polling fallback)
 *
 * Open Arduino Serial Monitor at 115200 baud.
 * Expected output every ~16 ms:
 *   #1  dist:412mm  zones:9/16  status:OK
 */

#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h>

#define LPN_A_PIN    2
#define LPN_B_PIN    3
#define SDA_PIN      4
#define SCL_PIN      5
#define INT_A_PIN    6
#define VALID_STATUS 5
#define NUM_ZONES   16   // 4x4

SparkFun_VL53L5CX    sensor_a;
VL53L5CX_ResultsData results_a;

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
  Serial.println("  Sensor A Isolation Test  (4x4 @ 60Hz)");
  Serial.println("  Sensor B held in reset — not on bus");
  Serial.println("========================================");
  Serial.flush();

  pinMode(INT_A_PIN, INPUT_PULLUP);
  pinMode(LPN_A_PIN, OUTPUT);
  pinMode(LPN_B_PIN, OUTPUT);

  // B stays LOW the entire time
  digitalWrite(LPN_A_PIN, LOW);
  digitalWrite(LPN_B_PIN, LOW);
  delay(10);

  Wire.setSDA(SDA_PIN);
  Wire.setSCL(SCL_PIN);
  Wire.begin();
  Wire.setClock(400000);

  Serial.print("  Booting Sensor A... "); Serial.flush();
  digitalWrite(LPN_A_PIN, HIGH);
  delay(100);

  Wire.setTimeout(200);
  bool ok = sensor_a.begin(0x29, Wire);
  Wire.setTimeout(0);

  if (!ok) {
    Serial.println("FAILED — check wiring on A (LPN_A=GP2, SDA=GP4, SCL=GP5)");
    Serial.flush();
    while (true) blink(3, 80, 80);   // rapid triple-blink = error
  }

  sensor_a.setAddress(0x2A);
  Serial.println("OK  (address set to 0x2A)");

  sensor_a.setResolution(16);   // 4x4
  sensor_a.setRangingFrequency(60);
  sensor_a.startRanging();

  digitalWrite(LED_BUILTIN, HIGH);   // solid = ranging

  Serial.println();
  Serial.println("  #frame  dist_mm  valid_zones  center_status");
  Serial.println("  ------  -------  -----------  -------------");
  Serial.flush();
}

void loop() {
  bool ready = (digitalRead(INT_A_PIN) == LOW) || sensor_a.isDataReady();
  if (!ready) { delay(5); return; }

  sensor_a.getRangingData(&results_a);
  frameCount++;

  // Count valid zones and compute simple average of valid readings
  int   validCount = 0;
  float sum = 0;
  for (int z = 0; z < NUM_ZONES; z++) {
    if (results_a.target_status[z] == VALID_STATUS) {
      sum += results_a.distance_mm[z];
      validCount++;
    }
  }
  float avg = (validCount > 0) ? sum / validCount : 0;

  // Center zone (zone 5 for 4x4)
  int16_t cDist   = results_a.distance_mm[5];
  uint8_t cStatus = results_a.target_status[5];

  char line[80];
  snprintf(line, sizeof(line),
           "  #%-5lu  %4.0fmm   %2d/16        %s",
           (unsigned long)frameCount,
           avg,
           validCount,
           (cStatus == VALID_STATUS) ? "valid" : "---  ");
  Serial.println(line);
}
