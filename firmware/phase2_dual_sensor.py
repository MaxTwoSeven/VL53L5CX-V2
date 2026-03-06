"""
Phase 2 — Dual VL53L5CX Sensor Verification
=============================================
Wiring (adds Sensor B to Phase 1 setup):
  Pico 3V3  (Pin 36) → VIN  [Sensor A]  [Sensor B]
  Pico GND  (Pin 38) → GND  [Sensor A]  [Sensor B]
  Pico GP4  (Pin  6) → SDA  [Sensor A]  [Sensor B]  (shared bus)
  Pico GP5  (Pin  7) → SCL  [Sensor A]  [Sensor B]  (shared bus)
  Pico GP2  (Pin  4) → LPn  [Sensor A]
  Pico GP3  (Pin  5) → LPn  [Sensor B]

Pull-up note: if both breakout boards have onboard pull-up resistors,
cut the jumper on one board to keep combined pull-up ~4.7kΩ on SDA/SCL.

Address init result:
  Sensor A → 0x2A  (chest, reassigned at boot)
  Sensor B → 0x29  (lean, default)

Expected output: interleaved distance readings from both sensors.
"""

import time
from machine import I2C, Pin

# ----- Hardware setup -----
i2c = I2C(0, sda=Pin(4), scl=Pin(5), freq=1_000_000)

lpn_a = Pin(2, Pin.OUT)
lpn_b = Pin(3, Pin.OUT)

# ----- Address initialization sequence -----
# Step 1: Disable both sensors
print("Disabling both sensors...")
lpn_a.value(0)
lpn_b.value(0)
time.sleep_ms(10)

# Step 2: Boot only Sensor A (at default 0x29), reassign to 0x2A
print("Booting Sensor A...")
lpn_a.value(1)
time.sleep_ms(100)  # REQUIRED: sensor needs 100ms to boot

from vl53l5cx.mp import VL53L5CX

print("  Uploading firmware to Sensor A...")
sensor_a = VL53L5CX(i2c, addr=0x29)
sensor_a.set_i2c_address(0x2A)
print("  Sensor A reassigned to 0x2A")
time.sleep_ms(100)

# Step 3: Boot Sensor B (stays at default 0x29)
print("Booting Sensor B...")
lpn_b.value(1)
time.sleep_ms(100)

print("  Uploading firmware to Sensor B...")
sensor_b = VL53L5CX(i2c, addr=0x29)
print("  Sensor B at 0x29")
time.sleep_ms(100)

# Step 4: Sensor A is already enabled from step 2 — nothing to do.
# Both sensors are now live.

# ----- Verify bus -----
devices = i2c.scan()
print("I2C scan:", [hex(d) for d in devices])
# Expected: [0x29, 0x2A] (order may vary)

if 0x29 not in devices or 0x2A not in devices:
    print("ERROR: Expected both 0x29 and 0x2A on bus.")
    print("  Found:", [hex(d) for d in devices])
    raise SystemExit

print("Both sensors confirmed on bus.")

# ----- Configure and start ranging -----
sensor_a.set_resolution(16)  # 4x4
sensor_b.set_resolution(16)
sensor_a.start_ranging()
sensor_b.start_ranging()

print("Reading from both sensors:")
print("-" * 50)

while True:
    a_ready = sensor_a.check_data_ready()
    b_ready = sensor_b.check_data_ready()

    if a_ready:
        data_a = sensor_a.get_ranging_data()
        mm_a   = data_a.distance_mm[5]
        st_a   = data_a.target_status[5]
        print(f"  A (chest, 0x2A): {mm_a:4d} mm  [{'OK' if st_a == 5 else f'status={st_a}'}]")

    if b_ready:
        data_b = sensor_b.get_ranging_data()
        mm_b   = data_b.distance_mm[5]
        st_b   = data_b.target_status[5]
        print(f"  B (lean,  0x29): {mm_b:4d} mm  [{'OK' if st_b == 5 else f'status={st_b}'}]")

    time.sleep_ms(50)
