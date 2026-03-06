"""
Phase 1 — Single VL53L5CX Sensor Verification
==============================================
Wiring:
  Pico 3V3  (Pin 36) → VIN  [Sensor A]
  Pico GND  (Pin 38) → GND  [Sensor A]
  Pico GP4  (Pin  6) → SDA  [Sensor A]
  Pico GP5  (Pin  7) → SCL  [Sensor A]
  Pico GP2  (Pin  4) → LPn  [Sensor A]

Expected output: continuous distance readings from center zone in mm.
Move your hand toward/away from the sensor to verify tracking.

Library required: mp-extras/vl53l5cx
  https://github.com/mp-extras/vl53l5cx
  Install with:
    mpremote mkdir :lib/vl53l5cx
    mpremote cp vl53l5cx/__init__.py  :lib/vl53l5cx/__init__.py
    mpremote cp vl53l5cx/mp.py        :lib/vl53l5cx/mp.py
    mpremote cp vl53l5cx/_config_file.py :lib/vl53l5cx/_config_file.py
    mpremote cp vl53l5cx/vl_fw_config.bin :lib/vl53l5cx/vl_fw_config.bin
"""

import time
from machine import I2C, Pin

# ----- Hardware setup -----
# 1 MHz for fast sensor firmware upload (~86 KB over I2C at boot)
i2c = I2C(0, sda=Pin(4), scl=Pin(5), freq=1_000_000)

lpn_a = Pin(2, Pin.OUT)

print("Enabling Sensor A...")
lpn_a.value(1)
time.sleep_ms(100)  # Required boot delay after LPn goes HIGH

# ----- Scan bus -----
devices = i2c.scan()
print("I2C devices found:", [hex(d) for d in devices])
# Expected: [0x29]  (decimal 41)

if 0x29 not in devices:
    print("ERROR: Sensor A not found on bus. Check wiring and power.")
    raise SystemExit

# ----- Initialize sensor -----
from vl53l5cx.mp import VL53L5CX

print("Initializing sensor (uploading firmware ~86KB, wait ~2s)...")
sensor = VL53L5CX(i2c)

# 4x4 = 16 zones. Simpler and faster (up to 60Hz) for initial testing.
sensor.set_resolution(16)
sensor.start_ranging()

print("Sensor A ready. Readings:")
print("  Zone layout: 4x4 grid. Index [5] = upper-center zone.")
print("-" * 40)

while True:
    if sensor.check_data_ready():
        data = sensor.get_ranging_data()

        # Center zone of 4x4 grid (indices 5 or 6 are center-ish)
        center_mm = data.distance_mm[5]
        status    = data.target_status[5]

        # Status 5 = valid range. Other values indicate noise/no target.
        validity = "OK" if status == 5 else f"status={status}"
        print(f"  Distance: {center_mm:4d} mm  [{validity}]")

    time.sleep_ms(50)
