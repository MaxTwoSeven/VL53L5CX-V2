"""
VL53L5CX-V2 — Production Firmware
===================================
Raspberry Pi Pico running MicroPython.

Reads two VL53L5CX sensors and emits newline-delimited JSON frames
over USB serial at ~20Hz. Mission Control (Electron app) connects
to this serial port and feeds data into the Live Inputs panel.

Wiring:
  Pico 3V3  (Pin 36) → VIN  [Sensor A]  [Sensor B]
  Pico GND  (Pin 38) → GND  [Sensor A]  [Sensor B]
  Pico GP4  (Pin  6) → SDA  [Sensor A]  [Sensor B]  (shared I2C bus)
  Pico GP5  (Pin  7) → SCL  [Sensor A]  [Sensor B]  (shared I2C bus)
  Pico GP2  (Pin  4) → LPn  [Sensor A = chest]
  Pico GP3  (Pin  5) → LPn  [Sensor B = lean]

Output JSON format (matches Mission Control inputStore.ts):
  {"tof":[{"id":"tof-chest","mm":354,"min":100,"max":700},
          {"id":"tof-lean","mm":310,"min":100,"max":700}]}

Each frame is one line terminated with \n.
Distances are cosine-corrected for 45° mount angle.
"""

import json
import math
import time
from machine import I2C, Pin

# ── Constants ──────────────────────────────────────────────────────────────────

SENSOR_A_ID   = "tof-chest"
SENSOR_B_ID   = "tof-lean"

ADDR_DEFAULT  = 0x29   # factory default (7-bit)
ADDR_SENSOR_A = 0x2A   # reassigned at boot
ADDR_SENSOR_B = 0x29   # stays at default

# 4x4 resolution → 16 zones, up to 60Hz. Center zone index in 4x4 grid.
RESOLUTION    = 16
CENTER_ZONE   = 5      # index 5 = upper-center of 4x4 grid

# Cosine correction for 45° mount: slant × cos(45°) = slant × 0.7071
COS_45        = math.cos(math.radians(45))

# Range limits after cosine correction (mm), passed to Mission Control UI
MIN_MM        = 100
MAX_MM        = 700

# Output rate limiter (ms between frames)
FRAME_INTERVAL_MS = 50   # 20Hz

# Status code 5 = valid target in ST ULD driver
VALID_STATUS  = 5

# ── Hardware init ──────────────────────────────────────────────────────────────

i2c   = I2C(0, sda=Pin(4), scl=Pin(5), freq=1_000_000)
lpn_a = Pin(2, Pin.OUT)
lpn_b = Pin(3, Pin.OUT)


def log(msg):
    """Print diagnostic messages to serial (visible in mpremote/screen)."""
    print(f"# {msg}")


# ── Address initialization ─────────────────────────────────────────────────────

def init_sensors():
    """
    Boot sensors sequentially using LPn pins to assign unique I2C addresses.
    Must run on every power cycle — address is not stored on the sensor.
    Returns (sensor_a, sensor_b).
    """
    from vl53l5cx.mp import VL53L5CX

    # Disable both sensors
    lpn_a.value(0)
    lpn_b.value(0)
    time.sleep_ms(10)

    # Boot Sensor A at default 0x29, reassign to 0x2A
    log("Booting Sensor A (chest)...")
    lpn_a.value(1)
    time.sleep_ms(100)   # required boot delay
    sensor_a = VL53L5CX(i2c, addr=ADDR_DEFAULT)
    sensor_a.set_i2c_address(ADDR_SENSOR_A)
    log(f"  Sensor A reassigned to {hex(ADDR_SENSOR_A)}")
    time.sleep_ms(100)

    # Boot Sensor B at default 0x29 (Sensor A is silent — different address)
    log("Booting Sensor B (lean)...")
    lpn_b.value(1)
    time.sleep_ms(100)
    sensor_b = VL53L5CX(i2c, addr=ADDR_DEFAULT)
    log(f"  Sensor B at {hex(ADDR_SENSOR_B)}")
    time.sleep_ms(100)

    # Verify both are on the bus
    found = i2c.scan()
    log(f"I2C scan: {[hex(d) for d in found]}")
    if ADDR_SENSOR_A not in found or ADDR_SENSOR_B not in found:
        raise RuntimeError(
            f"Sensors not found. Expected {hex(ADDR_SENSOR_A)} and "
            f"{hex(ADDR_SENSOR_B)}, got {[hex(d) for d in found]}"
        )

    # Configure resolution and start ranging
    for sensor in (sensor_a, sensor_b):
        sensor.set_resolution(RESOLUTION)
        sensor.start_ranging()

    log("Both sensors ranging.")
    return sensor_a, sensor_b


# ── Distance helpers ───────────────────────────────────────────────────────────

def read_center_mm(sensor):
    """
    Return the corrected distance in mm from the center zone,
    or None if no valid reading is ready.
    """
    if not sensor.check_data_ready():
        return None
    data = sensor.get_ranging_data()
    status = data.target_status[CENTER_ZONE]
    if status != VALID_STATUS:
        return None
    slant_mm = data.distance_mm[CENTER_ZONE]
    return int(slant_mm * COS_45)


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


# ── Main loop ──────────────────────────────────────────────────────────────────

def main():
    log("VL53L5CX-V2 firmware starting...")
    sensor_a, sensor_b = init_sensors()
    log("Streaming JSON to serial. Connect Mission Control.")

    last_mm_a = MIN_MM
    last_mm_b = MIN_MM
    last_frame = time.ticks_ms()

    while True:
        now = time.ticks_ms()

        # Read whatever is ready; keep last known value if sensor not ready yet
        mm_a = read_center_mm(sensor_a)
        mm_b = read_center_mm(sensor_b)

        if mm_a is not None:
            last_mm_a = clamp(mm_a, MIN_MM, MAX_MM)
        if mm_b is not None:
            last_mm_b = clamp(mm_b, MIN_MM, MAX_MM)

        # Emit a frame at the configured interval
        if time.ticks_diff(now, last_frame) >= FRAME_INTERVAL_MS:
            frame = {
                "tof": [
                    {
                        "id":  SENSOR_A_ID,
                        "mm":  last_mm_a,
                        "min": MIN_MM,
                        "max": MAX_MM,
                    },
                    {
                        "id":  SENSOR_B_ID,
                        "mm":  last_mm_b,
                        "min": MIN_MM,
                        "max": MAX_MM,
                    },
                ]
            }
            print(json.dumps(frame))
            last_frame = now

        time.sleep_ms(10)


main()
