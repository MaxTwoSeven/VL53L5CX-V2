# VL53L5CX-V2

MicroPython firmware and test documentation for dual ST VL53L5CX V2 Time-of-Flight sensors on a Raspberry Pi Pico. Designed for rider position sensing — outputs serial JSON that feeds directly into Mission Control (`mx-controller`).

---

## Quick Start

```bash
# 1. Install MicroPython on the Pico
#    Hold BOOTSEL, plug in USB, drag .uf2 to RPI-RP2 drive
#    https://micropython.org/download/rp2-pico/

# 2. Install mpremote
pip3 install mpremote

# 3. Install the VL53L5CX MicroPython library
#    Clone https://github.com/mp-extras/vl53l5cx first, then:
mpremote mkdir :lib/vl53l5cx
mpremote cp vl53l5cx/__init__.py  :lib/vl53l5cx/__init__.py
mpremote cp vl53l5cx/mp.py        :lib/vl53l5cx/mp.py
mpremote cp vl53l5cx/_config_file.py :lib/vl53l5cx/_config_file.py
mpremote cp vl53l5cx/vl_fw_config.bin :lib/vl53l5cx/vl_fw_config.bin

# 4. Run Phase 1 test (single sensor)
mpremote run firmware/phase1_single_sensor.py

# 5. Deploy production firmware
mpremote cp firmware/main.py :main.py
```

---

## Repo Structure

```
VL53L5CX-V2/
├── firmware/
│   ├── phase1_single_sensor.py   Phase 1: single sensor boot + ranging test
│   ├── phase2_dual_sensor.py     Phase 2: dual sensor address init + verify
│   └── main.py                   Production: 20Hz JSON output over serial
└── docs/
    ├── test_plan.md              Full test plan with wiring diagram
    └── phase3_placement_test.md  45° placement guide and pass criteria
```

---

## Wiring

```
Pico 3V3  (Pin 36) ──┬──── VIN  [Sensor A]
                     └──── VIN  [Sensor B]

Pico GND  (Pin 38) ──┬──── GND  [Sensor A]
                     └──── GND  [Sensor B]

Pico GP4  (Pin  6) ──┬──── SDA  [Sensor A]   I2C0 shared bus
                     └──── SDA  [Sensor B]

Pico GP5  (Pin  7) ──┬──── SCL  [Sensor A]   I2C0 shared bus
                     └──── SCL  [Sensor B]

Pico GP2  (Pin  4) ──────── LPn [Sensor A]   chest sensor
Pico GP3  (Pin  5) ──────── LPn [Sensor B]   lean sensor
```

> **Pull-up note:** Most breakout boards have onboard pull-ups. If both boards have them active, cut the jumper on one so the combined bus resistance stays near 4.7kΩ.

---

## JSON Output Format

`main.py` emits one line per frame at ~20Hz:

```json
{"tof":[{"id":"tof-chest","mm":354,"min":100,"max":700},
        {"id":"tof-lean","mm":310,"min":100,"max":700}]}
```

- `mm` values are **cosine-corrected** for the 45° mount angle
- `min`/`max` define the Mission Control UI bar range
- IDs `tof-chest` and `tof-lean` map directly to Mission Control's `inputStore.ts`

---

## Test Progress

| Phase | Description | Status |
|-------|-------------|--------|
| 1 | Single sensor boot + ranging | ✅ Complete |
| 2 | Dual sensor address init | ✅ Complete |
| 3 | 45° placement + cosine correction | ✅ Complete |
| 4 | Serial JSON → Mission Control | ✅ Complete |

---

## Hardware

| Part | Notes |
|------|-------|
| Raspberry Pi Pico | Any revision, stock MicroPython firmware |
| VL53L5CX V2 breakout ×2 | SparkFun or Pimoroni recommended |
| Sensor spacing | 500mm center-to-center |
| Mount angle | 45° inward toward rider |

---

## Dependencies

- [MicroPython for Pico](https://micropython.org/download/rp2-pico/)
- [mp-extras/vl53l5cx](https://github.com/mp-extras/vl53l5cx) — pure MicroPython ULD driver
- [mpremote](https://pypi.org/project/mpremote/) — for flashing scripts to the Pico
