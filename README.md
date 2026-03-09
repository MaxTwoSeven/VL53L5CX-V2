# VL53L5CX-V2

Arduino C++ firmware and test documentation for dual ST VL53L5CX V2 Time-of-Flight sensors on a Raspberry Pi Pico (Arduino-Pico core). Designed for rider position sensing — outputs serial JSON that feeds directly into Mission Control (`mx-controller`).

---

## Quick Start

**1. Install the Arduino-Pico core**

In Arduino IDE → Preferences → Additional boards manager URLs, add:
```
https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json
```
Then: Boards Manager → search `Raspberry Pi Pico/RP2040` → Install

**2. Install the VL53L5CX library**

Arduino IDE → Library Manager → search `VL53L5CX` → install **"VL53L5CX by STMicroelectronics"**

**3. Select your board**

Tools → Board → Raspberry Pi RP2040 Boards → **Raspberry Pi Pico**

**4. Flash and run Phase 1**

Open `firmware/phase1_single_sensor/phase1_single_sensor.ino`, upload, open Serial Monitor at 115200 baud.

---

## Repo Structure

```
VL53L5CX-V2/
├── firmware/
│   ├── phase1_single_sensor/
│   │   └── phase1_single_sensor.ino   Phase 1: single sensor boot + ranging test
│   ├── phase2_dual_sensor/
│   │   └── phase2_dual_sensor.ino     Phase 2: dual sensor address init + verify
│   └── main/
│       └── main.ino                   Production: 20Hz JSON output over serial
└── docs/
    ├── test_plan.md                   Full test plan with wiring diagram
    ├── mounting_geometry.md           Top-down diagram, cosine correction, pre-mount checklist
    └── phase3_placement_test.md       45° placement guide and pass criteria
```

---

## Mounting Geometry

See **[docs/mounting_geometry.md](docs/mounting_geometry.md)** for the top-down placement diagram, cosine correction math, and pre-mount checklist (rider depth, beam convergence, pull-up resistors).

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

`main.ino` emits one line per frame at ~20Hz:

```json
{"tof":[{"id":"tof-chest","mm":354,"min":100,"max":700},
        {"id":"tof-lean","mm":310,"min":100,"max":700}]}
```

- `mm` values are **cosine-corrected** for the 45° mount angle
- `min`/`max` define the Mission Control UI bar range
- IDs `tof-chest` and `tof-lean` map directly to Mission Control's `inputStore.ts`
- Lines prefixed with `#` are diagnostic logs — Mission Control ignores them

---

## Test Progress

| Phase | Description | Status |
|-------|-------------|--------|
| 1 | Single sensor boot + ranging | ⬜ Pending |
| 2 | Dual sensor address init | ⬜ Pending |
| 3 | 45° placement + cosine correction | ⬜ Pending |
| 4 | Serial JSON → Mission Control | ⬜ Pending |

---

## Hardware

| Part | Notes |
|------|-------|
| Raspberry Pi Pico | Any revision, Arduino-Pico core |
| VL53L5CX V2 breakout ×2 | SparkFun or Pimoroni recommended |
| Sensor spacing | 300mm center-to-center |
| Mount angle | 45° inward toward rider |

---

## Dependencies

- [Arduino-Pico core](https://github.com/earlephilhower/arduino-pico) — RP2040 Arduino support by Earle Philhower
- [stm32duino/VL53L5CX](https://github.com/stm32duino/VL53L5CX) — official ST Arduino library
