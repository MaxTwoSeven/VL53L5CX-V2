# VL53L5CX-V2

Arduino C++ firmware for dual ST VL53L5CX V2 Time-of-Flight sensors on a Raspberry Pi Pico (earlephilhower arduino-pico core). Designed for rider position sensing in a motocross simulator controller — outputs serial JSON that feeds directly into Mission Control (`mx-controller` Electron app).

**Ultimate goal:** merge this firmware with `magduino.ino` (USB HID gamepad) into a unified `main.ino` using the RP2040's dual-core architecture. Core 0 runs the latency-critical HID loop; Core 1 runs the ToF sensors and Serial JSON output.

---

## Quick Start

**1. Install the Arduino-Pico core**

Arduino IDE → Preferences → Additional boards manager URLs, add:
```
https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json
```
Then: Boards Manager → search `Raspberry Pi Pico/RP2040` → Install

> **⚠️ Must use the earlephilhower core** (`architecture: rp2040`), not Arduino Mbed RP2040 (`mbed_rp2040`). `Wire.setSDA()` / `Wire.setSCL()` only exist in earlephilhower.

**2. Install the VL53L5CX library**

Arduino IDE → Library Manager → search `SparkFun VL53L5CX` → install **"SparkFun VL53L5CX Library"**

> **Do not install "VL53L5CX by STMicroelectronics" (STM32duino).** That library only supports STM32/SAM architectures and will fail to compile on RP2040.

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
│   │   └── phase1_single_sensor.ino   ✅  Single sensor boot + ranging
│   ├── phase1_timing/
│   │   └── phase1_timing.ino          ✅  Live Hz + rolling average + ASCII bar
│   ├── phase1_depthmap/
│   │   └── phase1_depthmap.ino        ✅  ANSI color depth map (4x4 or 8x8, single sensor)
│   ├── phase2_dual_sensor/
│   │   └── phase2_dual_sensor.ino     ⚠️  Dual sensor address init + verify (working, not fully validated)
│   ├── phase2_depthmap/
│   │   └── phase2_depthmap.ino        🔧  Side-by-side ANSI depth maps (priority-alternating fix applied, pending upload/test)
│   └── main/
│       └── main.ino                   ⬜  Production: 20Hz JSON output (untested)
└── docs/
    ├── test_plan.md                   Full test plan with wiring diagram
    ├── mounting_geometry.md           Top-down diagram, cosine correction, pre-mount checklist
    └── phase3_placement_test.md       45° placement guide and pass criteria
```

---

## Test Progress

| Phase | Sketch | Description | Status |
|-------|--------|-------------|--------|
| 1 | `phase1_single_sensor` | Single sensor boot + ranging | ✅ Pass |
| 1 | `phase1_timing` | Live Hz / latency monitor | ✅ Pass |
| 1 | `phase1_depthmap` | ANSI color depth map (single sensor) | ✅ Pass |
| 2 | `phase2_dual_sensor` | Dual sensor address init + verify | ⚠️ Working, not fully validated |
| 2 | `phase2_depthmap` | Side-by-side ANSI depth maps (dual) | 🔧 In progress |
| 3 | — | 45° placement + cosine correction | ⬜ Pending |
| 4 | `main` | Serial JSON → Mission Control | ⬜ Pending |

---

## ANSI Depth Map Sketches

`phase1_depthmap` and `phase2_depthmap` use ANSI escape codes for color rendering. The Arduino IDE Serial Monitor **does not** render ANSI. Use macOS Terminal:

```bash
screen /dev/cu.usbmodem<PORT> 115200
```

Quit with `Ctrl-A` then `K`. The port name appears in Arduino IDE → Tools → Port.

4x4 mode (default, 60Hz) is stable for both sensors. 8x8 mode (15Hz) is available via `#define USE_8X8` but requires solid pull-ups on both boards for extended dual-sensor runs.

---

## Prototype Wiring (current breakout boards — pre-PCB)

> ⚠️ These pins **conflict with magduino buttons** (GP2–GP7 are buttons in the production firmware). This wiring is for prototype testing only and must change before integration.

```
Pico 3V3  (Pin 36) ──┬──── VIN  [Sensor A]
                     └──── VIN  [Sensor B]

Pico GND  (Pin 38) ──┬──── GND  [Sensor A]
                     └──── GND  [Sensor B]

Pico GP4  (Pin  6) ──┬──── SDA  [Sensor A]   I2C0 shared bus  ← conflicts with USER4 button
                     └──── SDA  [Sensor B]

Pico GP5  (Pin  7) ──┬──── SCL  [Sensor A]   I2C0 shared bus  ← conflicts with USER3 button
                     └──── SCL  [Sensor B]

Pico GP2  (Pin  4) ──────── LPn [Sensor A]   chest sensor     ← conflicts with RESETMX button
Pico GP3  (Pin  5) ──────── LPn [Sensor B]   lean sensor      ← conflicts with USER5 button

Pico GP6  (Pin  9) ──────── INT [Sensor A]                    ← conflicts with USER2 button
Pico GP7  (Pin 10) ──────── INT [Sensor B]                    ← conflicts with USER1 button
```

**Pull-up note:** Both Alibaba breakout boards have three 4.7kΩ SMD pull-up resistors each (SDA, SCL, INT) soldered directly — no jumper to cut. Combined resistance on the shared bus: ~2.35kΩ. Acceptable at 400kHz; desoldering is not required.

---

## Proposed Production Pin Assignment (post-integration)

These GPIOs are free in `magduino.ino` and will be used once the PCB I2C routes are confirmed:

| Signal | GPIO | Physical Pin | Notes |
|--------|------|-------------|-------|
| SDA (Wire1 / I2C1) | GP14 | 19 | |
| SCL (Wire1 / I2C1) | GP15 | 20 | |
| LPN_A (Sensor A — chest) | GP16 | 21 | |
| LPN_B (Sensor B — lean) | GP17 | 22 | |
| INT_A | GP11 | 15 | |
| INT_B | GP22 | 29 | |

> **PCB note:** The production PCB (`pico_breakout.kicad_sch`) already has a **TCA9406DC** (U4) — a 3.3V↔1.8V bidirectional I2C level translator — wired specifically for the VL53L5CX sensors. Net labels `scl_pi` / `sda_pi` are the Pico side of this translator. The exact Pico GPIO pins for those nets have not yet been traced in KiCad. Final firmware must use `Wire1` (I2C1) on whatever GPIOs `scl_pi`/`sda_pi` resolve to.

**All free GPIOs:** GP1, GP11, GP14, GP15, GP16, GP17, GP22, GP28

---

## magduino.ino — Existing HID Firmware (DO NOT MODIFY)

The production controller also runs `magduino.ino` — a fully working USB HID gamepad firmware on the same Pico. It must not be broken during integration.

| GPIO | Function |
|------|----------|
| GP0 | Timing/debug output |
| GP2 | RESETMX button |
| GP3 | USER5 button |
| GP4 | USER4 button |
| GP5 | USER3 button |
| GP6 | USER2 button |
| GP7 | USER1 button |
| GP8 | S_VIEW button |
| GP9 | LOOK_R button |
| GP10 | LOOK_L button |
| GP12 | SDA — Wire/I2C0 (ADS1115) |
| GP13 | SCL — Wire/I2C0 (ADS1115) |
| GP18 | SPI SCK (TMAG5170) |
| GP19 | SPI TX (TMAG5170) |
| GP20 | SPI RX (TMAG5170) |
| GP21 | TMAG5170 CS |
| GP25 | LED (built-in) |
| GP26 | STAND button |
| GP27 | BACKUP button |

Peripherals: TMAG5170 3D Hall effect sensor (SPI), two ADS1115 16-bit ADCs (I2C0 @ GP12/GP13), 11 digital buttons. Output: USB HID gamepad (8 axes, 16 buttons) via Adafruit TinyUSB at 5ms poll rate.

---

## Planned Dual-Core Architecture (main.ino — not yet implemented)

```
Core 0 (setup / loop)       — HID loop (magduino)
  TMAG5170 via SPI
  2× ADS1115 via Wire/I2C0 (GP12/GP13)
  11 buttons
  USB HID gamepad report @ 5ms

Core 1 (setup1 / loop1)     — ToF + Serial JSON
  2× VL53L5CX via Wire1/I2C1 (GP14/GP15 TBD)
  Serial JSON @ ~20Hz
```

Uses earlephilhower dual-core API: `setup1()` / `loop1()` run on Core 1 automatically.

---

## Planned HID Axis Mapping

| Source | Active when | HID axis | Meaning |
|--------|------------|----------|---------|
| TMAG X | Always | `gp.x` | Bike lean |
| VL53L5CX chest distance | ToF toggle ON | `gp.y` | Rider forward/back |
| TMAG Y | ToF toggle OFF | `gp.y` | Rider forward/back (fallback) |
| Thumbstick X | ToF toggle OFF | new axis | Rider lean L/R |
| Thumbstick Y | ToF toggle OFF | new axis | Rider F/B |

A physical GPIO toggle switch selects between ToF mode and thumbstick mode for the `gp.y` axis.

---

## JSON Output Format

`main.ino` will emit one newline-delimited JSON frame per cycle at ~20Hz:

```json
{"tof":[{"id":"tof-chest","mm":354,"min":100,"max":700},
        {"id":"tof-lean","mm":310,"min":100,"max":700}]}
```

- `mm` values are **cosine-corrected** for the 45° mount angle (`true_mm = slant_mm × cos(45°) × 0.7071`)
- `min` / `max` define the Mission Control UI bar range
- IDs `tof-chest` and `tof-lean` map directly to Mission Control's `inputStore.ts`
- Lines prefixed with `#` are diagnostic logs — Mission Control ignores them

---

## Lessons Learned

| Topic | Decision |
|-------|----------|
| Library | Use **SparkFun VL53L5CX Library** — STM32duino has no RP2040 platform layer |
| Board core | Must use **earlephilhower arduino-pico** — `Wire.setSDA/SCL()` only exist there |
| Serial init | **Never use `while (!Serial)`** on RP2040 — causes DTR reset loop. Use `delay(3000)` |
| ANSI output | Arduino IDE Serial Monitor does not render ANSI — use `screen /dev/cu.usbmodem<PORT> 115200` |
| I2C speed | **400kHz** is the stable speed for dual sensor. 1MHz caused instability; 100kHz is too slow for 8x8 at 60Hz |
| Breadboards | **Do not use.** Bad breadboard contacts caused all early sensor-not-found failures. Direct wire only |
| INT pins | Connected but SparkFun library does not assert INT by default — always use `isDataReady()` as fallback |
| Dual-core | Final firmware uses `setup1()` / `loop1()` — earlephilhower RP2040 dual-core API |
| PCB I2C | Production PCB has TCA9406 level translator for VL53L5CX — final firmware must use `Wire1` on PCB-correct GPIOs |

---

## Immediate Next Steps

1. **Upload and test `phase2_depthmap.ino`** — confirm the priority-alternating fix resolves Sensor B starvation (view in `screen` terminal, not Arduino IDE)
2. **Open `pico_breakout.kicad_sch` in KiCad** — trace `scl_pi` and `sda_pi` net labels back to specific Pico GPIOs; also check if `LPN_A`, `LPN_B`, `INT_A`, `INT_B` are routed on the PCB
3. **Write unified `main.ino`** — merge magduino + VL53L5CX using `setup1()` / `loop1()` on Wire1 with PCB-confirmed pins
4. **Phase 3** — 45° placement + cosine correction validation (`docs/phase3_placement_test.md`)
5. **Phase 4** — Serial JSON → Mission Control live input bars

---

## Dependencies

- [Arduino-Pico core](https://github.com/earlephilhower/arduino-pico) — RP2040 Arduino support by Earle Philhower
- [SparkFun VL53L5CX Library](https://github.com/sparkfun/SparkFun_VL53L5CX_Arduino_Library) — RP2040-compatible Arduino wrapper for the ST ULD
