# Phase 3 — 45° Placement Test Guide

## Setup

Place both sensors on the test board with the following geometry:

```
         [Sensor A]                [Sensor B]
           45° ↘                    45° ↙
               ↘                  ↙
                ↘                ↙
         ←——————  300mm  ————————→
                      ↓
                  [Rider]
```

- Sensor A = **chest** — positioned to read lean left/right torso position
- Sensor B = **lean** — positioned to read forward/back body lean
- Both angled **45° inward** toward the seated rider
- **300mm separation** between sensors (center to center)

---

## Slant vs. Perpendicular Distance

The VL53L5CX reports **slant distance** (along the sensor's optical axis),
not the true perpendicular distance to the rider.

At a 45° mount angle:

```
true_perpendicular_mm = reported_slant_mm × cos(45°)
                      = reported_slant_mm × 0.7071
```

| Slant (reported) | True Perpendicular |
|------------------|--------------------|
| 400 mm           | 283 mm             |
| 500 mm           | 354 mm             |
| 600 mm           | 424 mm             |
| 700 mm           | 495 mm             |
| 800 mm           | 566 mm             |

The `main.py` firmware applies this correction automatically before emitting JSON.

---

## Baseline Test (no rider)

Run `phase2_dual_sensor.py` with sensors in final position.

Expected readings with no rider present:
- Both sensors should report slant distances consistent with the wall/background (~600–1000mm depending on room depth)
- Readings should be stable (±10mm noise is normal)

---

## Rider Position Test

With rider seated in normal position:

| Body Position       | Sensor A (chest) | Sensor B (lean) |
|---------------------|------------------|-----------------|
| Neutral upright     | ~400–550mm slant | ~400–550mm slant |
| Lean forward        | decreases        | decreases        |
| Lean back           | increases        | increases        |
| Lean left           | decreases (A closer) | increases    |
| Lean right          | increases        | decreases (B closer) |

Verify each movement produces the expected directional change.

---

## Pass Criteria

- [ ] Phase 2 confirmed (both sensors on bus, no collision)
- [ ] Baseline readings stable with no rider (noise < ±20mm)
- [ ] Neutral rider position reads 300–600mm on both sensors (slant)
- [ ] Forward lean decreases both readings proportionally
- [ ] Left/right lean creates asymmetry between sensors A and B
- [ ] No freeze or timeout during continuous 5-minute run

---

## Cosine Correction Validation

To verify the correction is working correctly after flashing `main.py`:

1. Place a flat board exactly **300mm** (perpendicular) in front of a sensor
2. The sensor is mounted at 45°, so slant distance = `300 / cos(45°)` = **424mm**
3. `main.py` should output `"mm": 300` (after correction)
4. Confirm this matches the measured physical distance

---

## Notes on 8×8 vs 4×4 Mode

`main.py` uses **4×4 mode (16 zones)** for up to 60Hz output rate.

If you want a more detailed depth map (e.g., detecting which part of the
body is closest), switch to `sensor.set_resolution(64)` for 8×8 mode.
Frame rate drops to ~15Hz but you get 64 individual zone distances.

For controller input purposes, the single center zone or the average of
the 4 center zones is sufficient.
