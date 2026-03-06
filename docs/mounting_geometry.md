# Sensor Mounting Geometry

## Top-Down View (Bird's Eye)

```
        ┌─────────────── 300mm ───────────────┐
        │                                     │
   [Sensor A]                            [Sensor B]
    chest ↘  45°                    45°  ↙ lean
            ↘                          ↙
             ↘  424mm (slant)    424mm ↙
              ↘                      ↙
               ↘                    ↙
                ↘                  ↙
                 →   [Rider]   ←
                     ~300mm
                  from each sensor
                  (perpendicular)
```

- The **baseplate** is 300mm wide (sensor A to sensor B, center-to-center)
- Both sensors angle **inward at 45°**, converging on the rider's torso
- The rider sits roughly **300mm away** from the sensor plane (perpendicular depth)
- The **laser beam travels diagonally** — that slant path is what the sensor hardware measures
- `main.py` applies cosine correction to convert slant → true perpendicular distance

| Measurement | What it is |
|-------------|------------|
| 300mm (horizontal) | Baseplate width — Sensor A to Sensor B |
| 300mm (depth) | Rider's perpendicular distance from sensor plane |
| 424mm | Diagonal laser path the sensor actually measures |

---

## Cosine Correction

The sensor reports **slant distance**, not perpendicular distance.

```
perpendicular = slant × cos(45°) = slant × 0.7071
slant         = perpendicular / cos(45°) = perpendicular / 0.7071
```

At the expected rider depth of 300mm:

```
slant = 300 / 0.7071 = 424mm  ← what the sensor hardware reports
main.py outputs "mm": 300     ← after correction
```

---

## Pre-Mount Checklist

### 1. Confirm Rider Depth
Measure the actual distance from the baseplate to where the rider's torso sits.
This determines the real-world `MIN_MM` / `MAX_MM` range in `firmware/main.py`.
Current assumption: **100–700mm**. Adjust after physical test.

### 2. Check Beam Convergence Point
With both sensors at 45° inward, their beams cross at a point in front of the plate.
Verify the crossover lands at the rider's torso depth — not behind them or too close to
the baseplate. If the rider sits too close, one or both sensors may read past center.

### 3. Pull-Up Resistor Check
Most breakout boards (SparkFun, Pimoroni) have onboard pull-up resistors on SDA/SCL.
**If both boards have pull-ups active simultaneously**, the combined bus resistance will
be too low (~2.2kΩ instead of ~4.7kΩ), causing I2C instability.

- Inspect both boards before wiring Phase 2
- If both have pull-ups, cut the jumper on one board
- Target combined SDA/SCL pull-up: **~4.7kΩ**
