## 1. Learning Objective

Test whether benchmark-owned motorized lift hardware can raise a ball from floor height onto a shelf while keeping the shelf-support keep-out clear and preserving the class-aware COTS import path. The engineer must reason about powered fixture motion, COTS motor torque limits, and clearance geometry.

## 2. Geometry

All dimensions in mm. Positions are world-frame centers unless noted.

- `floor_plate`: fixed base plate centered near `(20, 0, 10)`, size `560 × 180 × 20`.
- `support_tower`: vertical tower centered near `(230, 0, 105)`, size `80 × 120 × 210`.
- `raised_goal_shelf`: raised target shelf centered near `(375, 0, 235)`, size `150 × 120 × 30`.
- `lift_carriage`: moving plate centered near `(-130, 0, 60)`, size `120 × 110 × 18`, single `slide_z` axis.
- `drive_motor`: benchmark-owned servo fixture imported through `ServoMotor.from_catalog_id("ServoMotor_DS3218")`, mounting center `(-130, 0, 10)`, catalog size `40 × 20 × 40.5` mm.
- `projectile_ball`: moved object, sphere, ABS plastic, radius `[44, 46]` mm, spawn position `(-200, 0, 60)`, runtime jitter `±[10, 8, 6]` mm.

## 3. Objectives

- **Goal zone**: AABB min `(320, -55, 220)`, max `(430, 55, 300)`. Success when the projectile ball center enters this volume.
- **Forbid zone** `shelf_support_clearance`: AABB min `(180, -95, 0)`, max `(280, 95, 210)`. Any contact = failure.
- **Build zone**: AABB min `(-260, -180, 0)`, max `(470, 180, 340)`.
- **Simulation bounds**: AABB min `(-320, -220, -10)`, max `(520, 220, 380)`.

## 4. Randomization

- Static: projectile ball radius varies in `[44, 46]` mm per benchmark variant.
- Runtime: projectile ball spawn position jitters by `±[10, 8, 6]` mm per simulation run.
- The solution must handle all positions within the jitter range robustly.

## 5. Implementation Notes

- Use build123d primitives for all geometry.
- Instantiate the drive motor via `ServoMotor.from_catalog_id("ServoMotor_DS3218")`.
- Preserve `benchmark_plan_evidence_script.py` and `benchmark_plan_technical_drawing_script.py` as read-only planner context.
- Keep `benchmark_script.py` import-safe: no `__main__` block, no in-module review submission call.
- Run validate/simulate/review submission only from external shell self-check commands.
- Benchmark-owned motion: `lift_carriage` has a single `slide_z` DOF, motorized, travel range z ≈ 10 → 220 mm, control mode `ON_OFF` at speed 1.0. The `drive_motor` is fixed and not engineer-interactable.
- Electronics: 24 V DC supply, max 10 A, max wire length 850 mm, wire restricted from the shelf-support clearance zone, circuit validation required.
- Planner estimate: `$63.33` / `1200 g`. Runtime-derived caps: `$95.00` / `1800 g`.
