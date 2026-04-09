## 1. Learning Objective

Test whether an engineer can meter a falling ball through a timed rotating gate while respecting the gate swing keep-out and the benchmark-owned power-routing constraint.

## 2. Environment Geometry

- `entry_ramp`: static ramp centered at `[-150, 0, 85]` with size `[180, 120, 50]`.
- `gate_housing`: static frame centered at `[100, 0, 75]` with size `[120, 140, 150]`.
- `gate_pivot_arm`: slender flap centered at `[100, 0, 70]` with size `[18, 120, 90]`; benchmark-side motion is one `rotate_z` axis only.
- `exit_tray`: static tray centered at `[320, 0, 25]` with size `[160, 110, 30]`.

## 3. Input Objective

- Shape: `sphere`
- Label: `projectile_ball`
- Static randomization:
  - radius in `[18, 20]` mm
- Nominal start position: `[-220, 0, 115]`
- Runtime jitter: `[6, 6, 4]` mm

## 4. Objectives

- `goal_zone`: min `[280, -45, 20]`, max `[380, 45, 110]`
- `forbid_zones`:
  - `gate_swing_keepout`: min `[40, -70, 0]`, max `[160, 70, 150]`
- `build_zone`: min `[-260, -140, 0]`, max `[420, 140, 260]`

## 5. Simulation Bounds

- min `[-320, -180, -10]`, max `[480, 180, 320]`

## 6. Constraints Handed To Engineering

- Benchmark/customer caps: `max_unit_cost <= 105 USD`, `max_weight <= 1700 g`
- Benchmark-owned motion must stay limited to one `rotate_z` gate axis with no auxiliary sliders or hidden second motors.
- The declared wiring corridor must avoid the gate swing keep-out volume.

## 7. Success Criteria

- Success if the moved object reaches `goal_zone` without entering `gate_swing_keepout`.
- Fail if the ball exits `simulation_bounds`, clips through undeclared gate geometry, or the benchmark relies on extra motion not declared in `benchmark_assembly_definition.yaml`.

## 8. Planner Artifacts

- `todo.md` tracks the ramp, gate housing, gate arm, and exit tray implementation.
- `benchmark_definition.yaml` mirrors the gate keep-out, power budget, and objective layout.
- `benchmark_assembly_definition.yaml` records the single rotating gate DOF plus benchmark-local costs.
