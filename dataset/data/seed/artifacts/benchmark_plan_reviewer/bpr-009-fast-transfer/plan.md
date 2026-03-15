## 1. Learning Objective

Test whether an engineer can handle a fast lateral transfer benchmark that includes a single driven roller and a central speed-bump keep-out without turning the benchmark into an over-actuated machine.

## 2. Environment Geometry

- `launch_lane`: static lane centered at `[-120, 0, 55]` with size `[620, 110, 40]`.
- `accelerator_roller`: cylindrical roller centered at `[0, 0, 45]` with diameter `50` mm and length `90` mm; benchmark-side motion is one `rotate_x` axis only.
- `speed_bump_guard`: static ridge centered at `[105, 0, 27]` with size `[90, 140, 55]`.
- `goal_brake_tray`: static tray centered at `[415, 0, 28]` with size `[150, 120, 32]`.

## 3. Input Objective

- Shape: `sphere`
- Label: `projectile_ball`
- Static randomization:
  - radius in `[24, 26]` mm
- Nominal start position: `[-400, 0, 105]`
- Runtime jitter: `[8, 6, 4]` mm

## 4. Objectives

- `goal_zone`: min `[360, -55, 15]`, max `[470, 55, 100]`
- `forbid_zones`:
  - `center_speed_bump`: min `[60, -70, 0]`, max `[150, 70, 55]`
- `build_zone`: min `[-460, -150, 0]`, max `[520, 150, 220]`

## 5. Simulation Bounds

- min `[-520, -190, -10]`, max `[580, 190, 260]`

## 6. Constraints Handed To Engineering

- Benchmark/customer caps: `max_unit_cost <= 90 USD`, `max_weight <= 1600 g`
- Benchmark-owned motion must stay limited to one `rotate_x` driven roller; the benchmark should not imply extra powered pushers, gates, or lifts.
- Cable routing must remain outside the `center_speed_bump` keep-out.

## 7. Success Criteria

- Success if the ball reaches `goal_zone` while clearing the `center_speed_bump` forbid volume.
- Fail if the moved object exits `simulation_bounds` or if the benchmark requires undeclared motion beyond the single driven roller.

## 8. Planner Artifacts

- `todo.md` tracks the lane, driven roller, speed-bump guard, and goal tray implementation.
- `benchmark_definition.yaml` mirrors the speed-oriented geometry and benchmark caps.
- `benchmark_assembly_definition.yaml` records the single roller DOF and benchmark-local costs.
