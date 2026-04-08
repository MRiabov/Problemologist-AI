## 1. Learning Objective

Test whether an engineer can meter a falling ball through a timed rotating gate while respecting the gate swing keep-out and the benchmark-owned power-routing constraint. The benchmark provides a single `rotate_z` gate axis motion that the engineer solution must synchronize with, and the engineer-owned moving assembly must stay outside the gate swing keep-out while executing its metering trajectory.

## 2. Environment Geometry

- `entry_ramp`: static ramp centered at `[-150, 0, 85]` with size `[180, 120, 50]`. Conditions the ball before the gate timing window.
- `gate_housing`: static frame centered at `[100, 0, 75]` with size `[120, 140, 150]`. Houses the gate pivot axis.
- `gate_pivot_arm`: slender flap centered at `[100, 0, 70]` with size `[18, 120, 90]`; benchmark-side motion is one `rotate_z` axis only. Opens a timing window that the engineer solution must exploit.
- `exit_tray`: static tray centered at `[320, 0, 25]` with size `[160, 110, 30]`. Receives the released ball after the gate opens.

## 3. Input Object

- Shape: `sphere`
- Label: `projectile_ball`
- Static randomization:
  - radius in `[18, 20]` mm
- Nominal start position: `[-220, 0, 115]`
- Runtime jitter: `[6, 6, 4]` mm

## 4. Objectives

- `goal_zone`: min `[280, -45, 20]`, max `[380, 45, 110]`
- `forbid_zones`:
  - `gate_swing_keepout`: min `[40, -70, 0]`, max `[160, 70, 150]`. The engineer-owned solution (including any moving parts) must not enter this volume at any point during execution.
- `build_zone`: min `[-260, -140, 0]`, max `[420, 140, 260]`

## 5. Simulation Bounds

- min `[-320, -180, -10]`, max `[480, 180, 320]`

## 6. Benchmark-Owned Motion Contract

The benchmark owns exactly one moving fixture:

- **Fixture identity**: `gate_pivot_arm`
- **Motion topology**: single DOF, `rotate_z` axis only
- **Motion kind**: `motorized_revolute` (benchmark-owned sinusoidal actuation)
- **Motion axis reference**: z-axis through the gate pivot center at `[100, 0, 70]`
- **Motion bounds**: full rotation cycle with periodic open/close window; the engineer solution must time its metering release to coincide with the open phase
- **Operating envelope**: the gate swing sweep is bounded by the `gate_swing_keepout` forbid zone declared in objectives
- **Trigger mode**: always-on periodic motion, externally triggered by benchmark simulation start
- **Engineer reliance**: the engineer solution may rely on this motion being available and deterministic during execution, but must not modify or add benchmark DOFs

The engineer solution may introduce its own moving parts (for example, a metering wheel or translating carriage) provided those moving parts:

1. Stay entirely outside the `gate_swing_keepout` volume throughout their trajectory
2. Do not require additional benchmark-owned motion beyond the declared gate pivot
3. Preserve the same moving-part set across the coarse `motion_forecast` in `assembly_definition.yaml` and the refined `payload_trajectory_definition.yaml` when drafting mode is active

## 7. Constraints Handed To Engineering

- Benchmark/customer caps: `max_unit_cost <= 105 USD`, `max_weight <= 1700 g`
- Benchmark-owned motion must stay limited to one `rotate_z` gate axis with no auxiliary sliders or hidden second motors.
- The declared wiring corridor must avoid the gate swing keep-out volume.
- Engineer-owned moving parts must execute a build-zone-valid start to goal-zone-contact trajectory that remains outside the gate swing keep-out.
- When drafting mode is active (evidence scripts and technical drawings present), the engineer planner must provide a coarse `motion_forecast` in `assembly_definition.yaml` with ordered world-frame anchors, explicit build-zone-valid first anchor, goal-zone terminal proof, and first-contact order. The engineer coder may refine this in `payload_trajectory_definition.yaml` without changing the moving-part set or loosening the approved endpoint proof.

## 8. Success Criteria

- Success if the moved object reaches `goal_zone` without entering `gate_swing_keepout`.
- Fail if the ball exits `simulation_bounds`, clips through undeclared gate geometry, or the benchmark relies on extra motion not declared in `benchmark_assembly_definition.yaml`.
- Fail if any engineer-owned moving part enters the `gate_swing_keepout` during execution.

## 9. Planner Artifacts

- `todo.md` tracks the ramp, gate housing, gate arm, and exit tray implementation.
- `benchmark_definition.yaml` mirrors the gate keep-out, power budget, and objective layout.
- `benchmark_assembly_definition.yaml` records the single rotating gate DOF plus benchmark-local costs.
- `benchmark_plan_evidence_script.py` and `benchmark_plan_technical_drawing_script.py` provide the drafting package for benchmark geometry review.
