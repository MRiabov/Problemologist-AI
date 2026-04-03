# Engineering Plan

## 1. Solution Overview

Use a compact single `ServoMotor_DS3218`-driven metering wheel ahead of the benchmark `gate_housing` so the seeded `projectile_ball` is released only when the benchmark-owned `gate_pivot_arm` is in its open window. The benchmark fixtures - `entry_ramp`, `gate_housing`, `gate_pivot_arm`, and `exit_tray` - are read-only context, and the engineer-owned solution stays left of the `gate_swing_keepout` while reserving a clean wiring corridor for the motor and return wire.

- `entry_ramp` conditions the ball before the gate timing window.
- `gate_housing` and `gate_pivot_arm` define the only benchmark motion, a single `rotate_z` axis.
- `exit_tray` receives the released ball after the gate opens.
- `goal_zone` must be reached without touching `gate_swing_keepout`.
- The engineer-owned stage is named `timed_metering_stage`.
- The only COTS motor in the plan is `ServoMotor_DS3218`.
- The numeric checks below use the exact catalog dimensions and material densities that back the assembly totals.

## 2. Parts List

| Part | Dimensions (mm) | Material | Purpose |
| -- | -- | -- | -- |
| base_plate | 520 x 130 x 10 | aluminum_6061 | Freestanding mounting surface with a through-window over the gate swing keep-out |
| settling_chute | 220 x 90 x 35 | hdpe | Settles the ball before it reaches the metering wheel |
| metering_wheel_guard | 140 x 55 x 32 | hdpe | Houses the motorized escapement wheel while preserving the gate keep-out |
| guide_rail | 150 x 18 x 24 | hdpe | Keeps the released ball on line toward the gate opening |
| post_gate_channel | 220 x 70 x 30 | hdpe | Captures the ball after the gate and steers it into the goal zone |
| goal_cup | 95 x 95 x 35 | hdpe | Final receiver that contains the ball once it exits the gate corridor |
| drive_motor | catalog `ServoMotor_DS3218` motor | cots | Single motorized DOF for the metering wheel |

**Estimated Total Weight**: 298.95 g
**Estimated Total Cost**: $69.00

## 3. Assembly Strategy

1. Place the `base_plate` entirely within `build_zone` with its centerline at x = 0 and keep the gate swing keep-out open through the middle window.
2. Mount `settling_chute` on the spawn side so the ball enters the metering wheel pocket with repeatable speed and orientation.
3. Mount `metering_wheel_guard` and `drive_motor` on the left side of the gate and keep the wiring corridor outside the keep-out.
4. Mount `guide_rail` and `post_gate_channel` downstream of the release point, then terminate the path in `goal_cup` whose interior overlaps `goal_zone`.
5. Preserve the benchmark-owned `entry_ramp`, `gate_housing`, `gate_pivot_arm`, and `exit_tray` geometry as read-only context while checking the release path against the gate opening.
6. The drafting sheet callouts `1`-`7` track the base plate, settling chute, metering wheel guard, guide rail, post-gate channel, goal cup, and the left-side drive motor corridor, respectively.

## 4. Assumption Register

| ID | Assumption | Source | Used By |
| -- | -- | -- | -- |
| ASSUMP-001 | The `ServoMotor_DS3218` body proxy measures `40.0 x 20.0 x 40.5 mm` and is mounted at `(-92, -68, 191)` in the drafting companion. | `shared/cots/parts/motors.py`, `solution_plan_evidence_script.py` | CALC-002 |
| ASSUMP-002 | Aluminum 6061 has density `2.7 g/cm^3` and HDPE has density `0.95 g/cm^3`. | `worker_heavy/workbenches/manufacturing_config.yaml` | CALC-004 |
| ASSUMP-003 | The drafted `goal_cup` placement is the intended capture location and the goal zone bounds in `benchmark_definition.yaml` are authoritative. | `solution_plan_evidence_script.py`, `benchmark_definition.yaml` | CALC-003 |
| ASSUMP-004 | The benchmark-owned gate motion remains a single `rotate_z` axis on the pivot arm, and the engineer solution may rely on the open window but not on any added benchmark DOFs. | `benchmark_assembly_definition.yaml`, `benchmark_definition.yaml` | CALC-001, CALC-002 |

## 5. Detailed Calculations

| ID | Problem / Decision | Result | Impact |
| -- | -- | -- | -- |
| CALC-001 | Windowed base plate fit in the build zone and clear the gate swing keep-out | Outer envelope `x [-260, 260]`, `y [-65, 65]`, `z [0, 10]`; keep-out window `x [55, 175]`, `y [-72.5, 72.5]`, `z [-1, 11]` | The stage fits on a freestanding base and leaves a `120 mm` x-window over the keep-out. |
| CALC-002 | Left-side motor corridor clears the gate swing keep-out | Motor body right edge at `x = -72 mm`, so x clearance to the keep-out is `132 mm` | The motor and return wire can stay on the left side without crossing the swing volume. |
| CALC-003 | Goal cup overlap with the goal zone | Overlap volume is `91,125 mm^3` with `x 67.5 mm`, `y 90 mm`, `z 15 mm` overlap | The cup reliably captures the ball inside the goal zone. |
| CALC-004 | Budget rollup from part masses and costs | `298.95 g` and `$69.00` total | The plan stays below the planner caps with substantial margin. |

### CALC-001: Windowed base plate fit in the build zone and clear the gate swing keep-out

#### Problem Statement

The solution needs a stable base that does not exceed the build zone and does not intrude into the gate swing keep-out.

#### Assumptions

- `ASSUMP-004`: The benchmark motion is fixed to the seeded gate pivot and does not require extra benchmark DOFs.
- The base plate uses the drafted `520 x 130 x 10 mm` footprint and is implemented as a single plate with a through-window centered on the keep-out.
- The build zone in `benchmark_definition.yaml` is authoritative.

#### Derivation

- Outer x span = `[-260, 260]`
- Outer y span = `[-65, 65]`
- Outer z span = `[0, 10]`
- The through-window spans `x [55, 175]`, `y [-72.5, 72.5]`, `z [-1, 11]`, so the keep-out `x [60, 170]` sits fully inside the opening with 5 mm margin on each side.
- Margins to the build zone are 0 mm on the left, 0 mm on the right, 75 mm on both y sides, and 230 mm on top.

#### Worst-Case Check

- No solid occupies the keep-out x-range, and no coordinate exceeds the allowed envelope.

#### Result

- The windowed base plate fits and clears the gate swing keep-out.

#### Design Impact

- Keep the outer footprint centered and preserve the 120 mm clearance window across the gate swing keep-out.

#### Cross-References

- `plan.md#3-assembly-strategy`
- `benchmark_definition.yaml`
- `assembly_definition.yaml`

### CALC-002: Left-side motor corridor clears the gate swing keep-out

#### Problem Statement

The servo and its cable exit must stay outside the benchmark `gate_swing_keepout`.

#### Assumptions

- `ASSUMP-001`: `ServoMotor_DS3218` uses the `40.0 x 20.0 x 40.5 mm` body proxy from `shared/cots/parts/motors.py`.
- The drafted mount location in `solution_plan_evidence_script.py` is `(-92, -68, 191)`.

#### Derivation

- Motor body x envelope = `[-112, -72]`
- Motor body y envelope = `[-78, -58]`
- Motor body z envelope = `[191, 231.5]`
- The keep-out starts at `x = 60`.
- The x-axis clearance from the motor body's right edge to the keep-out is `60 - (-72) = 132 mm`.
- The cable exit and shaft remain even farther left than the body envelope's right edge.
- The full motor proxy stays below the build-zone ceiling because the shaft top is `238.79 mm`, which is under `240 mm`.

#### Worst-Case Check

- Because the motor body never reaches x = 60, the keep-out is not intersected even though the y ranges overlap.

#### Result

- The motor corridor is legally left of the swing volume.

#### Design Impact

- Route the motor cable and any service loop along the left side of the stage.

#### Cross-References

- `plan.md#3-assembly-strategy`
- `solution_plan_evidence_script.py`
- `benchmark_definition.yaml`
- `shared/cots/parts/motors.py`

### CALC-003: Goal cup overlap with the goal zone

#### Problem Statement

The receiver must actually intersect the goal zone rather than just sit nearby.

#### Assumptions

- `ASSUMP-003`: The goal cup is centered at x = 320 mm and uses the `95 x 95 x 35 mm` draft geometry from the evidence script.
- The goal zone bounds in `benchmark_definition.yaml` are authoritative.

#### Derivation

- Goal cup x envelope = `[272.5, 367.5]`
- Goal cup y envelope = `[-47.5, 47.5]`
- Goal cup z envelope = `[0, 35]`
- Goal zone x envelope = `[300, 390]`
- Goal zone y envelope = `[-45, 45]`
- Goal zone z envelope = `[20, 110]`
- Overlap extents: x = `67.5 mm`, y = `90 mm`, z = `15 mm`
- Overlap volume = `67.5 x 90 x 15 = 91,125 mm^3`

#### Worst-Case Check

- All three axes overlap, so the cup is not merely adjacent to the goal zone.

#### Result

- The goal cup is a valid capture receiver.

#### Design Impact

- Keep the cup centered near x = 320 mm and do not lower it below z = 20 mm if the placement changes.

#### Cross-References

- `plan.md#3-assembly-strategy`
- `benchmark_definition.yaml`
- `solution_plan_evidence_script.py`

### CALC-004: Budget rollup from part masses and costs

#### Problem Statement

The plan must stay under the planner target cost and weight caps.

#### Assumptions

- `ASSUMP-002`: Aluminum 6061 density is `2.7 g/cm^3` and HDPE density is `0.95 g/cm^3`.
- The COTS servo mass and unit cost come from the catalog entry.

#### Derivation

- Base plate weight = `67.6 cm^3 x 2.7 = 182.52 g`
- Settling chute weight = `16.5 cm^3 x 0.95 = 15.675 g`
- Metering wheel guard weight = `9.0 cm^3 x 0.95 = 8.55 g`
- Guide rail weight = `6.5 cm^3 x 0.95 = 6.175 g`
- Post-gate channel weight = `15.4 cm^3 x 0.95 = 14.63 g`
- Goal cup weight = `12.0 cm^3 x 0.95 = 11.40 g`
- Servo weight = `60.0 g`
- Total weight = `298.95 g`
- Total cost = `16.00 + 8.00 + 6.50 + 5.00 + 8.50 + 7.00 + 18.00 = $69.00`
- Planner target headroom = `$79.00 - $69.00 = $10.00` and `1200.0 g - 298.95 g = 901.05 g`

#### Worst-Case Check

- Both totals stay well below the planner caps and even farther below the benchmark caps.

#### Result

- The budget is feasible with substantial margin.

#### Design Impact

- The implementation can absorb normal revision churn without breaching the target caps.

#### Cross-References

- `assembly_definition.yaml`
- `benchmark_definition.yaml`
- `worker_heavy/workbenches/manufacturing_config.yaml`

## 6. Critical Constraints / Operating Envelope

| Limit ID | Limit | Bound | Basis |
| -- | -- | -- | -- |
| LIMIT-001 | Base plate footprint | `x [-260, 260]`, `y [-65, 65]`, `z [0, 10]` | `CALC-001` |
| LIMIT-002 | Motor corridor clearance | Motor body right edge stays at or below `x = -72 mm`, which is `132 mm` left of the keep-out | `CALC-002` |
| LIMIT-003 | Goal capture overlap | Overlap extents must stay positive in x, y, and z | `CALC-003` |
| LIMIT-004 | Budget envelope | `<= $79.00` and `<= 1200.0 g` | `CALC-004` |

- Build zone: keep `base_plate` and all downstream pieces within the permitted footprint.
- Gate keep-out: do not enter `gate_swing_keepout` with the motor, guard, or wiring.
- Motion contract: the benchmark-owned gate remains the only moving benchmark fixture.
- Goal zone: `goal_cup` must overlap the goal zone and absorb the released ball.
- Budget envelope: preserve the planned cost and weight margin with a single small servo-grade motor.

## 7. Cost & Weight Budget

| Item | Volume (cm^3) | Weight (g) | Cost ($) |
| -- | -- | -- | -- |
| base_plate | 67.6 | 182.52 | 16.00 |
| settling_chute | 16.5 | 15.675 | 8.00 |
| metering_wheel_guard | 9.0 | 8.55 | 6.50 |
| guide_rail | 6.5 | 6.175 | 5.00 |
| post_gate_channel | 15.4 | 14.63 | 8.50 |
| goal_cup | 12.0 | 11.40 | 7.00 |
| ServoMotor_DS3218 | n/a | 60.00 | 18.00 |
| **TOTAL** | 127.0 | 298.95 | **69.00** |

**Budget Margin**: 12.7% cost headroom and 75.1% weight headroom versus the planner target.

## 8. Risk Assessment

| Risk | Likelihood | Impact | Mitigation |
| -- | -- | -- | -- |
| Metering wheel or wiring enters the gate swing keep-out | Low | High | Reference the seeded `gate_swing_keepout` AABB and keep the motor on the left side only |
| Ball releases at the wrong phase and misses the gate opening | Medium | High | Use the metering wheel to hold a single ball and release only near the opening window |
| Ball exits the gate but rebounds out of the goal | Medium | Medium | Terminate the post-gate channel in a goal cup with a small drop for energy absorption |
| Power budget is too tight for the chosen motor | Low | Medium | Keep to one small servo-grade motor and preserve the seeded supply headroom in `assembly_definition.yaml` |

### Jitter Robustness Check

- Capture area covers spawn jitter: Yes
- Tested edge cases considered: early release, late release, left-offset spawn, right-offset spawn
