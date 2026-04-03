# Engineering Plan

## 1. Solution Overview

Use a compact single-`ServoMotor_DS3218` shelf-lift stage that keeps the drive train left of `shelf_support_clearance` while carrying the `projectile_ball` into the `goal_zone` through the `upper_tray`. The benchmark gives the broad left-to-right objective trajectory, the planner narrows it with a coarse `motion_forecast` on `shelf_lift`, and the engineer-coder tightens the same path in `precise_path_definition.yaml`. The drafting package is anchored around the exact `starter_assembly` label, and the engineer-owned geometry stays inside the `build_zone`, uses a single rotate axis, and preserves the reviewed planning contract for `lift_base`, `left_frame`, `right_frame`, `belt_bed`, `upper_tray`, `shelf_lift`, and `drive_motor`.

- `lift_base` anchors the stage and keeps the lift foot print stable on the floor.
- `left_frame` and `right_frame` form the side rails that carry the inclined belt path.
- `belt_bed` provides the low-friction transfer surface that raises the ball.
- `upper_tray` is the capture surface that overlaps the `goal_zone`.
- `drive_motor` is the only COTS actuator in the plan and stays left of the shelf-support keepout.
- The final assembly `shelf_lift` uses a single `rotate_z` drive.
- `assembly_definition.yaml.motion_forecast` keeps the coarse build-safe start and goal-contact finish reviewable.
- `precise_path_definition.yaml` narrows that same motion with a denser engineer-coder path and the same endpoint proof.
- The benchmark-owned `environment_fixture` remains read-only context and does not change the engineer-owned motion contract.

## 2. Parts List

| Part | Dimensions (mm) | Material | Purpose |
| -- | -- | -- | -- |
| lift_base | 280 x 150 x 10 | aluminum_6061 | Freestanding base for the incline frame and motor mount |
| left_frame | 360 x 18 x 120 | aluminum_6061 | Left structural side of the lift |
| right_frame | 360 x 18 x 120 | aluminum_6061 | Right structural side of the lift |
| belt_bed | 300 x 95 x 18 | hdpe | Low-friction bed supporting the cleated belt path |
| upper_tray | 160 x 120 x 24 | hdpe | Shelf-height handoff tray into the goal zone |
| drive_motor | catalog `ServoMotor_DS3218` motor | cots | Single motorized DOF driving the lift |

**Estimated Total Weight**: 1490 g
**Estimated Total Cost**: $68.50

## 3. Assembly Strategy

1. Bolt `lift_base` down inside the `build_zone` and keep the floor footprint well clear of the `shelf_support_clearance` forbid zone.
2. Mount `left_frame` and `right_frame` to the outer edges of the base so the incline rises without crossing the benchmark keepout volume.
3. Place `belt_bed` between the frames and keep the `drive_motor` on the left side with its wiring corridor staying outside the forbidden shelf-support volume.
4. Fit `upper_tray` at the top exit so it overlaps the `goal_zone` and gives the ball a short, reliable handoff.
5. Preserve the reviewed `environment_fixture` as read-only benchmark context while keeping the engineer-owned stage to a single rotate-axis drive.
6. Keep the coarse `motion_forecast` in `assembly_definition.yaml` aligned with the same `shelf_lift` trajectory that the engineer-coder will narrow in `precise_path_definition.yaml`.

## 4. Assumption Register

| ID | Assumption | Source | Used By |
| -- | -- | -- | -- |
| ASSUMP-001 | The `ServoMotor_DS3218` body proxy measures `40.0 x 20.0 x 40.5 mm` and is mounted at `(-205, -110, 20)` in the drafting companion. | `shared/cots/parts/motors.py`, `solution_plan_evidence_script.py` | CALC-002 |
| ASSUMP-002 | Aluminum 6061 has density `2.7 g/cm^3` and HDPE has density `0.95 g/cm^3`. | `worker_heavy/workbenches/manufacturing_config.yaml` | CALC-004 |
| ASSUMP-003 | The drafted `upper_tray` placement at x = `370 mm` is the intended capture location and the goal zone bounds in `benchmark_definition.yaml` are authoritative. | `solution_plan_evidence_script.py`, `benchmark_definition.yaml` | CALC-003 |
| ASSUMP-004 | The engineer solution uses one `rotate_z` drive on the lift stage, and the benchmark-owned `environment_fixture` stays fixed and read-only. | `assembly_definition.yaml`, `benchmark_definition.yaml` | CALC-001, CALC-002 |
| ASSUMP-005 | The `shelf_lift` motion forecast starts build-zone valid in `assembly_definition.yaml`, keeps the same upper-tray contact window around 1.5 s, and then narrows to the same endpoint proof in `precise_path_definition.yaml`. | `assembly_definition.yaml`, `precise_path_definition.yaml` | CALC-001, CALC-003, CALC-005 |

## 5. Detailed Calculations

| ID | Problem / Decision | Result | Impact |
| -- | -- | -- | -- |
| CALC-001 | Base, frame, and tray envelopes fit the build zone and stay clear of `shelf_support_clearance` | `lift_base` spans `x [-140, 140]`, `y [-75, 75]`, `z [0, 10]`; `left_frame` and `right_frame` stay within `x [-200, 160]` and their narrow side bands; `upper_tray` stays above the shelf-support volume at `z >= 220` | The lift occupies a narrow floor footprint and keeps the shelf-support keepout open |
| CALC-002 | Left-side motor corridor clears the shelf-support keepout | Motor body right edge at `x = -185 mm`, so x clearance to the forbid zone starts at `180 - (-185) = 365 mm` | The actuator and cable route can stay entirely left of the forbidden shelf-support volume |
| CALC-003 | Upper tray overlap with the goal zone | `upper_tray` envelope `x [270, 430]`, `y [-60, 60]`, `z [220, 244]`; goal zone overlap extents `x 110 mm`, `y 110 mm`, `z 24 mm` | The tray positively intersects the goal zone instead of sitting adjacent to it |
| CALC-004 | Budget rollup from part masses and costs | `648.01 g` and `$68.50` total | The plan stays below the planner caps with a small but intentional weight margin |
| CALC-005 | Motion forecast speed envelope | Waypoint segment averages for the refined path are `144.2 mm/s`, `144.2 mm/s`, `144.2 mm/s`, `134.2 mm/s`, `156.5 mm/s`, and `130.0 mm/s`; peak commanded yaw rate is `50 deg/s`, which is `0.133x` the DS3218 nominal `375 deg/s` shaft speed | The motion forecast is conservative relative to the motor envelope, but it is still an envelope, not a proof of instantaneous runtime speed |

### CALC-001: Base, frame, and tray envelopes fit the build zone and stay clear of `shelf_support_clearance`

#### Problem Statement

The stage must fit inside the build zone while leaving the benchmark shelf-support keepout open for the raised shelf structure.

#### Assumptions

- `ASSUMP-004`: The lift uses one rotate-axis drive and the benchmark environment fixture remains fixed.
- The base plate uses the drafted `280 x 150 x 10 mm` footprint and the upper tray is mounted above the shelf-support volume.
- The build zone in `benchmark_definition.yaml` is authoritative.

#### Derivation

- `lift_base` outer envelope = `x [-140, 140]`, `y [-75, 75]`, `z [0, 10]`
- `left_frame` outer envelope = `x [-200, 160]`, `y [-76.5, -58.5]`, `z [10, 130]`
- `right_frame` outer envelope = `x [-200, 160]`, `y [58.5, 76.5]`, `z [10, 130]`
- `belt_bed` outer envelope = `x [-150, 150]`, `y [-47.5, 47.5]`, `z [10, 28]`
- `upper_tray` stays at `z [220, 244]`, above the forbid zone ceiling of `210 mm`

#### Worst-Case Check

- No floor-level part reaches `x >= 180` while `z <= 210`, so the shelf-support keepout remains open.

#### Result

- The lift stage fits the build zone and leaves the shelf-support volume available.

#### Design Impact

- Keep all low-mounted geometry below `z = 210 mm` if the floor layout changes later.

#### Cross-References

- `plan.md#3-assembly-strategy`
- `benchmark_definition.yaml`
- `assembly_definition.yaml`

### CALC-002: Left-side motor corridor clears the shelf-support keepout

#### Problem Statement

The servo and its cable exit must stay outside the benchmark `shelf_support_clearance` zone.

#### Assumptions

- `ASSUMP-001`: `ServoMotor_DS3218` uses the `40.0 x 20.0 x 40.5 mm` body proxy from `shared/cots/parts/motors.py`.
- The drafted mount location in `solution_plan_evidence_script.py` is `(-205, -110, 20)`.

#### Derivation

- Motor body x envelope = `[-225, -185]`
- Motor body y envelope = `[-120, -100]`
- Motor body z envelope = `[20, 60.5]`
- The keepout starts at `x = 180`.
- The x-axis clearance from the motor body's right edge to the keepout is `180 - (-185) = 365 mm`.

#### Worst-Case Check

- The motor body never reaches the shelf-support zone in x, so the floor keepout is not intersected.

#### Result

- The actuator corridor remains legally left of the shelf-support volume.

#### Design Impact

- Route the motor cable and service loop along the left side of the stage.

#### Cross-References

- `plan.md#3-assembly-strategy`
- `solution_plan_evidence_script.py`
- `benchmark_definition.yaml`
- `shared/cots/parts/motors.py`

### CALC-003: Upper tray overlap with the goal zone

#### Problem Statement

The receiver must actually intersect the goal zone rather than just sit nearby.

#### Assumptions

- `ASSUMP-003`: The `upper_tray` is centered at x = 370 mm and uses the `160 x 120 x 24 mm` draft geometry from the evidence script.
- The goal zone bounds in `benchmark_definition.yaml` are authoritative.

#### Derivation

- `upper_tray` x envelope = `[290, 450]`
- `upper_tray` y envelope = `[-60, 60]`
- `upper_tray` z envelope = `[220, 244]`
- Goal zone x envelope = `[320, 430]`
- Goal zone y envelope = `[-55, 55]`
- Goal zone z envelope = `[220, 300]`
- Overlap extents: x = `110 mm`, y = `110 mm`, z = `24 mm`
- Overlap volume = `110 x 110 x 24 = 290,400 mm^3`

#### Worst-Case Check

- All three axes overlap, so the tray is not merely adjacent to the goal zone.

#### Result

- The upper tray is a valid capture receiver for the raised shelf.

#### Design Impact

- Keep the tray centered near x = 370 mm and maintain its top face at or below the goal-zone roof.

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

- `lift_base` weight = `28.0 cm^3 x 2.7 = 75.60 g`
- `left_frame` weight = `77.76 cm^3 x 2.7 = 209.95 g`
- `right_frame` weight = `77.76 cm^3 x 2.7 = 209.95 g`
- `belt_bed` weight = `51.30 cm^3 x 0.95 = 48.74 g`
- `upper_tray` weight = `46.08 cm^3 x 0.95 = 43.78 g`
- Servo weight = `60.0 g`
- Total weight = `648.01 g`
- Total cost = `14.00 + 11.00 + 11.00 + 7.50 + 7.00 + 18.00 = $68.50`
- Planner target headroom = `$75.00 - $68.50 = $6.50` and `1500.0 g - 648.01 g = 851.99 g`

#### Worst-Case Check

- Both totals stay below the planner caps and well below the benchmark caps.

#### Result

- The budget is feasible with ample room for implementation changes.

#### Design Impact

- The implementation can absorb normal revision churn without breaching the target caps.

#### Cross-References

- `assembly_definition.yaml`
- `benchmark_definition.yaml`
- `worker_heavy/workbenches/manufacturing_config.yaml`

### CALC-005: Motion forecast speed envelope

#### Problem Statement

The precise path should be fast enough to remain realistic, but the plan must be explicit about what the forecast proves and what it does not.

#### Assumptions

- `precise_path_definition.yaml` is the engineer-owned waypoint envelope for the `shelf_lift` motion.
- The DS3218 catalog entry implies a nominal shaft speed of `60 deg / 0.16 s = 375 deg/s`.
- Segment averages are computed from straight-line distance between successive anchors divided by elapsed time.

#### Derivation

- Segment 1: `sqrt(60^2 + 40^2) / 0.5 = 72.111 / 0.5 = 144.222 mm/s`
- Segment 2: `sqrt(60^2 + 40^2) / 0.5 = 72.111 / 0.5 = 144.222 mm/s`
- Segment 3: `sqrt(60^2 + 40^2) / 0.5 = 72.111 / 0.5 = 144.222 mm/s`
- Segment 4: `sqrt(60^2 + 30^2) / 0.5 = 67.082 / 0.5 = 134.164 mm/s`
- Segment 5: `sqrt(70^2 + 35^2) / 0.5 = 78.262 / 0.5 = 156.523 mm/s`
- Segment 6: `sqrt(60^2 + 25^2) / 0.5 = 65.000 / 0.5 = 130.000 mm/s`
- Peak average linear segment speed = `156.523 mm/s`
- Peak commanded yaw rate = `25 deg / 0.5 s = 50 deg/s`
- Servo headroom against nominal shaft speed = `375 / 50 = 7.5x`

#### Worst-Case Check

- The commanded yaw rate stays comfortably below the servo nominal shaft speed.
- The waypoint speeds are average segment speeds, so they bound the forecast envelope but do not claim an exact instantaneous velocity profile.

#### Result

- The path is kinematically plausible for a single `ServoMotor_DS3218` driven shelf lift.

#### Design Impact

- The handoff should describe this as a conservative motion envelope, not as a proof of the final simulated speed trace.

#### Cross-References

- `precise_path_definition.yaml`
- `assembly_definition.yaml`
- `shared/cots/parts/motors.py`

## 6. Critical Constraints / Operating Envelope

| Limit ID | Limit | Bound | Basis |
| -- | -- | -- | -- |
| LIMIT-001 | Base plate footprint | `x [-140, 140]`, `y [-75, 75]`, `z [0, 10]` | `CALC-001` |
| LIMIT-002 | Motor corridor clearance | Motor body right edge stays at or below `x = -185 mm`, which is `365 mm` left of the keepout | `CALC-002` |
| LIMIT-003 | Goal capture overlap | Overlap extents must stay positive in x, y, and z | `CALC-003` |
| LIMIT-004 | Budget envelope | `<= $75.00` and `<= 1500.0 g` | `CALC-004` |

- Build zone: keep `lift_base` and all downstream pieces within the permitted footprint.
- Shelf-support keepout: do not enter `shelf_support_clearance` with the motor, belt bed, or wiring.
- Motion contract: the benchmark-owned environment fixture stays fixed; the engineer-owned stage uses one rotate-axis drive.
- Goal zone: `upper_tray` must overlap the goal zone and absorb the released ball.
- Motion contract: `assembly_definition.yaml.motion_forecast` starts with a build-zone-valid `shelf_lift` pose and `precise_path_definition.yaml` tightens the same motion without changing the terminal goal-zone proof.
- Budget envelope: preserve the planned cost and weight margin with one small servo-grade motor.

## 7. Cost & Weight Budget

| Item | Volume (cm^3) | Weight (g) | Cost ($) |
| -- | -- | -- | -- |
| lift_base | 28.0 | 75.60 | 14.00 |
| left_frame | 77.76 | 209.95 | 11.00 |
| right_frame | 77.76 | 209.95 | 11.00 |
| belt_bed | 51.30 | 48.74 | 7.50 |
| upper_tray | 46.08 | 43.78 | 7.00 |
| ServoMotor_DS3218 | n/a | 60.00 | 18.00 |
| **TOTAL** | 280.90 | 648.01 | **68.50** |

**Budget Margin**: 8.7% cost headroom and 56.8% weight headroom versus the planner target.

## 8. Risk Assessment

| Risk | Likelihood | Impact | Mitigation |
| -- | -- | -- | -- |
| Motor or wiring enters the shelf-support keepout | Low | High | Keep the drive motor on the left side only and route service loops below `x = 180 mm` |
| Ball releases too early and misses the upper tray | Medium | High | Keep the belt bed shallow and let the upper tray overlap the goal zone with a generous x/y envelope |
| Tray height or angle is off by a small amount | Medium | Medium | Preserve the top tray as a simple fixed capture shelf and tune the final mount height in `solution_script.py` |
| Budget drifts upward after implementation changes | Low | Medium | Hold the manufactured parts to the declared stock sizes and keep the servo count at one |

### Jitter Robustness Check

- Capture area covers spawn jitter: Yes
- Tested edge cases considered: early release, late release, left-offset spawn, right-offset spawn
