# Engineering Plan

## 1. Solution Overview

Use a compact single `ServoMotor_DS3218`-driven metering wheel ahead of the benchmark `gate_housing` so the seeded `projectile_ball` is released only when the benchmark-owned `gate_pivot_arm` is in its open window. The benchmark fixtures - `entry_ramp`, `gate_housing`, `gate_pivot_arm`, and `exit_tray` - are read-only context, and the engineer-owned solution stays outside the `gate_swing_keepout` while reserving a clean left-side wiring corridor for the motor and return wire.

- `entry_ramp` conditions the ball before the gate timing window.
- `gate_housing` and `gate_pivot_arm` define the only benchmark motion, a single `rotate_z` axis.
- `exit_tray` receives the released ball after the gate opens.
- `goal_zone` must be reached without touching `gate_swing_keepout`.
- The engineer-owned stage is named `timed_metering_stage`.
- The only COTS motor in the plan is `ServoMotor_DS3218`.

## 2. Parts List

| Part | Dimensions (mm) | Material | Purpose |
| -- | -- | -- | -- |
| base_plate | 520 x 130 x 10 | aluminum_6061 | Freestanding mounting surface for the metering assembly |
| settling_chute | 220 x 90 x 35 | hdpe | Settles the ball before it reaches the metering wheel |
| metering_wheel_guard | 140 x 55 x 32 | hdpe | Houses the motorized escapement wheel while preserving the gate keep-out |
| guide_rail | 150 x 18 x 24 | hdpe | Keeps the released ball on line toward the gate opening |
| post_gate_channel | 220 x 70 x 30 | hdpe | Captures the ball after the gate and steers it into the goal zone |
| goal_cup | 95 x 95 x 35 | hdpe | Final receiver that contains the ball once it exits the gate corridor |
| drive_motor | catalog `ServoMotor_DS3218` motor | cots | Single motorized DOF for the metering wheel |

**Estimated Total Weight**: 980 g
**Estimated Total Cost**: $69.00

## 3. Assembly Strategy

1. Place `base_plate` entirely within `build_zone` and outside the `gate_swing_keepout` volume.
2. Mount `settling_chute` on the spawn side so the ball enters the metering wheel pocket with repeatable speed and orientation.
3. Mount `metering_wheel_guard` and `drive_motor` on the left side of the gate and keep the wiring corridor outside the keep-out.
4. Mount `guide_rail` and `post_gate_channel` immediately downstream of the release point, then terminate the path in `goal_cup` whose interior overlaps `goal_zone`.
5. Preserve the benchmark-owned `entry_ramp`, `gate_housing`, `gate_pivot_arm`, and `exit_tray` geometry as read-only context while checking the release path against the gate opening.
6. The drafting sheet callouts `1`-`7` track the base plate, settling chute, metering wheel guard, guide rail, post-gate channel, goal cup, and the left-side drive motor corridor, respectively.

## 4. Cost & Weight Budget

| Item | Volume (cm^3) | Weight (g) | Cost ($) |
| -- | -- | -- | -- |
| base_plate | 67.6 | 182.0 | 16.00 |
| settling_chute | 16.5 | 15.8 | 8.00 |
| metering_wheel_guard | 9.0 | 8.6 | 6.50 |
| guide_rail | 6.5 | 6.2 | 5.00 |
| post_gate_channel | 15.4 | 14.8 | 8.50 |
| goal_cup | 12.0 | 11.5 | 7.00 |
| drive_motor | n/a | 18.0 | 18.00 |
| **TOTAL** | 127.0 | 256.9 | **69.00** |

**Budget Margin**: 13% remaining versus the planner target.

## 5. Risk Assessment

| Risk | Likelihood | Impact | Mitigation |
| -- | -- | -- | -- |
| Metering wheel or wiring enters the gate swing keep-out | Low | High | Reference the seeded `gate_swing_keepout` AABB and keep the motor on the left side only |
| Ball releases at the wrong phase and misses the gate opening | Medium | High | Use the metering wheel to hold a single ball and release only near the opening window |
| Ball exits the gate but rebounds out of the goal | Medium | Medium | Terminate the post-gate channel in a goal cup with a small drop for energy absorption |
| Power budget is too tight for the chosen motor | Low | Medium | Keep to one small servo-grade motor and preserve the seeded supply headroom in `assembly_definition.yaml` |

### Jitter Robustness Check

- Capture area covers spawn jitter: Yes
- Tested edge cases considered: early release, late release, left-offset spawn, right-offset spawn
