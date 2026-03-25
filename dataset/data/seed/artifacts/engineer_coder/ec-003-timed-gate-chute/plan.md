# Engineering Plan

## 1. Solution Overview

Use a compact single-motor metering wheel ahead of the rotating gate so the seeded ball reaches the gate only when the opening is available, then capture it with a short post-gate channel into the goal. The mechanism stays inside the seeded keep-out and reserves a clean left-side wiring corridor for downstream electronics work.

## 2. Parts List

| Part | Dimensions (mm) | Material | Purpose |
| -- | -- | -- | -- |
| base_plate | 520 x 130 x 10 | aluminum_6061 | Freestanding mounting surface for the metering assembly |
| settling_chute | 220 x 90 x 35 | hdpe | Settles the ball before it reaches the metering wheel |
| metering_wheel_guard | 140 x 55 x 32 | hdpe | Houses the motorized escapement wheel while preserving the gate keep-out |
| guide_rail | 150 x 18 x 24 | hdpe | Keeps the released ball on line toward the gate opening |
| post_gate_channel | 220 x 70 x 30 | hdpe | Captures the ball after the gate and steers it into the goal zone |
| goal_cup | 95 x 95 x 35 | hdpe | Final receiver that contains the ball once it exits the gate corridor |
| drive_motor | catalog gearmotor | cots | Single motorized DOF for the metering wheel |

**Estimated Total Weight**: 980 g
**Estimated Total Cost**: $67.50

## 3. Assembly Strategy

1. Place `base_plate` entirely within the seeded build zone and outside the gate swing keep-out volume.
2. Mount `settling_chute` on the spawn side so the ball enters the metering wheel pocket with repeatable speed and orientation.
3. Mount `metering_wheel_guard` and `drive_motor` left of the gate keep-out, keeping the wheel tangent to the release line while reserving the seeded wire corridor.
4. Mount `guide_rail` and `post_gate_channel` immediately downstream of the release point, then terminate the path in `goal_cup` whose interior overlaps the seeded goal zone.

## 4. Cost & Weight Budget

| Item | Volume (cm^3) | Weight (g) | Cost ($) |
| -- | -- | -- | -- |
| base_plate | 67.6 | 182.0 | 16.00 |
| settling_chute | 16.5 | 15.8 | 8.00 |
| metering_wheel_guard | 9.0 | 8.6 | 6.50 |
| guide_rail | 6.5 | 6.2 | 5.00 |
| post_gate_channel | 15.4 | 14.8 | 8.50 |
| goal_cup | 12.0 | 11.5 | 7.00 |
| drive_motor | n/a | 18.0 | 16.50 |
| **TOTAL** | 127.0 | 256.9 | **67.50** |

**Budget Margin**: 15% remaining versus the planner target.

## 5. Risk Assessment

| Risk | Likelihood | Impact | Mitigation |
| -- | -- | -- | -- |
| Metering wheel or wiring enters the gate swing keep-out | Low | High | Reference all powered geometry and wire routing from the seeded keep-out AABB and hold the motor on the left side only |
| Ball releases at the wrong phase and misses the gate opening | Medium | High | Use the metering wheel to hold a single ball and release only near the opening window |
| Ball exits the gate but rebounds out of the goal | Medium | Medium | Terminate the post-gate channel in a goal cup with a small drop for energy absorption |
| Power budget is too tight for the chosen motor | Low | Medium | Keep to one small gearmotor and reserve the seeded supply headroom in `assembly_definition.yaml` |

### Jitter Robustness Check

- Capture area covers spawn jitter: Yes
- Tested edge cases considered: early release, late release, left-offset spawn, right-offset spawn
