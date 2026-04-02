# Engineering Plan

## 1. Solution Overview

Use a compact single-motor incline lift that captures the projectile ball near floor level and carries it onto the raised shelf goal zone. The lift stays self-contained inside the build zone and hands the ball into a short upper tray aligned with the shelf opening.

## 2. Parts List

| Part | Dimensions (mm) | Material | Purpose |
| -- | -- | -- | -- |
| lift_base | 280 x 150 x 10 | aluminum_6061 | Freestanding base for the incline frame and motor mount |
| left_frame | 360 x 18 x 120 | aluminum_6061 | Left structural side of the lift |
| right_frame | 360 x 18 x 120 | aluminum_6061 | Right structural side of the lift |
| belt_bed | 300 x 95 x 18 | hdpe | Low-friction bed supporting the cleated belt path |
| upper_tray | 160 x 120 x 24 | hdpe | Shelf-height handoff tray into the goal zone |
| drive_motor | catalog gearmotor | cots | Single motorized DOF driving the lift |

**Estimated Total Weight**: 1490 g
**Estimated Total Cost**: $69.00

## 3. Assembly Strategy

1. Bolt `left_frame` and `right_frame` to `lift_base` so the incline reaches the raised shelf without entering the seeded support-clearance forbid zone.
2. Mount `belt_bed` between the frames and place `upper_tray` tangent to the top exit so the ball transitions cleanly into the goal zone.
3. Mount `drive_motor` at the lower end of the lift and preserve the seeded cable corridor along the outside of the left frame.

## 4. Cost & Weight Budget

| Item | Weight (g) | Cost ($) |
| -- | -- | -- |
| lift_base | 420 | 14.0 |
| left_frame | 220 | 11.0 |
| right_frame | 220 | 11.0 |
| belt_bed | 55 | 7.5 |
| upper_tray | 38 | 7.0 |
| drive_motor | 85 | 18.5 |
| **TOTAL** | **1038** | **69.0** |

**Budget Margin**: 27% remaining versus the planner target.

## 5. Risk Assessment

| Risk | Likelihood | Impact | Mitigation |
| -- | -- | -- | -- |
| Ball rolls backward on the incline | Medium | High | Keep the incline moderate and include shallow retaining features on the moving path |
| Lift top misses the shelf opening | Low | High | Align the upper tray directly to the seeded goal-zone centerline and shelf edge |
| Motor or cable intrudes into the shelf-support keepout | Low | Medium | Reserve the outer-left cable corridor and keep the motor below the support-clearance volume |
