# Engineering Plan

## 1. Solution Overview

Use a compact incline lift with a raised bridge tray that hands the ball up to the shelf-height goal zone. The design stays self-contained inside the build zone and uses a single motorized lift path with a passive capture tray at the top.

## 2. Parts List

| Part | Dimensions (mm) | Material | Purpose |
| -- | -- | -- | -- |
| lift_base | 280 x 150 x 10 | aluminum_6061 | Low, stable base for the entire lift bridge |
| left_frame | 340 x 20 x 120 | aluminum_6061 | Left frame member that carries the lift path |
| right_frame | 340 x 20 x 120 | aluminum_6061 | Right frame member that carries the lift path |
| belt_bed | 300 x 90 x 18 | hdpe | Low-friction guide bed for the rising path |
| upper_tray | 170 x 120 x 24 | hdpe | Shelf-height receiving tray for the ball |
| guide_rail | 150 x 18 x 24 | hdpe | Side guide that keeps the ball centered during ascent |
| cable_shroud | 90 x 40 x 18 | aluminum_6061 | Cable cover and motor-side protection plate |
| motor_mount | 80 x 45 x 22 | aluminum_6061 | Motor mounting block for the incline drive |
| drive_motor | catalog gearmotor | cots | Main actuator for the lift bridge |
| pivot_bearings | catalog bearing set | cots | Low-friction support for the rising path |
| enable_switch | catalog switch | cots | Manual enable for the lift bridge |
| motor_relay | catalog relay | cots | Power switching for the drive motor |

**Estimated Total Weight**: 1040 g
**Estimated Total Cost**: $87.00

## 3. Assembly Strategy

1. Mount `left_frame` and `right_frame` to `lift_base` so the incline stays inside the build zone while reaching the shelf opening.
2. Mount `belt_bed` and `upper_tray` along a continuous rising path, then use `guide_rail` to keep the ball centered during the transition.
3. Keep the motor, relay, and switch on the outside-left side behind `cable_shroud` so the wiring does not intrude into the moving path.

## 4. Cost & Weight Budget

| Item | Weight (g) | Cost ($) |
| -- | -- | -- |
| lift_base | 330 | 14.0 |
| left_frame | 180 | 10.0 |
| right_frame | 180 | 10.0 |
| belt_bed | 50 | 7.0 |
| upper_tray | 40 | 6.0 |
| guide_rail | 20 | 5.0 |
| cable_shroud | 30 | 4.0 |
| motor_mount | 25 | 5.5 |
| drive_motor | 55 | 12.0 |
| pivot_bearings | 8 | 10.0 |
| enable_switch | 5 | 2.0 |
| motor_relay | 5 | 1.5 |
| **TOTAL** | **948** | **87.00** |

**Budget Margin**: 9% remaining versus the planner target.

## 5. Risk Assessment

| Risk | Likelihood | Impact | Mitigation |
| -- | -- | -- | -- |
| The ball stalls on the incline before reaching the shelf | Medium | High | Keep the guide bed shallow and continuous through the upper transition |
| The motor or cable shroud steals clearance from the upper tray | Low | High | Place the shroud outside the lift path and keep the tray tangent to the exit |
| The lift path wanders laterally and misses the tray opening | Medium | Medium | Use the side guide rail and keep the frame spacing symmetric |
