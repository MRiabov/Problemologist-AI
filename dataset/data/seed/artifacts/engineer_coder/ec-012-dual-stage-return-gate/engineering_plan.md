# Engineering Plan

## 1. Solution Overview

Use a two-stage return gate. A low passive flap nudges the ball off the central dead zone, then a motorized upper gate catches it and drops it into the right-hand goal. Keep the motors and wiring on the rear-left side so the cable path never crosses the gate sweep volume.

## 2. Parts List

| Part | Dimensions (mm) | Material | Purpose |
| -- | -- | -- | -- |
| base_plate | 260 x 150 x 12 | aluminum_6061 | Freestanding base that anchors the full return-gate frame |
| left_frame | 260 x 20 x 110 | aluminum_6061 | Left structural sidewall for the two-stage return path |
| right_frame | 260 x 20 x 110 | aluminum_6061 | Right structural sidewall for the two-stage return path |
| primary_gate | 110 x 18 x 90 | hdpe | First rotating flap that kicks the ball out of the dead zone |
| return_flap | 90 x 16 x 70 | hdpe | Second rotating flap that hands the ball into the goal chute |
| capture_tray | 140 x 45 x 25 | hdpe | Final pocket aligned to the goal zone opening |
| control_box | 80 x 40 x 20 | aluminum_6061 | Electronics enclosure and cable tie-off point |
| drive_motor | catalog gearmotor | cots | Primary actuator for the first gate |
| return_motor | catalog gearmotor | cots | Secondary actuator for the return flap |
| pivot_bearings | catalog bearing pair | cots | Low-friction support for the gate pivots |
| relay | catalog relay | cots | Power switching for the timed gate sequence |
| enable_switch | catalog switch | cots | Manual enable for the gate sequence |

**Estimated Total Weight**: 1185 g
**Estimated Total Cost**: $103.50

## 3. Assembly Strategy

1. Bolt `left_frame` and `right_frame` to `base_plate` so the whole mechanism spans the dead zone without entering the forbid volume.
2. Mount `primary_gate` and `return_flap` on separate pivots, then align `capture_tray` with the goal centerline so the second gate delivers directly into the pocket.
3. Mount `control_box` behind the left frame and route both motor circuits through the rear-left cable corridor so nothing crosses the moving sweep paths.

## 4. Cost & Weight Budget

| Item | Weight (g) | Cost ($) |
| -- | -- | -- |
| base_plate | 340 | 15.0 |
| left_frame | 200 | 11.0 |
| right_frame | 200 | 11.0 |
| primary_gate | 60 | 7.5 |
| return_flap | 35 | 5.0 |
| capture_tray | 55 | 6.5 |
| control_box | 40 | 4.5 |
| drive_motor | 60 | 18.0 |
| return_motor | 60 | 18.0 |
| pivot_bearings | 5 | 5.0 |
| relay | 10 | 1.5 |
| enable_switch | 5 | 0.5 |
| **TOTAL** | **1070** | **103.50** |

**Budget Margin**: 11% remaining versus the planner target.

## 5. Risk Assessment

| Risk | Likelihood | Impact | Mitigation |
| -- | -- | -- | -- |
| The first gate kicks the ball back into the dead zone | Medium | High | Use a shallow deflection angle and keep the first pivot low enough to reject backspin |
| The second gate or cable slack intrudes into the goal chute | Medium | High | Reserve the rear-left cable chase and keep the capture tray fixed below the moving sweep |
| Timed actuation does not stay synchronized across both motors | Low | High | Keep the relay and switch logic simple and use one shared enable path |
