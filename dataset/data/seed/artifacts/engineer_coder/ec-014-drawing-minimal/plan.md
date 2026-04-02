# Engineering Plan

## 1. Solution Overview

Use a timed diverter gate with a secondary latch leaf. The diverter gate steers the ball into the right-hand goal while the latch leaf prevents rebound from the moving benchmark guard arm. Keep the motors and the control box on the rear-left side so the wiring stays out of the sweep zone.

## 2. Parts List

| Part | Dimensions (mm) | Material | Purpose |
| -- | -- | -- | -- |
| mount_base | 180 x 80 x 8 | aluminum_6061 | Fixed base plate that locates the diverter |
| gate_bracket | 80 x 40 x 20 | aluminum_6061 | Mounting bracket for the diverter and latch pivots |
| diverter_gate | 70 x 8 x 50 | hdpe | Main gate blade that rotates to redirect the ball |
| latch_leaf | 60 x 8 x 40 | hdpe | Secondary leaf that blocks rebound from the guard arm |
| cable_cover | 90 x 32 x 18 | aluminum_6061 | Cable cover and side guard for the wiring path |
| sensor_stand | 70 x 20 x 18 | aluminum_6061 | Small fixed post for cable tie-down and sensor alignment |
| control_box | 70 x 35 x 20 | aluminum_6061 | Electronics enclosure at the rear-left corner |
| gate_motor | catalog gearmotor | cots | Primary actuator for the diverter gate |
| latch_motor | catalog gearmotor | cots | Secondary actuator for the latch leaf |
| pivot_bearings | catalog bearing pair | cots | Low-friction support for the gate pivots |
| gate_relay | catalog relay | cots | Power switching for the timed diverter sequence |
| gate_switch | catalog switch | cots | Manual enable for the timed diverter sequence |

**Estimated Total Weight**: 1120 g
**Estimated Total Cost**: $104.00

## 3. Assembly Strategy

1. Bolt `mount_base` and `gate_bracket` together so the diverter assembly stays rigid while the two leaves move independently.
2. Mount `diverter_gate` and `latch_leaf` on separate pivots, then align `control_box` and `sensor_stand` behind the left frame so the wiring never crosses the moving sweep.
3. Keep the motors, relay, and switch inside the rear-left service corridor and preserve the guard-arm clearance declared in the benchmark handoff.

## 4. Cost & Weight Budget

| Item | Weight (g) | Cost ($) |
| -- | -- | -- |
| mount_base | 140 | 15.0 |
| gate_bracket | 60 | 12.0 |
| diverter_gate | 25 | 11.0 |
| latch_leaf | 18 | 7.0 |
| cable_cover | 22 | 6.0 |
| sensor_stand | 16 | 5.0 |
| control_box | 18 | 5.0 |
| gate_motor | 60 | 18.0 |
| latch_motor | 60 | 18.0 |
| pivot_bearings | 6 | 5.0 |
| gate_relay | 10 | 1.5 |
| gate_switch | 5 | 0.5 |
| **TOTAL** | **440** | **104.00** |

**Budget Margin**: 7% remaining versus the planner target.

## 5. Risk Assessment

| Risk | Likelihood | Impact | Mitigation |
| -- | -- | -- | -- |
| The latch leaf rebounds the ball back toward the benchmark guard arm | Medium | High | Keep the latch leaf shallow and bias it toward the right-hand goal pocket |
| Wiring or slack crosses the guard-arm sweep zone | Medium | High | Route all wires through the rear-left corridor and keep the control box behind the bracket |
| The two leaves drift out of sync and leave the goal zone open too long | Low | High | Use the relay and switch to keep the timing path simple and deterministic |
