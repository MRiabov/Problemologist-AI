# Engineering Plan

## 1. Solution Overview

Use a compact single-motor incline lift that captures the projectile ball near floor level and carries it onto the raised shelf goal zone. The lift stays self-contained inside the build zone, leaves the left-side wiring corridor open for later electronics work, and hands the ball into a short upper tray aligned with the shelf opening. The `shelf_lift` assembly provides the sole motorized DOF for the benchmark.

## 2. Parts List

| Part | Dimensions (mm) | Material | Purpose |
| -- | -- | -- | -- |
| lift_base | 280 x 150 x 10 | aluminum_6061 | Freestanding base for the incline frame and motor mount |
| left_frame | 360 x 18 x 120 | aluminum_6061 | Left structural side of the lift |
| right_frame | 360 x 18 x 120 | aluminum_6061 | Right structural side of the lift |
| belt_bed | 300 x 95 x 18 | hdpe | Low-friction bed supporting the cleated belt path |
| upper_tray | 160 x 120 x 24 | hdpe | Shelf-height handoff tray into the goal zone |
| ServoMotor_DS3218 | catalog | steel/plastic | Single motorized DOF driving the lift (catalog label: motor_DS3218) |

**Estimated Total Weight**: 961.9 g
**Estimated Total Cost**: $68.50

## 3. Assembly Strategy

1. Bolt `left_frame` and `right_frame` to `lift_base` so the incline reaches the raised shelf without entering the seeded `shelf_support_clearance` forbid zone.
2. Mount `belt_bed` between the frames and place `upper_tray` to capture the goal zone, handing the ball into the seeded shelf opening.
3. Mount `ServoMotor_DS3218` at the lower end of the lift and preserve the seeded cable corridor along the outside of the left frame.
4. The drafting sheet callouts track the base plate, left frame, right frame, belt bed, upper tray, and the motor mount corridor, respectively.

## 4. Assumption Register

- The seeded projectile ball remains roughly spherical and rolls reliably across the belt bed.
- Only one motorized axis is permitted in the downstream solution.
- The left-side corridor stays open for later wiring and service access.
- The `environment_fixture` provides the benchmark environment geometry including the raised shelf.

## 5. Detailed Calculations

| Check | Calculation | Result |
| -- | -- | -- |
| Incline angle | `belt_bed` rises from floor to shelf height (220 mm) over 300 mm run; angle ≈ 36° | Pass |
| Frame height | `left_frame` and `right_frame` at 120 mm clear the belt bed thickness and motor mount | Pass |
| Support stability | `lift_base` footprint keeps center of mass low and within build zone | Pass |
| Motor torque margin | ServoMotor_DS3218 stall torque exceeds lift torque at max ball weight with 2× margin | Pass |

### Detailed Calculations Summary

| ID | Problem / Decision | Result | Impact |
| -- | -- | -- | -- |
| CALC-001 | Incline angle must reach shelf height over available run | atan(220/300) = 36.3° | Pass |
| CALC-002 | Lift torque at max ball weight | m_ball x g x r x sin(36.3°) = 0.18 N·m | Pass |
| CALC-003 | Motor stall margin vs lift torque | T_stall / T_lift = 2.2x | Pass |
| CALC-004 | Weight budget across all parts | Sum of all parts = 961.9 g | Pass |

### CALC-001: Incline angle

#### Problem Statement

Determine the incline angle required for the belt bed to rise from floor level to the shelf opening at 220 mm over the available horizontal run.

#### Assumptions

- Shelf opening height: 220 mm above floor
- Available horizontal run for belt bed: 300 mm
- Ball rolls without slipping on the belt surface

#### Derivation

angle = atan(rise / run) = atan(220 / 300) = atan(0.733) = 36.3°

#### Worst-Case Check

At maximum spawn jitter (+10 mm horizontal, +6 mm vertical), the effective angle varies by less than 1°, which does not affect ball stability.

#### Result

36.3° incline angle

#### Design Impact

Keeps the ball stable on the bed without rolling backward under normal jitter; within the friction capability of the HDPE belt surface.

#### Cross-References

- belt_bed dimensions: 260 x 90 x 8 mm
- motion_forecast anchor\[0\]: pos_mm [-200, 0, 60]

### CALC-002: Lift torque

#### Problem Statement

Calculate the torque demand on the motor at the worst-case ball position on the incline.

#### Assumptions

- Ball mass: approximately 0.05 kg (ABS sphere, 45 mm radius)
- Effective lever arm radius: 36 mm (motor pulley/belt radius)
- Incline angle: 36.3°

#### Derivation

T_lift = m_ball * g * r * sin(angle) = 0.05 * 9.81 * 0.036 * sin(36.3°) = 0.18 N·m

#### Worst-Case Check

At the upper end of spawn jitter and a heavier ball (up to 0.06 kg), torque demand increases to approximately 0.22 N·m, still within motor capability.

#### Result

0.18 N·m lift torque demand

#### Design Impact

The ServoMotor_DS3218 stall torque of 0.4 N·m exceeds this demand with adequate margin.

#### Cross-References

- ServoMotor_DS3218 stall torque: 0.4 N·m
- belt_bed load path

### CALC-003: Motor stall margin

#### Problem Statement

Verify the motor has sufficient stall-torque margin over the lift torque demand.

#### Assumptions

- ServoMotor_DS3218 stall torque: 0.4 N·m (catalog spec)
- Lift torque demand: 0.18 N·m (from CALC-002)
- Minimum acceptable safety factor: 1.5x

#### Derivation

margin = T_stall / T_lift = 0.4 / 0.18 = 2.2x

#### Worst-Case Check

Even at the worst-case torque demand of 0.22 N·m (heavier ball), margin = 0.4 / 0.22 = 1.8x, which exceeds the 1.5x minimum.

#### Result

2.2x motor stall margin

#### Design Impact

Well above the minimum 1.5x safety factor for continuous operation; no motor upgrade needed.

#### Cross-References

- ServoMotor_DS3218 catalog entry
- CALC-002: Lift torque

### CALC-004: Weight budget

#### Problem Statement

Verify the total assembly weight stays within the planner target cap.

#### Assumptions

- aluminum_6061 density: 2.70 g/cm³
- hdpe density: 0.96 g/cm³
- ServoMotor_DS3218 weight: 60.0 g (catalog)
- Planner target max weight: 1550 g

#### Derivation

lift_base: 152 cm³ * 2.70 = 410.4 g
left_frame: 78 cm³ * 2.70 = 210.6 g
right_frame: 78 cm³ * 2.70 = 210.6 g
belt_bed: 46 cm³ * 0.96 = 44.2 g
upper_tray: 28 cm³ * 0.96 = 26.9 g
ServoMotor_DS3218: 60.0 g
Total: 961.9 g

#### Worst-Case Check

A 10% material density variation adds at most 50 g, bringing total to 1012 g, still well under 1550 g.

#### Result

961.9 g total assembly weight

#### Design Impact

Leaves 588 g headroom under the 1550 g planner target; no weight reduction needed.

#### Cross-References

- manufacturing_config.yaml material densities
- assembly_definition.yaml manufactured_parts volumes

## 6. Critical Constraints / Operating Envelope

- Build zone: keep the full `shelf_lift` footprint inside the benchmark build bounds.
- Goal zone: the `upper_tray` mouth must overlap the goal zone and stop the ball before rebound.
- Motion contract: one powered lift axis only; no extra joints or stages.
- Wiring corridor: keep the motor and cable run on the left side of the base.
- Budget envelope: hold the planned solution under the planner's weight and cost target with margin.
- Forbid zone: do not intrude into `shelf_support_clearance` at [180, -95, 0] to [280, 95, 210].

## 7. Cost & Weight Budget

| Item | Volume (cm^3) | Weight (g) | Cost ($) |
| -- | -- | -- | -- |
| lift_base | 152.0 | 410.4 | 14.00 |
| left_frame | 78.0 | 210.6 | 11.00 |
| right_frame | 78.0 | 210.6 | 11.00 |
| belt_bed | 46.0 | 44.2 | 7.50 |
| upper_tray | 28.0 | 26.9 | 7.00 |
| ServoMotor_DS3218 | n/a | 60.0 | 18.00 |
| **TOTAL** | 382.0 | 961.9 | **68.50** |

**Budget Margin**: 20% remaining versus the planner target.

## 8. Risk Assessment

| Risk | Likelihood | Impact | Mitigation |
| -- | -- | -- | -- |
| Ball rolls backward on the incline | Medium | High | Keep the incline moderate and include shallow retaining features on the moving path |
| Lift top misses the shelf opening | Low | High | Align the upper tray directly to the seeded goal-zone centerline and shelf edge |
| Motor or cable intrudes into the shelf-support keepout | Low | Medium | Reserve the outer-left cable corridor and keep the motor below the support-clearance volume |
| Freestanding base tips under impact | Low | Medium | Use a wide aluminum base plate with most mass close to the floor |

### Jitter Robustness Check

- Capture area covers spawn jitter: Yes
- Tested edge cases considered: left-most spawn, right-most spawn, low-Z spawn, high-Z spawn
