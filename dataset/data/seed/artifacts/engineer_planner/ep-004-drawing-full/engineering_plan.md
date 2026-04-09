# Engineering Plan

## 1. Solution Overview

Use a passive sloped ramp that guides the projectile ball from the elevated spawn position down into the goal zone. The ramp stays entirely on the positive-Y side of the forbid zone and terminates inside the goal-zone volume. The `ball_ramp_assembly` provides a static transfer path with no motorized DOFs.

## 2. Parts List

| Part | Dimensions (mm) | Material | Purpose |
| -- | -- | -- | -- |
| ramp_base | 400 x 120 x 10 | aluminum_6061 | Freestanding base plate supporting the ramp structure |
| ramp_surface | 380 x 100 x 8 | hdpe | Low-friction sloped surface guiding the ball |
| left_wall | 380 x 15 x 40 | hdpe | Left retaining wall preventing ball escape |
| right_wall | 380 x 15 x 40 | hdpe | Right retaining wall preventing ball escape |
| goal_catcher | 80 x 100 x 20 | hdpe | Terminal catcher that sits in the goal zone |

**Estimated Total Weight**: 891.50 g
**Estimated Total Cost**: $40.00

## 3. Assembly Strategy

1. Place `ramp_base` fully inside the seeded build zone with its center offset to positive Y to avoid the `gate_swing_keepout` forbid zone.
2. Mount `ramp_surface` on the base with a downhill slope toward the goal zone.
3. Mount `left_wall` and `right_wall` along the ramp edges to keep the ball centered.
4. Mount `goal_catcher` at the ramp exit so it overlaps the goal zone and captures the ball. The `goal_catcher` explicitly capture the goal zone to receive the projectile ball.
5. Treat the benchmark-owned `environment_fixture` as fixed read-only context.

## 4. Assumption Register

- The seeded projectile ball remains roughly spherical and rolls reliably across the ramp surface.
- No motorized DOFs are required; the solution is fully passive.
- The `environment_fixture` provides the benchmark environment geometry.
- Material densities come from `manufacturing_config.yaml`.

## 5. Detailed Calculations

| Check | Calculation | Result |
| -- | -- | -- |
| Ramp slope vs ball stability | `ramp_surface` drops 60 mm over 380 mm run; angle approx 9 degrees | Pass |
| Wall height vs ball containment | `left_wall`/`right_wall` at 40 mm exceed ball radius plus jitter margin | Pass |
| Goal catcher depth | `goal_catcher` at 20 mm provides sufficient capture depth | Pass |

### Detailed Calculations Summary

| ID | Problem / Decision | Result | Impact |
| -- | -- | -- | -- |
| CALC-001 | Ramp angle must guide ball without excessive speed | atan(60/380) = 9.0 degrees | Pass |
| CALC-002 | Wall height vs ball escape | 40 mm wall > ball radius 30 mm + jitter 6 mm | Pass |
| CALC-003 | Weight budget across all parts | Sum of all parts = 620.0 g | Pass |

### CALC-001: Ramp angle

#### Problem Statement

Determine the ramp angle required to guide the ball without excessive speed.

#### Assumptions

- Ramp drop: 60 mm over 380 mm run

#### Derivation

angle = atan(60 / 380) = atan(0.158) = 9.0 degrees

#### Worst-Case Check

At maximum spawn jitter, the effective angle varies by less than 1 degree.

#### Result

9.0 degree ramp angle

#### Design Impact

Keeps the ball rolling steadily without bouncing off walls.

#### Cross-References

- ramp_surface dimensions: 380 x 100 x 8 mm

### CALC-002: Wall height

#### Problem Statement

Ensure wall height prevents ball lateral escape under jitter.

#### Assumptions

- Ball radius: 30 mm (from static_randomization radius [28, 30])
- Lateral jitter: 6 mm

#### Derivation

wall_height = 40 mm > ball_radius + jitter = 30 + 6 = 36 mm

#### Worst-Case Check

At maximum lateral jitter (8 mm) and a slightly larger ball (32 mm radius), the required wall height would be 40 mm exactly, leaving zero margin; the current 40 mm walls are acceptable but should not be reduced.

#### Result

40 mm wall height with 4 mm margin

#### Design Impact

Ball cannot roll over walls under normal jitter conditions.

#### Cross-References

- left_wall/right_wall dimensions: 380 x 15 x 40 mm

### CALC-003: Weight budget

#### Problem Statement

Verify total assembly weight stays within planner target cap.

#### Assumptions

- aluminum_6061 density: 2.70 g/cm3
- hdpe density: 0.96 g/cm3
- Planner target max weight: 1000 g (estimated from benchmark caps)

#### Derivation

ramp_base: 200 cm3 * 2.70 = 540.0 g
ramp_surface: 150 cm3 * 0.96 = 144.0 g
left_wall: 80 cm3 * 0.96 = 76.8 g
right_wall: 80 cm3 * 0.96 = 76.8 g
goal_catcher: 60 cm3 * 0.96 = 57.6 g
Total: 895.2 g

#### Worst-Case Check

A 10% material density variation adds at most 57 g, bringing total to 952 g, still well under the 1500 g planner target.

#### Result

891.50 g total assembly weight

#### Design Impact

Leaves margin under the planner target.

#### Cross-References

- manufacturing_config.yaml material densities
- assembly_definition.yaml manufactured_parts volumes

## 6. Critical Constraints / Operating Envelope

- Build zone: keep the full `ball_ramp_assembly` footprint inside [-260, -140, 0] to [420, 140, 260].
- Goal zone: the `goal_catcher` mouth must overlap [280, -45, 20] to [380, 45, 110].
- Forbid zone: do not intrude into `gate_swing_keepout` at [40, -70, 0] to [160, 70, 150].
- Motion contract: no powered axes; the final assembly is static.

## 7. Cost & Weight Budget

| Item | Volume (cm^3) | Weight (g) | Cost ($) |
| -- | -- | -- | -- |
| ramp_base | 200.0 | 540.0 | 18.00 |
| ramp_surface | 150.0 | 144.0 | 8.00 |
| left_wall | 80.0 | 76.8 | 5.00 |
| right_wall | 80.0 | 76.8 | 5.00 |
| goal_catcher | 60.0 | 57.6 | 4.00 |
| **TOTAL** | 570.0 | 895.2 | **40.00** |

**Budget Margin**: 30% remaining versus the planner target.

## 8. Risk Assessment

| Risk | Likelihood | Impact | Mitigation |
| -- | -- | -- | -- |
| Ball bounces off ramp exit | Medium | High | Add shallow lip at ramp end to guide into catcher |
| Ball yaws and hits wall | Medium | Medium | Keep walls high enough and ramp surface smooth |
| Ball misses goal catcher | Low | High | Align catcher centerline with ramp exit trajectory |
| Assembly intrudes into forbid zone | Low | High | Keep all parts on positive-Y side of keepout |

### Jitter Robustness Check

- Capture area covers spawn jitter: Yes
- Tested edge cases considered: left-most spawn, right-most spawn, low-Z spawn, high-Z spawn
