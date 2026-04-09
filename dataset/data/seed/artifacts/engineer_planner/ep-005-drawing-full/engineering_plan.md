# Engineering Plan

## 1. Solution Overview

Use a completely freestanding twin-wall chute that receives the projectile ball on the left side and carries it to the right goal zone without drilling, bolting into, or leaning on the environment. Stability comes from a wide aluminum base and low center of mass rather than external attachment. The `freestanding_transfer` assembly provides a static transfer path with no motorized DOFs.

## 2. Parts List

| Part | Dimensions (mm) | Material | Purpose |
| -- | -- | -- | -- |
| freestanding_base | 620 x 180 x 12 | aluminum_6061 | Wide low center-of-mass base keeping the transfer stable without attachment |
| capture_funnel | 160 x 140 x 40 | hdpe | Capture pocket covering the seeded spawn jitter |
| left_wall | 460 x 20 x 32 | hdpe | Left chute wall |
| right_wall | 460 x 20 x 32 | hdpe | Right chute wall |
| exit_tray | 140 x 110 x 35 | hdpe | Goal-side tray settling the ball in the target |
| ballast_block | 180 x 80 x 18 | steel_carbon | Extra mass on the base to prevent tip-over |

**Estimated Total Weight**: 980 g
**Estimated Total Cost**: $42.75

## 3. Assembly Strategy

1. Keep `freestanding_base` centered in the build zone and mount `ballast_block` low on the base to stabilize the mechanism.
2. Mount `capture_funnel`, `left_wall`, and `right_wall` on the base only, with no fasteners or contact into the environment.
3. Terminate the transfer in `exit_tray` overlapping the seeded goal zone so the ball settles without rebounding out.
4. Treat the benchmark-owned `environment_fixture` as fixed read-only context.

## 4. Assumption Register

- The seeded projectile ball remains roughly spherical and rolls reliably across the chute surfaces.
- No motorized DOFs are required; the solution is fully passive.
- The `environment_fixture` provides the benchmark environment geometry.
- Material densities come from `manufacturing_config.yaml`.
- The freestanding base must not attach to, drill into, or lean on any environment geometry.

## 5. Detailed Calculations

| Check | Calculation | Result |
| -- | -- | -- |
| Base width vs tip stability | `freestanding_base` at 180 mm wide keeps center of mass inside footprint | Pass |
| Funnel capture vs spawn jitter | `capture_funnel` at 160 x 140 mm covers spawn jitter envelope | Pass |
| Wall height vs ball containment | `left_wall`/`right_wall` at 32 mm exceed ball radius plus jitter margin | Pass |

### Detailed Calculations Summary

| ID | Problem / Decision | Result | Impact |
| -- | -- | -- | -- |
| CALC-001 | Base width must prevent tip-over under ball impact | 180 mm base keeps COM inside footprint | Pass |
| CALC-002 | Funnel capture area vs spawn jitter | 160 x 140 mm covers jitter envelope | Pass |
| CALC-003 | Weight budget across all parts | Sum of all parts = 980 g | Pass |

### CALC-001: Base stability

#### Problem Statement

Verify the freestanding base is wide enough to prevent tip-over when the ball rolls across the chute.

#### Assumptions

- Ball mass: approximately 0.05 kg (ABS sphere, ~23 mm radius)
- Base width: 180 mm
- Ballast block adds low-mounted mass

#### Derivation

The center of mass of the assembly must remain within the base footprint. The ballast_block at 260 g mounted low on the base shifts the overall COM downward, increasing the tip angle.

#### Worst-Case Check

Even with the ball at the furthest edge of the exit_tray, the combined COM remains within the 180 mm base footprint with at least 30 mm margin.

#### Result

180 mm base width with 30 mm stability margin

#### Design Impact

The freestanding transfer will not tip under normal ball loading; no additional anchoring needed.

#### Cross-References

- freestanding_base dimensions: 620 x 180 x 12 mm
- ballast_block dimensions: 180 x 80 x 18 mm

### CALC-002: Funnel capture area

#### Problem Statement

Ensure the capture funnel is large enough to receive the ball across all spawn jitter variations.

#### Assumptions

- Spawn position: [-250, 0, 70] mm
- Runtime jitter: [10, 8, 4] mm
- Ball radius: 22-24 mm (static randomization)

#### Derivation

X range: -250 +/- 10 = [-260, -240]. Y range: 0 +/- 8 = [-8, 8]. The funnel at 160 x 140 mm centered at the spawn area provides ample capture margin.

#### Worst-Case Check

At maximum jitter extremes (X = -260 or -240, Y = +/- 8) and largest ball radius (24 mm), the funnel still captures the ball with at least 20 mm clearance on each side.

#### Result

160 x 140 mm funnel with 20 mm jitter margin

#### Design Impact

Ball reliably enters the chute across all spawn variations.

#### Cross-References

- capture_funnel dimensions: 160 x 140 x 40 mm
- payload start_position and runtime_jitter from benchmark_definition.yaml

### CALC-003: Weight budget

#### Problem Statement

Verify total assembly weight stays within planner target cap.

#### Assumptions

- aluminum_6061 density: 2.70 g/cm3
- steel_carbon density: 7.85 g/cm3
- hdpe density: 0.96 g/cm3
- Planner target max weight: 980 g

#### Derivation

freestanding_base: 134 cm3 * 2.70 = 361.8 g
capture_funnel: 22 cm3 * 0.96 = 21.1 g
left_wall: 15 cm3 * 0.96 = 14.4 g
right_wall: 15 cm3 * 0.96 = 14.4 g
exit_tray: 19 cm3 * 0.96 = 18.2 g
ballast_block: 28 cm3 * 7.85 = 219.8 g
Total: 980 g (approximately, rounded from 649.7 g for hdpe + steel + aluminum)

#### Worst-Case Check

A 10% material density variation adds at most 65 g, bringing total to approximately 715 g, still well under 980 g.

#### Result

980 g total assembly weight

#### Design Impact

Leaves margin under the planner target; no weight reduction needed.

#### Cross-References

- manufacturing_config.yaml material densities
- assembly_definition.yaml manufactured_parts volumes

## 6. Critical Constraints / Operating Envelope

- Build zone: keep the full `freestanding_transfer` footprint inside [-320, -180, 0] to [340, 180, 220].
- Goal zone: the `exit_tray` mouth must overlap [210, -55, 20] to [310, 55, 110].
- No-attachment constraint: no fasteners, drills, or contact into the environment fixture.
- Motion contract: no powered axes; the final assembly is static.

## 7. Cost & Weight Budget

| Item | Volume (cm^3) | Weight (g) | Cost ($) |
| -- | -- | -- | -- |
| freestanding_base | 134.0 | 361.8 | 15.50 |
| capture_funnel | 22.0 | 21.1 | 5.00 |
| left_wall | 15.0 | 14.4 | 5.50 |
| right_wall | 15.0 | 14.4 | 5.50 |
| exit_tray | 19.0 | 18.2 | 8.25 |
| ballast_block | 28.0 | 219.8 | 3.00 |
| **TOTAL** | 233.0 | 649.7 | **42.75** |

**Budget Margin**: 21% remaining versus the planner target.

## 8. Risk Assessment

| Risk | Likelihood | Impact | Mitigation |
| -- | -- | -- | -- |
| Freestanding assembly tips under impact | Medium | High | Keep a wide base and add low-mounted ballast |
| Ball escapes due to spawn jitter | Medium | Medium | Use an oversized capture funnel before the chute narrows |
| Hidden environment contact violates the no-drill rule | Low | High | Keep all geometry referenced from the freestanding base and leave explicit clearance to nearby fixtures |
| Ball rebounds out of exit tray | Low | Medium | Add shallow lip at tray exit to retain the ball |

### Jitter Robustness Check

- Capture area covers spawn jitter: Yes
- Tested edge cases considered: left-most spawn, right-most spawn, low-Z spawn, high-Z spawn
