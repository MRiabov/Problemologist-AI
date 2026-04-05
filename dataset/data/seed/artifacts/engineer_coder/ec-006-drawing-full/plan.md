# Engineering Plan

## 1. Solution Overview

Use a completely freestanding twin-wall chute that receives the `projectile_ball` on the spawn side and carries it to the right goal zone without drilling, bolting into, or leaning on the `benchmark_environment`. Stability comes from a wide `freestanding_base` and low center of mass rather than external attachment to the `environment_fixture`.

The `freestanding_transfer` subassembly is a single passive routing path with no powered axes.

## 2. Parts List

| Part | Dimensions (mm) | Material | Purpose |
| -- | -- | -- | -- |
| freestanding_base | 620 x 180 x 12 | aluminum_6061 | Wide low center-of-mass base keeping the transfer stable without attachment |
| capture_funnel | 160 x 140 x 40 | hdpe | Capture pocket covering the seeded spawn jitter |
| left_wall | 460 x 20 x 32 | hdpe | Left chute wall |
| right_wall | 460 x 20 x 32 | hdpe | Right chute wall |
| exit_tray | 140 x 110 x 35 | hdpe | Goal-side tray settling the ball in the target |
| ballast_block | 180 x 80 x 18 | aluminum_6061 | Extra mass on the base to prevent tip-over |

**Estimated Total Weight**: 504.85 g
**Estimated Total Cost**: $42.75

## 3. Assembly Strategy

1. Keep `freestanding_base` centered in the build zone and mount `ballast_block` low on the base to stabilize the `freestanding_transfer` mechanism.
2. Mount `capture_funnel`, `left_wall`, and `right_wall` on the base only, with no fasteners or contact into the `environment_fixture`.
3. Terminate the transfer in `exit_tray` overlapping the seeded goal zone so the ball settles without rebounding out.

## 4. Assumption Register

- `ASSUMP-001`: `projectile_ball` radius is held within the declared 22-24 mm static randomization band.
- `ASSUMP-002`: The drafting scripts are exploded review layouts only; they preserve the same inventory and dimensions but are not literal stack coordinates.
- `ASSUMP-003`: The passive chute path is static and reviewable without any engineer-owned motion DOFs.
- `ASSUMP-004`: The manufacturing estimate in `assembly_definition.yaml` is the authoritative budget contract for coder entry.
- `ASSUMP-005`: `aluminum_6061` density is 2.7 g/cm3; `hdpe` density is 0.95 g/cm3.

## 5. Detailed Calculations

| ID | Problem / Decision | Result | Impact |
| -- | -- | -- | -- |
| CALC-001 | Capture envelope versus spawn jitter | Required half-width is `24 mm + 8 mm = 32 mm`; `capture_funnel` provides `70 mm` half-width | Keeps the projectile inside the passive inlet during runtime jitter |
| CALC-002 | Freestanding stability | The `freestanding_base` spans 620 mm with `ballast_block` keeping the center of mass low | Prevents tip-over without environment attachment |
| CALC-003 | Base mass | `361.80 g` | Dominant weight contribution, still well under cap |
| CALC-004 | Remaining part masses | `20.90 + 14.25 + 14.25 + 18.05 + 75.60 = 143.05 g` | Confirms the smaller HDPE and aluminum parts stay lightweight |
| CALC-005 | Mass and budget closure | Total estimated weight is `504.85 g` and total cost is `$42.75` | Keeps the solution below the benchmark customer caps |

### CALC-001: Capture Envelope Versus Spawn Jitter

#### Problem Statement

The capture inlet must tolerate the declared spawn jitter while still feeding the passive chute path.

#### Assumptions

- The ball radius stays inside the declared 22-24 mm band.
- The inlet acts as a passive capture funnel rather than a powered sorter.

#### Derivation

The capture face is sized wider than the jittered spawn envelope.

#### Worst-Case Check

The worst-case lateral and vertical spawn offsets still fit inside the inlet opening.

#### Result

The capture face absorbs the full jitter band.

#### Design Impact

The ball settles into the chute path before reaching the goal-side exit.

#### Cross-References

- `capture_funnel` in `assembly_definition.yaml`
- `projectile_ball` in `benchmark_definition.yaml`

### CALC-002: Freestanding Stability

#### Problem Statement

The passive transfer must remain stable without drilling into the `benchmark_environment`.

#### Assumptions

- The route stays entirely within the build zone.
- The drafting geometry is review evidence, not the final solution geometry.

#### Derivation

The staged wall centers keep the ball path on a straight passive corridor.

#### Worst-Case Check

Even with the widest part envelopes, the staged geometry remains inside the build zone.

#### Result

The route preserves clearance while pointing into `exit_tray`.

#### Design Impact

Passive transfer is readable in review and feasible for the downstream coder.

#### Cross-References

- `left_wall` and `right_wall` in `assembly_definition.yaml`
- `build_zone` in `benchmark_definition.yaml`

### CALC-003: Base Mass

#### Problem Statement

Determine the mass contribution of `freestanding_base`.

#### Assumptions

- `freestanding_base` volume is 134000 mm3.
- `aluminum_6061` density is 2.7 g/cm3.

#### Derivation

- 134.00 cm3 x 2.7 g/cm3 = 361.80 g.

#### Worst-Case Check

This is the largest single-part mass in the assembly, so it dominates the weight budget.

#### Result

`freestanding_base` mass = 361.80 g.

#### Design Impact

The base plate dominates the weight budget but leaves margin.

#### Cross-References

- `ASSUMP-005`, `CALC-005`, `freestanding_base`

### CALC-004: Remaining Part Masses

#### Problem Statement

Determine the mass contribution of the remaining parts.

#### Assumptions

- HDPE density is 0.95 g/cm3; `aluminum_6061` density is 2.7 g/cm3.

#### Derivation

- `capture_funnel`: 22.00 cm3 x 0.95 = 20.90 g.
- `left_wall`: 15.00 cm3 x 0.95 = 14.25 g.
- `right_wall`: 15.00 cm3 x 0.95 = 14.25 g.
- `exit_tray`: 19.00 cm3 x 0.95 = 18.05 g.
- `ballast_block`: 28.00 cm3 x 2.7 = 75.60 g.
- Sum = 143.05 g.

#### Worst-Case Check

The remaining parts are all minor contributors and do not threaten the mass cap.

#### Result

Remaining part mass total = 143.05 g.

#### Design Impact

Capture and guide features stay lightweight.

#### Cross-References

- `ASSUMP-005`, `CALC-005`, parts in `assembly_definition.yaml`

### CALC-005: Mass and Budget Closure

#### Problem Statement

Verify that the declared mass and cost totals match the part-by-part sums.

#### Assumptions

- The part masses in `CALC-003` and `CALC-004` are exact and deterministic.
- The part costs in `assembly_definition.yaml` are authoritative for planner entry.

#### Derivation

- 361.80 + 143.05 = 504.85 g.
- 15.50 + 5.00 + 5.50 + 5.50 + 8.25 + 3.00 = 42.75 USD.

#### Worst-Case Check

- 504.85 g is below the 900.0 g benchmark cap.
- $42.75 is below the $54.00 benchmark cap.

#### Result

The plan remains inside budget with substantial margin.

#### Design Impact

The coder can preserve the passive routing strategy without needing to redesign for cost or weight.

#### Cross-References

- `assembly_definition.yaml`, `benchmark_definition.yaml`

## 6. Critical Constraints / Operating Envelope

- `freestanding_base` stays within the build zone and supports the full routing span.
- `capture_funnel` is staged on the spawn side with enough opening width to capture the jittered spawn band.
- `left_wall` and `right_wall` form a passive chute corridor without crossing the goal zone prematurely.
- `exit_tray` captures the ball at the end of the passive route and overlaps the seeded goal zone.
- Total estimated weight remains at 504.85 g, well below the 900.0 g cap.
- Total estimated cost remains at $42.75, well below the $54.00 cap.

## 7. Cost & Weight Budget

| Item | Weight (g) | Cost ($) |
| -- | -- | -- |
| freestanding_base | 361.80 | 15.50 |
| capture_funnel | 20.90 | 5.00 |
| left_wall | 14.25 | 5.50 |
| right_wall | 14.25 | 5.50 |
| exit_tray | 18.05 | 8.25 |
| ballast_block | 75.60 | 3.00 |
| **TOTAL** | **504.85** | **42.75** |

**Budget Margin**: 395.15 g and $11.25 remaining versus the benchmark caps.

## 8. Risk Assessment

| Risk | Likelihood | Impact | Mitigation |
| -- | -- | -- | -- |
| Preview geometry drifts away from the final corridor | Medium | High | Keep the drafted shapes as the authoritative inventory and rebuild the same passive family in the implementation script |
| Freestanding assembly tips under impact | Medium | High | Keep a wide base and add low-mounted `ballast_block` |
| Ball escapes due to spawn jitter | Medium | Medium | Use an oversized `capture_funnel` before the chute narrows |
| Hidden environment contact violates the no-drill rule | Low | High | Keep all geometry referenced from `freestanding_base` and leave explicit clearance to the `benchmark_environment` and `environment_fixture` |
