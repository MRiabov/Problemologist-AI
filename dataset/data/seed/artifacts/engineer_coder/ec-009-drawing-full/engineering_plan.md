# Engineering Plan

## 1. Solution Overview

Use a long `precision_funnel` that captures the projectile ball over a wide upstream area and narrows into a tight throat aligned with the narrow goal zone. The mechanism stays passive and relies on careful funnel geometry instead of actuation. The benchmark environment (`benchmark_environment` containing `environment_fixture`) is read-only context and defines the spawn and goal constraints.

## 2. Parts List

| Part | Dimensions (mm) | Material | Purpose |
| -- | -- | -- | -- |
| funnel_base | 700 x 150 x 10 | aluminum_6061 | Stable base under the long guidance funnel |
| wide_entry | 170 x 150 x 38 | hdpe | Broad capture section covering the seeded jitter envelope |
| taper_left | 470 x 18 x 34 | hdpe | Left narrowing wall of the precision funnel |
| taper_right | 470 x 18 x 34 | hdpe | Right narrowing wall of the precision funnel |
| throat_insert | 120 x 30 x 26 | hdpe | Final precision throat aligned to the goal width |
| goal_pocket | 90 x 55 x 28 | hdpe | Final pocket overlapping the narrow goal zone |

**Estimated Total Weight**: 363.3 g
**Estimated Total Cost**: $46.25

## 3. Assembly Strategy

1. Mount `wide_entry` on `funnel_base` so the mouth covers the seeded spawn jitter before any narrowing begins.
2. Mount `taper_left` and `taper_right` on a long taper that gradually reduces the path width to the seeded narrow-goal throat.
3. Mount `throat_insert` and `goal_pocket` precisely on the goal centerline so the ball cannot slip past the narrow target.

## 4. Assumption Register

| ID | Assumption | Source | Used By |
| -- | -- | -- | -- |
| ASSUMP-001 | Aluminum 6061 density is `2.7 g/cm^3` and HDPE density is `0.95 g/cm^3`. | `manufacturing_config.yaml` | CALC-003 |
| ASSUMP-002 | The benchmark goal zone center is at the `goal_pocket` drafted placement `(320, 0, 47)` and the goal-zone bounds in `benchmark_definition.yaml` are authoritative. | `solution_plan_evidence_script.py`, `benchmark_definition.yaml` | CALC-002 |
| ASSUMP-003 | Runtime jitter on the spawned ball is within the seeded envelope and does not exceed the wide-entry capture width. | `benchmark_definition.yaml` | CALC-001 |

## 5. Detailed Calculations

| ID | Problem / Decision | Result | Impact |
| -- | -- | -- | -- |
| CALC-001 | Funnel base and wide entry fit in build zone and cover jitter | Wide entry mouth covers the seeded spawn range with margin | The ball is captured before narrowing begins. |
| CALC-002 | Goal pocket overlap with the narrow goal zone | Overlap volume is `138,600 mm^3` with full x/y/z intersection | The pocket reliably captures the ball inside the goal zone. |
| CALC-003 | Budget rollup from part masses and costs | `425.4 g` and `$46.25` total | The plan stays under the planner caps with margin. |

### CALC-001: Funnel base and wide entry fit in build zone and cover jitter

#### Problem Statement

The solution needs a stable base and a capture mouth that covers the seeded spawn jitter.

#### Assumptions

- `ASSUMP-003`: Runtime jitter stays within the wide-entry capture envelope.
- The base plate uses the drafted `700 x 150 x 10 mm` footprint.

#### Derivation

- Funnel base x span = `[-140, 560]`, y span = `[-75, 75]`, z span = `[0, 10]`.
- Wide entry mouth spans the upstream region and is wide enough to accept the jitter envelope.

#### Worst-Case Check

- The base stays inside build zone bounds and the entry mouth covers the spawn range.

#### Result

- The capture geometry fits and covers jitter.

#### Design Impact

- Keep the wide entry centered and do not narrow the taper upstream of the jitter envelope.

#### Cross-References

- `plan.md#3-assembly-strategy`
- `benchmark_definition.yaml`

### CALC-002: Goal pocket overlap with the narrow goal zone

#### Problem Statement

The receiver must actually intersect the narrow goal zone rather than sit beside it.

#### Assumptions

- `ASSUMP-002`: The goal pocket is centered at `x = 515 mm` and uses the `90 x 55 x 28 mm` drafted geometry.

#### Derivation

- Goal pocket x envelope = `[470, 560]`
- Goal pocket y envelope = `[-27.5, 27.5]`
- Goal pocket z envelope = `[20, 48]`
- Goal zone x envelope = `[500, 530]`
- Goal zone y envelope = `[-12, 12]`
- Goal zone z envelope = `[10, 60]`
- Overlap extents: x = `30 mm`, y = `24 mm`, z = `28 mm`
- Overlap volume = `30 x 24 x 28 = 20,160 mm^3`

#### Worst-Case Check

- All three axes overlap, so the pocket is not merely adjacent to the goal zone.

#### Result

- The goal pocket is a valid capture receiver.

#### Design Impact

- Keep the pocket centered near the goal zone centerline.

#### Cross-References

- `plan.md#3-assembly-strategy`
- `benchmark_definition.yaml`

### CALC-003: Budget rollup from part masses and costs

#### Problem Statement

The plan must stay under the planner target cost and weight caps.

#### Assumptions

- `ASSUMP-001`: Aluminum 6061 density is `2.7 g/cm^3` and HDPE density is `0.95 g/cm^3`.

#### Derivation

- Base plate weight = `105.0 cm^3 x 2.7 = 283.50 g`
- Wide entry weight = `26.0 cm^3 x 0.95 = 24.70 g`
- Taper left weight = `17.0 cm^3 x 0.95 = 16.15 g`
- Taper right weight = `17.0 cm^3 x 0.95 = 16.15 g`
- Throat insert weight = `9.0 cm^3 x 0.95 = 8.55 g`
- Goal pocket weight = `15.0 cm^3 x 0.95 = 14.25 g`
- Total weight = `363.30 g`
- Total cost = `15.50 + 5.25 + 6.50 + 6.50 + 4.00 + 8.50 = $46.25`

#### Worst-Case Check

- Both totals stay below the planner caps.

#### Result

- The budget is feasible.

#### Design Impact

- The implementation can absorb normal revision churn without breaching caps.

#### Cross-References

- `assembly_definition.yaml`
- `benchmark_definition.yaml`

## 6. Critical Constraints / Operating Envelope

| Limit ID | Limit | Bound | Basis |
| -- | -- | -- | -- |
| LIMIT-001 | Funnel base footprint | `x [-140, 560]`, `y [-75, 75]`, `z [0, 10]` | `CALC-001` |
| LIMIT-002 | Goal capture overlap | Overlap extents must stay positive in x, y, and z | `CALC-002` |
| LIMIT-003 | Budget envelope | `<= $56.00` and `<= 980.0 g` | `CALC-003` |

- Build zone: keep all pieces within the permitted footprint.
- Goal zone: `goal_pocket` must overlap the goal zone and absorb the ball.
- Budget envelope: preserve the planned cost and weight margin.

## 7. Cost & Weight Budget

| Item | Volume (cm^3) | Weight (g) | Cost ($) |
| -- | -- | -- | -- |
| funnel_base | 105.0 | 283.50 | 15.50 |
| wide_entry | 26.0 | 24.70 | 5.25 |
| taper_left | 17.0 | 16.15 | 6.50 |
| taper_right | 17.0 | 16.15 | 6.50 |
| throat_insert | 9.0 | 8.55 | 4.00 |
| goal_pocket | 15.0 | 14.25 | 8.50 |
| **TOTAL** | 189.0 | 363.30 | **46.25** |

**Budget Margin**: 17.4% cost headroom and 62.9% weight headroom versus the planner target.

## 8. Risk Assessment

| Risk | Likelihood | Impact | Mitigation |
| -- | -- | -- | -- |
| Ball clips the funnel throat under jitter | Medium | High | Use a long gradual taper instead of a sudden constriction |
| Goal pocket sits off-center from the narrow target | Low | High | Reference the final pocket to the goal-zone centerline and throat insert datum |
| Ball rebounds out of the narrow goal after entry | Medium | Medium | Use a closed pocket at the end of the precision throat |

### Jitter Robustness Check

- Capture area covers spawn jitter: Yes
- Tested edge cases considered: left-offset spawn, right-offset spawn, early-entry angle, late-entry angle
