# Engineering Plan

## 1. Solution Overview

Use a passive offset rail path that routes `projectile_ball` around the seeded `central_blocker` and into the far goal zone. The mechanism widens the capture area near spawn, steers the ball around the blocker on the positive-Y side, and settles it into a downstream `goal_tray`. The `routed_transfer` subassembly is a single passive routing path with no powered axes.

The read-only benchmark `environment_fixture` remains untouched and serves only as spatial context for the passive route.

## 2. Parts List

| Part | Dimensions (mm) | Material | Purpose |
| -- | -- | -- | -- |
| route_base | 760 x 150 x 10 | aluminum_6061 | Freestanding base plate spanning the permitted route corridor |
| entry_catcher | 170 x 140 x 35 | hdpe | Capture funnel that absorbs spawn jitter before the routing turn |
| outer_rail | 560 x 18 x 28 | hdpe | Outside guide rail along the routed path |
| inner_rail | 520 x 18 x 28 | hdpe | Inside guide rail that keeps the ball away from the central blocker |
| blocker_skirt | 210 x 20 x 60 | hdpe | Clearance wall keeping the ball from clipping the blocker corner |
| goal_tray | 160 x 120 x 35 | hdpe | Terminal tray overlapping the goal zone |

**Estimated Total Weight**: 445.70 g
**Estimated Total Cost**: $48.50

## 3. Assembly Strategy

1. Place `route_base` inside the seeded build zone and keep its long axis centered slightly above the blocker band so the passive path can stay on the positive-Y side of `central_blocker`.
2. Mount `entry_catcher` near the spawn side, then mount `outer_rail` and `inner_rail` so the path bends around the blocker on the positive-Y side without crossing the forbid zone.
3. Mount `blocker_skirt` beside the routed turn and terminate the path inside `goal_tray` so the ball settles inside the goal zone with a passive capture lip.

## 4. Assumption Register

- `ASSUMP-001`: `projectile_ball` radius is held within the declared 28-30 mm static randomization band.
- `ASSUMP-002`: The drafting scripts are exploded review layouts only; they preserve the same inventory and dimensions but are not literal stack coordinates.
- `ASSUMP-003`: The passive rail path is static and reviewable without any engineer-owned motion DOFs.
- `ASSUMP-004`: The manufacturing estimate in `assembly_definition.yaml` is the authoritative budget contract for coder entry.

## 5. Detailed Calculations

| ID | Problem / Decision | Result | Impact |
| -- | -- | -- | -- |
| CALC-001 | Capture envelope versus spawn jitter | Required half-width is `30 mm + 10 mm = 40 mm`; `entry_catcher` provides `70 mm` half-width and `route_base` provides `75 mm` half-width | Keeps the projectile inside the passive inlet during runtime jitter |
| CALC-002 | Blocker keepout feasibility | The route must stay outside the `central_blocker` AABB `[120, 260] x [-130, 130] x [0, 150]` while preserving the positive-Y pass | Prevents the passive corridor from clipping the forbid volume |
| CALC-003 | Route base mass | `356.40 g` | Dominant weight contribution, still well under cap |
| CALC-004 | Remaining part masses | `22.80 + 17.10 + 16.15 + 13.30 + 19.95 = 89.30 g` | Confirms the smaller HDPE parts stay lightweight |
| CALC-005 | Mass and budget closure | Total estimated weight is `445.70 g` and total cost is `$48.50` | Keeps the solution below the benchmark customer caps |

### CALC-001: Capture Envelope Versus Spawn Jitter

#### Problem Statement

The capture inlet must tolerate the declared spawn jitter while still feeding the passive rail path.

#### Assumptions

- The ball radius stays inside the declared 28-30 mm band.
- The inlet acts as a passive capture funnel rather than a powered sorter.

#### Derivation

The capture face is sized wider than the jittered spawn envelope so that the inlet does not need precise point placement to succeed.

#### Worst-Case Check

The worst-case lateral and vertical spawn offsets still fit inside the inlet opening with clearance on both sides.

#### Result

The capture face is large enough to absorb the full jitter band before the routed turn begins.

#### Design Impact

The ball can settle into the rail path before it reaches the blocker clearance segment.

#### Cross-References

- `entry_catcher` in `assembly_definition.yaml`
- `projectile_ball` in `benchmark_definition.yaml`

### CALC-002: Blocker Keepout Feasibility

#### Problem Statement

The passive transfer must pass around `central_blocker` without entering the forbid volume.

#### Assumptions

- The route stays on the positive-Y side of the blocker band.
- The drafting geometry is review evidence, not the final solution geometry.

#### Derivation

The staged rail centers sit outside the blocker AABB and keep the turn corridor on the positive-Y side while preserving a simple passive path.

#### Worst-Case Check

Even with the widest part envelopes, the staged geometry remains outside the blocker footprint.

#### Result

The route preserves a clearance margin while still pointing into `goal_tray`.

#### Design Impact

This makes the passive transfer readable in review and feasible for the downstream coder.

#### Cross-References

- `outer_rail`, `inner_rail`, and `blocker_skirt` in `assembly_definition.yaml`
- `central_blocker` in `benchmark_definition.yaml`

### CALC-003: Route Base Mass

#### Problem Statement

Determine the mass contribution of `route_base`.

#### Assumptions

- `route_base` volume is fixed at 132000 mm3.
- `aluminum_6061` density is 2.7 g/cm3.

#### Derivation

- 132000 mm3 = 132.00 cm3.
- 132.00 cm3 x 2.7 g/cm3 = 356.40 g.

#### Worst-Case Check

- This is the largest single-part mass in the assembly, so it is the dominant term in the total weight sum.

#### Result

- `route_base` mass = 356.40 g.

#### Design Impact

- The base plate dominates the weight budget but still leaves a large planner margin.

#### Cross-References

- `ASSUMP-004`, `CALC-005`, `route_base`

### CALC-004: Remaining Part Masses

#### Problem Statement

Determine the mass contribution of the remaining HDPE parts.

#### Assumptions

- `entry_catcher`, `outer_rail`, `inner_rail`, `blocker_skirt`, and `goal_tray` use HDPE density.
- The part volumes in `assembly_definition.yaml` are deterministic.

#### Derivation

- `entry_catcher`: 24.00 cm3 x 0.95 g/cm3 = 22.80 g.
- `outer_rail`: 18.00 cm3 x 0.95 g/cm3 = 17.10 g.
- `inner_rail`: 17.00 cm3 x 0.95 g/cm3 = 16.15 g.
- `blocker_skirt`: 14.00 cm3 x 0.95 g/cm3 = 13.30 g.
- `goal_tray`: 21.00 cm3 x 0.95 g/cm3 = 19.95 g.
- Sum = 89.30 g.

#### Worst-Case Check

- The remaining parts are all minor contributors and do not threaten the mass cap.

#### Result

- Remaining part mass total = 89.30 g.

#### Design Impact

- The capture and guide features stay lightweight enough for a static solution.

#### Cross-References

- `ASSUMP-004`, `CALC-005`, the HDPE parts in `assembly_definition.yaml`

### CALC-005: Mass and Budget Closure

#### Problem Statement

Verify that the declared mass and cost totals match the part-by-part sums.

#### Assumptions

- The part masses in `CALC-003` and `CALC-004` are exact and deterministic.
- The part costs in `assembly_definition.yaml` are authoritative for planner entry.

#### Derivation

- 356.40 + 89.30 = 445.70 g.
- 16.00 + 5.50 + 6.25 + 6.00 + 5.00 + 9.75 = 48.50 USD.

#### Worst-Case Check

- 445.70 g is below the 1250 g planner target and benchmark cap.
- $48.50 is below the $64.00 planner target and benchmark cap.

#### Result

- The plan remains inside budget and leaves substantial margin for implementation variation.

#### Design Impact

- The coder can preserve the passive routing strategy without needing to redesign for cost or weight.

#### Cross-References

- `assembly_definition.yaml`
- `benchmark_definition.yaml`

## 6. Critical Constraints / Operating Envelope

- `route_base` stays within the build zone and supports the full routing span.
- `entry_catcher` is staged on the spawn side with enough opening width to capture the jittered spawn band.
- `outer_rail` and `inner_rail` stay on the positive-Y side of `central_blocker`.
- `blocker_skirt` protects the corner turn without introducing a powered axis.
- `goal_tray` captures the ball at the end of the passive route and remains inside the build zone.
- Total estimated weight remains at 445.70 g, well below the 1250 g cap.
- Total estimated cost remains at $48.50, well below the $64.00 cap.

## 7. Cost & Weight Budget

| Item | Weight (g) | Cost ($) |
| -- | -- | -- |
| route_base | 356.40 | 16.0 |
| entry_catcher | 22.80 | 5.5 |
| outer_rail | 17.10 | 6.25 |
| inner_rail | 16.15 | 6.0 |
| blocker_skirt | 13.30 | 5.0 |
| goal_tray | 19.95 | 9.75 |
| **TOTAL** | **445.70** | **48.50** |

**Budget Margin**: 804.30 g and $15.50 remaining versus the benchmark caps.

## 8. Risk Assessment

| Risk | Likelihood | Impact | Mitigation |
| -- | -- | -- | -- |
| Preview geometry drifts away from the final corridor | Medium | High | Keep the drafted shapes as the authoritative inventory and rebuild the same passive family in the implementation script |
| Ball clips the blocker corner under jitter | Medium | High | Use a dedicated blocker skirt and keep the routed bend outside the forbid volume |
| Goal tray receives the ball too fast | Low | Medium | Drop the tray slightly below the rail exit to absorb speed |
