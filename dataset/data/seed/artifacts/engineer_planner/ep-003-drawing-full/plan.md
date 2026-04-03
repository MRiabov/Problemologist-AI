# Engineering Plan

## 1. Solution Overview

Use a passive offset rail path that routes the projectile ball around the seeded central blocker and into the far goal zone. The mechanism widens the capture area near spawn, steers the ball around the blocker on the positive-Y side, and settles it into a downstream goal tray.

The approved final assembly is `routed_transfer`.

## 2. Parts List

| Part | Dimensions (mm) | Material | Purpose |
| -- | -- | -- | -- |
| `route_base` | 760 x 150 x 10 | `aluminum_6061` | Freestanding base plate spanning the permitted route corridor |
| `entry_catcher` | 170 x 140 x 35 | `hdpe` | Capture funnel that absorbs spawn jitter before the routing turn |
| `outer_rail` | 560 x 18 x 28 | `hdpe` | Outside guide rail along the routed path |
| `inner_rail` | 520 x 18 x 28 | `hdpe` | Inside guide rail that keeps the ball away from the central blocker |
| `blocker_skirt` | 210 x 20 x 60 | `hdpe` | Clearance wall keeping the ball from clipping the blocker corner |
| `goal_tray` | 160 x 120 x 35 | `hdpe` | Terminal tray overlapping the goal zone |

**Estimated Total Weight**: 445.70 g
**Estimated Total Cost**: $48.50

## 3. Assembly Strategy

1. Place `route_base` fully inside the seeded build zone and keep its mid-span clear of the blocker keepout.
2. Mount `entry_catcher` near the spawn side, then mount `outer_rail` and `inner_rail` so the path bends around the blocker on one side.
3. Mount `blocker_skirt` beside the routed turn and terminate the path inside `goal_tray` so the ball settles inside the goal zone.
4. Treat the benchmark-owned `environment_fixture` as fixed read-only context; it is part of the seeded benchmark handoff, not an engineer-owned deliverable.

## 4. Assumption Register

| ID | Assumption | Source / Use |
| -- | -- | -- |
| `ASSUMP-001` | The benchmark-owned `environment_fixture` remains fixed and is only read as context. | `benchmark_assembly_definition.yaml` / `benchmark_definition.yaml` |
| `ASSUMP-002` | Material densities come directly from `manufacturing_config.yaml`. | Weight calculations below |
| `ASSUMP-003` | The ball radius and runtime jitter in `benchmark_definition.yaml` are the sizing basis for the capture funnel and turn clearance. | Capture and rail envelope |
| `ASSUMP-004` | The final assembly is intentionally static; no engineered DOFs are required to complete the transfer. | `final_assembly` |

## 5. Detailed Calculations

| ID | Problem / Decision | Result | Impact |
| -- | -- | -- | -- |
| CALC-001 | Determine the base plate mass from its stock volume and aluminum density. | 356.40 g | Sets the dominant share of the weight budget. |
| CALC-002 | Determine the capture funnel mass from its stock volume and HDPE density. | 22.80 g | Confirms the catcher does not drive the mass budget. |
| CALC-003 | Determine the outer rail mass from its stock volume and HDPE density. | 17.10 g | Keeps the routed turn light enough for static support. |
| CALC-004 | Determine the inner rail mass from its stock volume and HDPE density. | 16.15 g | Confirms the inner guide is a minor budget item. |
| CALC-005 | Determine the blocker skirt mass from its stock volume and HDPE density. | 13.30 g | Keeps the collision guard inexpensive and light. |
| CALC-006 | Determine the goal tray mass from its stock volume and HDPE density. | 19.95 g | Confirms the capture tray stays within the planner cap. |
| CALC-007 | Sum the declared part masses to verify the full assembly total. | 445.70 g | Confirms the assembly sits comfortably under both weight caps. |
| CALC-008 | Sum the declared part costs to verify the full assembly total. | $48.50 | Confirms the assembly sits comfortably under both cost caps. |

### CALC-001: Route base mass

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

- `ASSUMP-002`, `CALC-007`, `route_base`

### CALC-002: Capture funnel mass

#### Problem Statement

Determine the mass contribution of `entry_catcher`.

#### Assumptions

- `entry_catcher` volume is fixed at 24000 mm3.
- `hdpe` density is 0.95 g/cm3.

#### Derivation

- 24000 mm3 = 24.00 cm3.
- 24.00 cm3 x 0.95 g/cm3 = 22.80 g.

#### Worst-Case Check

- The catcher is a minor fraction of the total mass and cannot dominate the budget.

#### Result

- `entry_catcher` mass = 22.80 g.

#### Design Impact

- The capture funnel remains lightweight enough to absorb jitter without driving cost or weight.

#### Cross-References

- `ASSUMP-002`, `CALC-007`, `entry_catcher`

### CALC-003: Outer rail mass

#### Problem Statement

Determine the mass contribution of `outer_rail`.

#### Assumptions

- `outer_rail` volume is fixed at 18000 mm3.
- `hdpe` density is 0.95 g/cm3.

#### Derivation

- 18000 mm3 = 18.00 cm3.
- 18.00 cm3 x 0.95 g/cm3 = 17.10 g.

#### Worst-Case Check

- The rail remains a minor mass contribution and does not threaten the cap.

#### Result

- `outer_rail` mass = 17.10 g.

#### Design Impact

- The routed turn stays lightweight and easy to support statically.

#### Cross-References

- `ASSUMP-002`, `CALC-007`, `outer_rail`

### CALC-004: Inner rail mass

#### Problem Statement

Determine the mass contribution of `inner_rail`.

#### Assumptions

- `inner_rail` volume is fixed at 17000 mm3.
- `hdpe` density is 0.95 g/cm3.

#### Derivation

- 17000 mm3 = 17.00 cm3.
- 17.00 cm3 x 0.95 g/cm3 = 16.15 g.

#### Worst-Case Check

- The inner rail is smaller than the outer rail and remains a minor budget item.

#### Result

- `inner_rail` mass = 16.15 g.

#### Design Impact

- The inside guide adds control without affecting the weight cap.

#### Cross-References

- `ASSUMP-002`, `CALC-007`, `inner_rail`

### CALC-005: Blocker skirt mass

#### Problem Statement

Determine the mass contribution of `blocker_skirt`.

#### Assumptions

- `blocker_skirt` volume is fixed at 14000 mm3.
- `hdpe` density is 0.95 g/cm3.

#### Derivation

- 14000 mm3 = 14.00 cm3.
- 14.00 cm3 x 0.95 g/cm3 = 13.30 g.

#### Worst-Case Check

- The guard remains a thin local add-on and cannot dominate the assembly mass.

#### Result

- `blocker_skirt` mass = 13.30 g.

#### Design Impact

- The collision guard stays cheap and lightweight.

#### Cross-References

- `ASSUMP-002`, `CALC-007`, `blocker_skirt`

### CALC-006: Goal tray mass

#### Problem Statement

Determine the mass contribution of `goal_tray`.

#### Assumptions

- `goal_tray` volume is fixed at 21000 mm3.
- `hdpe` density is 0.95 g/cm3.

#### Derivation

- 21000 mm3 = 21.00 cm3.
- 21.00 cm3 x 0.95 g/cm3 = 19.95 g.

#### Worst-Case Check

- The goal tray remains a minor contributor and does not threaten the assembly cap.

#### Result

- `goal_tray` mass = 19.95 g.

#### Design Impact

- The terminal capture feature stays lightweight enough for a static solution.

#### Cross-References

- `ASSUMP-002`, `CALC-007`, `goal_tray`

### CALC-007: Total assembly mass

#### Problem Statement

Verify that the declared mass total matches the part-by-part sum.

#### Assumptions

- The part masses in `CALC-001` through `CALC-006` are exact and deterministic.

#### Derivation

- 356.40 + 22.80 + 17.10 + 16.15 + 13.30 + 19.95 = 445.70 g.

#### Worst-Case Check

- 445.70 g is below both the 1250 g planner target and the 2200 g benchmark cap.

#### Result

- Total assembly mass = 445.70 g.

#### Design Impact

- The design has substantial mass margin and does not need structural simplification for weight alone.

#### Cross-References

- `CALC-001` through `CALC-006`, `ASSUMP-002`

### CALC-008: Total assembly cost

#### Problem Statement

Verify that the declared cost total matches the part-by-part sum.

#### Assumptions

- The manufactured-part cost estimates are deterministic and already rounded to the contract precision.

#### Derivation

- 16.00 + 5.50 + 6.25 + 6.00 + 5.00 + 9.75 = 48.50.

#### Worst-Case Check

- 48.50 is below both the $64 planner target and the $120 benchmark cap.

#### Result

- Total assembly cost = $48.50.

#### Design Impact

- The design has ample cost margin and does not need cost-driven simplification.

#### Cross-References

- `route_base`, `entry_catcher`, `outer_rail`, `inner_rail`, `blocker_skirt`, `goal_tray`

## 6. Critical Constraints / Operating Envelope

- `route_base` must stay inside the seeded build zone.
- The routed path must pass outside the central forbid zone on the positive-Y side.
- `entry_catcher` must absorb the spawn jitter envelope instead of assuming a lucky first bounce.
- `outer_rail`, `inner_rail`, and `blocker_skirt` must maintain a continuous clearance corridor through the turn.
- `goal_tray` must sit far enough into the goal zone that the settled ball is unambiguously captured.
- No unnecessary moving DOFs are permitted; the final assembly is static.

## 7. Cost & Weight Budget

| Item | Weight (g) | Cost ($) |
| -- | -- | -- |
| `route_base` | 356.40 | 16.00 |
| `entry_catcher` | 22.80 | 5.50 |
| `outer_rail` | 17.10 | 6.25 |
| `inner_rail` | 16.15 | 6.00 |
| `blocker_skirt` | 13.30 | 5.00 |
| `goal_tray` | 19.95 | 9.75 |
| **TOTAL** | **445.70** | **48.50** |

| Budget | Cap | Planned | Margin |
| -- | -- | -- | -- |
| Benchmark customer cap | 2200.00 g | 445.70 g | 1754.30 g |
| Planner target | 1250.00 g | 445.70 g | 804.30 g |
| Benchmark customer cap | $120.00 | $48.50 | $71.50 |
| Planner target | $64.00 | $48.50 | $15.50 |

## 8. Risk Assessment

| Risk | Likelihood | Impact | Mitigation |
| -- | -- | -- | -- |
| Ball clips the blocker corner under jitter | Medium | High | Keep the routed bend outside the forbid volume and reserve extra rail height through the turn |
| Ball exits the outer rail on the long bend | Medium | Medium | Increase rail height through the turn and keep the bend radius shallow |
| Goal tray receives the ball too fast | Low | Medium | Drop the tray slightly below the rail exit to absorb speed |
| The benchmark-owned `environment_fixture` becomes a hidden dependency | Low | High | Treat it as read-only context and do not rely on modifying it during engineering |
