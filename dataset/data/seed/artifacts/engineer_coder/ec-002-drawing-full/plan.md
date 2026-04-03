# Engineering Plan

## 1. Solution Overview

Use a freestanding bridge deck with shallow side fences to move `transfer_cube` from the seeded `left_start_deck` across the `floor_gap` and into the `right_goal_deck` capture zone. The passive path uses a 300 mm `bridge_deck` that clears the 160 mm void with 70 mm nominal overlap on each side, and a 130 x 110 x 30 `landing_pocket` that overlaps the `goal_zone` to catch the cube before rebound. The benchmark-owned `bridge_reference_table` and `gap_floor_guard` stay read-only context; the solution uses them only as spatial references while keeping the bridge passive and within the planner budgets.

## 2. Parts List

| Part | Dimensions (mm) | Material | Purpose |
| -- | -- | -- | -- |
| base_frame | 560 x 180 x 12 | aluminum_6061 | Freestanding support frame that keeps the bridge aligned without touching the benchmark fixtures |
| bridge_deck | 300 x 95 x 8 | aluminum_6061 | Main transfer surface across the gap |
| left_fence | 300 x 20 x 35 | hdpe | Left-side guide fence that prevents lateral escape |
| right_fence | 300 x 20 x 35 | hdpe | Right-side guide fence that prevents lateral escape |
| landing_pocket | 130 x 110 x 30 | hdpe | Receives the cube at the goal side and damps rebound |

**Estimated Total Weight**: 452 g
**Estimated Total Cost**: $57.50

## 3. Assembly Strategy

1. Place `base_frame` centered in the build zone so its 560 x 180 mm footprint stays inside the 700 x 360 mm build footprint with 70 mm x 90 mm nominal margin and the support feet remain clear of the `floor_gap` keep-out volume.
2. Mount `bridge_deck` along the x-axis so the 300 mm span covers the 160 mm gap with 70 mm of contact length on each side and points toward `right_goal_deck`.
3. Mount `left_fence` and `right_fence` along the deck edges to keep the 16 x 16 x 10 mm runtime jitter envelope centered on the 95 mm deck without letting the cube climb the rails.
4. Position `landing_pocket` so its 130 x 110 x 30 mm body overlaps the `goal_zone` with 10 mm of x-overlap and 15 mm of y-side clearance while damping rebound.
5. Keep `bridge_reference_table` and `gap_floor_guard` as read-only spatial references only, and keep every part label grounded in `plan.md`, `todo.md`, and `assembly_definition.yaml`.
6. The drafting sheet callouts `1`-`5` track the base frame, bridge deck, left fence, right fence, and landing pocket, respectively.

## 4. Assumption Register

| ID | Assumption | Source | Used By |
| -- | -- | -- | -- |
| ASSUMP-001 | The `floor_gap` keep-out stays fixed at 160 mm in x and 300 mm in y, and the support frame can be centered without touching it. | `benchmark_definition.yaml` | CALC-001, CALC-002 |
| ASSUMP-002 | The bridge remains fully passive; no actuators, hinges, or benchmark-side moving fixtures are added. | `benchmark_definition.yaml` and `benchmark_assembly_definition.yaml` | CALC-002, CALC-004 |
| ASSUMP-003 | Runtime jitter on `transfer_cube` is limited to `±8 mm` in x and y and `±5 mm` in z. | `benchmark_definition.yaml` | CALC-003 |
| ASSUMP-004 | `landing_pocket` is allowed to overlap the `goal_zone` as the capture feature. | `benchmark_definition.yaml` | CALC-004 |
| ASSUMP-005 | `bridge_reference_table` and `gap_floor_guard` remain read-only references and are not touched by the implementation. | `benchmark_definition.yaml` and `benchmark_assembly_definition.yaml` | CALC-001, CALC-004 |

## 5. Detailed Calculations

| ID | Problem / Decision | Result | Impact |
| -- | -- | -- | -- |
| CALC-001 | Can the `base_frame` fit inside the `build_zone` with enough placement slack to keep the support structure clear of the `floor_gap`? | The `build_zone` footprint is 700 x 360 mm, the `base_frame` footprint is 560 x 180 mm, and the centered slack is 140 mm in x and 180 mm in y, leaving 70 mm and 90 mm per side. | Confirms the freestanding support has enough placement margin to stay clear of the gap keep-out once the bridge deck is aligned. |
| CALC-002 | Does the passive bridge path span the full void with enough overlap to remain stable? | The `bridge_deck` length is 300 mm and the `floor_gap` x-span is 160 mm, so the deck has 140 mm of total excess length, or 70 mm of nominal overlap on each side. | Gives the bridge adequate contact area at both ends without adding motion or powered correction. |
| CALC-003 | Does the `transfer_cube` jitter envelope stay centered between the side fences and within the deck corridor? | Runtime jitter totals 16 mm in x, 16 mm in y, and 10 mm in z; the 95 mm deck width leaves 79 mm of lateral slack, and the 35 mm fences stand 5 mm taller than the 30 mm goal-side pocket height. | Keeps the cube on a passive, repeatable path across the bridge. |
| CALC-004 | Does the `landing_pocket` overlap the `goal_zone` enough to catch the cube and suppress rebound? | The `goal_zone` spans 110 mm in x, 140 mm in y, and 95 mm in z. The `landing_pocket` spans 130 mm in x, 110 mm in y, and 30 mm in z, so it has 20 mm of x excess, 15 mm of y clearance per side, and occupies the lower third of the goal volume. | Lets the pocket capture the cube inside the goal area while leaving the rest of the zone unobstructed. |

### CALC-001: Base frame fit and void clearance

#### Problem Statement

Verify that `base_frame` can be positioned inside the `build_zone` with enough placement slack to keep the support structure clear of `floor_gap`.

#### Assumptions

- `ASSUMP-001`: the void size is fixed.
- `ASSUMP-005`: the reference parts remain read-only.

#### Derivation

- `build_zone` footprint: 700 mm in x by 360 mm in y.
- `base_frame` footprint: 560 mm in x by 180 mm in y.
- Remaining slack: 140 mm in x and 180 mm in y.
- Centered margin: 70 mm per side in x and 90 mm per side in y.

#### Worst-Case Check

- Even with small placement drift, the frame still has substantial build-zone slack on both axes, so the support can be nudged onto solid ground without colliding with the void.

#### Result

- The `base_frame` fits the build envelope with ample placement margin.

#### Design Impact

- The bridge can stay freestanding and passive instead of depending on the benchmark fixtures for support.

#### Cross-References

- `plan.md#3-assembly-strategy`
- `plan.md#6-critical-constraints--operating-envelope`
- `benchmark_definition.yaml`
- `benchmark_assembly_definition.yaml`

### CALC-002: Bridge span and nominal overlap

#### Problem Statement

Verify that `bridge_deck` spans `floor_gap` with enough overlap to keep the passive transfer path continuous.

#### Assumptions

- `ASSUMP-001`: the gap width is fixed.
- `ASSUMP-002`: the bridge stays passive.

#### Derivation

- `floor_gap` x-span: 160 mm.
- `bridge_deck` length: 300 mm.
- Total excess length: 140 mm.
- Nominal overlap if centered: 70 mm per side.

#### Worst-Case Check

- A small placement offset still leaves a large overlap reserve at both ends, so the bridge can remain stable without any powered correction.

#### Result

- The deck span covers the void with enough contact area on both ends.

#### Design Impact

- The engineer can keep the transfer path simple, passive, and manufacturable.

#### Cross-References

- `plan.md#1-solution-overview`
- `plan.md#3-assembly-strategy`
- `benchmark_definition.yaml`

### CALC-003: Jitter capture and guide corridor

#### Problem Statement

Verify that the runtime jitter envelope stays inside the passive guide corridor formed by `bridge_deck`, `left_fence`, and `right_fence`.

#### Assumptions

- `ASSUMP-003`: the spawn jitter values are exact.

#### Derivation

- Runtime jitter envelope: 16 mm in x, 16 mm in y, and 10 mm in z.
- `bridge_deck` width: 95 mm.
- Lateral slack versus the jitter envelope: 79 mm.
- Fence height: 35 mm.
- Goal-side pocket height: 30 mm.
- Fence height reserve above the pocket height: 5 mm.

#### Worst-Case Check

- The deck and fence envelope still leave wide static clearance around the jittered spawn positions, so the cube can stay centered without a driven correction stage.

#### Result

- The passive guide corridor is wide enough for the jittered cube path.

#### Design Impact

- The solution remains robust across the declared runtime variation while staying passive.

#### Cross-References

- `plan.md#1-solution-overview`
- `plan.md#3-assembly-strategy`
- `benchmark_definition.yaml`

### CALC-004: Goal capture overlap and rebound control

#### Problem Statement

Verify that `landing_pocket` overlaps the `goal_zone` and absorbs the cube before it can rebound out of the capture area.

#### Assumptions

- `ASSUMP-004`: goal-zone overlap is allowed for the capture feature.
- `ASSUMP-002`: no moving benchmark-side hardware is introduced.

#### Derivation

- `goal_zone` span in x: 110 mm.
- `goal_zone` span in y: 140 mm.
- `goal_zone` span in z: 95 mm.
- `landing_pocket` span in x: 130 mm.
- `landing_pocket` span in y: 110 mm.
- `landing_pocket` span in z: 30 mm.
- X overlap reserve: 20 mm total, or 10 mm per side when centered.
- Y clearance reserve: 15 mm per side.
- Vertical capture depth: the pocket occupies the lower third of the goal volume.

#### Worst-Case Check

- If the cube arrives slightly off-center or slightly high, the pocket still sits inside the goal volume and keeps the final motion passive.

#### Result

- The goal-side receiver overlaps the goal zone and provides a stable capture pocket.

#### Design Impact

- The cube can settle into the goal without bouncing back through the exit path.

#### Cross-References

- `plan.md#3-assembly-strategy`
- `plan.md#6-critical-constraints--operating-envelope`
- `benchmark_definition.yaml`

## 6. Critical Constraints / Operating Envelope

| Limit ID | Limit | Bound | Basis |
| -- | -- | -- | -- |
| LIMIT-001 | Base frame fit | `560 x 180 mm` footprint inside the `700 x 360 mm` build-zone footprint | `CALC-001` |
| LIMIT-002 | Gap coverage | `300 mm` bridge deck across the `160 mm` gap with `70 mm` nominal overlap per side | `CALC-002` |
| LIMIT-003 | Jitter corridor | `16 x 16 x 10 mm` runtime envelope remains inside the passive guide corridor | `CALC-003` |
| LIMIT-004 | Goal capture | `130 x 110 x 30 mm` landing pocket overlaps `goal_zone` with x-overlap and y clearance reserve | `CALC-004` |
| LIMIT-005 | Motion contract | Passive only, no added DOFs or powered components | `ASSUMP-002` |
| LIMIT-006 | Budget envelope | Keep the solution under the planner target with cost as the governing margin | `CALC-002`, `CALC-004` |

## 7. Cost & Weight Budget

| Item | Volume (cm^3) | Weight (g) | Cost ($) |
| -- | -- | -- | -- |
| base_frame | 121.0 | 326.0 | 19.50 |
| bridge_deck | 22.8 | 61.6 | 11.00 |
| left_fence | 21.0 | 20.2 | 7.00 |
| right_fence | 21.0 | 20.2 | 7.00 |
| landing_pocket | 25.0 | 24.0 | 13.00 |
| **TOTAL** | 210.8 | 452.0 | **57.50** |

**Budget Margin**: 31% remaining versus the planner target on cost; weight remains 73% under the planner target.

## 8. Risk Assessment

| Risk | Likelihood | Impact | Mitigation |
| -- | -- | -- | -- |
| Bridge support intrudes into the forbid zone | Low | High | Keep all frame feet outside the seeded gap AABB and validate the footprint against `CALC-001` |
| Cube yaws and rides over a fence | Medium | High | Keep the fences high enough to resist yaw while preserving top clearance from `CALC-003` |
| Cube rebounds out of the landing area | Medium | Medium | Use the `landing_pocket` overlap and lower capture volume from `CALC-004` |
| Bridge deck flex reduces consistency | Low | Medium | Keep the deck short and support it from both ends with the aluminum frame |

### Jitter Robustness Check

- Capture area covers spawn jitter: Yes
- Tested edge cases considered: left-offset spawn, right-offset spawn, low-Z spawn, high-Z spawn, shallow-angle entry
