# Engineering Plan

## 1. Solution Overview

Use a freestanding bridge deck with shallow side fences to move `transfer_cube` from the seeded `left_start_deck` across the `floor_gap` and into the `right_goal_deck` capture zone. The benchmark-owned `bridge_reference_table` and `gap_floor_guard` stay read-only context; the solution uses them only as spatial references while keeping the bridge passive and within the planner budgets.

## 2. Parts List

| Part | Dimensions (mm) | Material | Purpose |
| -- | -- | -- | -- |
| base_frame | 560 x 180 x 12 | aluminum_6061 | Freestanding support frame that keeps the bridge aligned without touching the benchmark fixtures |
| bridge_deck | 300 x 95 x 8 | aluminum_6061 | Main transfer surface across the gap |
| left_fence | 300 x 20 x 35 | hdpe | Left-side guide fence that prevents lateral escape |
| right_fence | 300 x 20 x 35 | hdpe | Right-side guide fence that prevents lateral escape |
| landing_pocket | 130 x 110 x 30 | hdpe | Receives the cube at the goal side and damps rebound |

**Estimated Total Weight**: 451 g
**Estimated Total Cost**: $57.50

## 3. Assembly Strategy

1. Place `base_frame` centered in the build zone so the support footprint stays clear of the `floor_gap` keep-out volume and aligned with `left_start_deck`.
2. Mount `bridge_deck` along the x-axis with its span centered over the gap corridor and its far end pointing at `right_goal_deck`.
3. Mount `left_fence` and `right_fence` along the deck edges with enough clearance for the jittered cube to pass without climbing the rails.
4. Position `landing_pocket` so its mouth overlaps the `goal_zone` and captures the cube before it can rebound off the right deck.
5. Keep every part label grounded in `plan.md`, `todo.md`, and `assembly_definition.yaml`, and keep the benchmark fixtures unchanged.

## 4. Cost & Weight Budget

| Item | Volume (cm^3) | Weight (g) | Cost ($) |
| -- | -- | -- | -- |
| base_frame | 121.0 | 326.0 | 19.50 |
| bridge_deck | 22.8 | 61.6 | 11.00 |
| left_fence | 21.0 | 20.2 | 7.00 |
| right_fence | 21.0 | 20.2 | 7.00 |
| landing_pocket | 25.0 | 24.0 | 13.00 |
| **TOTAL** | 210.8 | 451.0 | **57.50** |

**Budget Margin**: 31% remaining versus the planner target.

## 5. Risk Assessment

| Risk | Likelihood | Impact | Mitigation |
| -- | -- | -- | -- |
| Bridge support intrudes into the forbid zone | Low | High | Keep all frame feet outside the seeded gap AABB and validate the footprint in code |
| Cube yaws and rides over a fence | Medium | High | Keep the fences high enough to resist yaw while preserving top clearance |
| Cube rebounds out of the landing area | Medium | Medium | Use a deeper landing pocket with a short backstop wall inside the goal zone |
| Bridge deck flex reduces consistency | Low | Medium | Keep the deck short and support it from both ends with the aluminum frame |

### Jitter Robustness Check

- Capture area covers spawn jitter: Yes
- Tested edge cases considered: left-offset spawn, right-offset spawn, low-Z spawn, high-Z spawn, shallow-angle entry
