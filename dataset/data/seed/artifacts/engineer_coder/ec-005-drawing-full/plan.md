# Engineering Plan

## 1. Solution Overview

Use a passive offset rail path that routes `projectile_ball` around the seeded `central_blocker` and into the far goal zone. The mechanism widens the capture area near spawn, steers the ball around the blocker on the positive-Y side, and settles it into a downstream `goal_tray`. The `routed_transfer` subassembly is a single passive routing path with no powered axes.

## 2. Parts List

| Part | Dimensions (mm) | Material | Purpose |
| -- | -- | -- | -- |
| route_base | 760 x 150 x 10 | aluminum_6061 | Freestanding base plate spanning the permitted route corridor |
| entry_catcher | 170 x 140 x 35 | hdpe | Capture funnel that absorbs spawn jitter before the routing turn |
| outer_rail | 560 x 18 x 28 | hdpe | Outside guide rail along the routed path |
| inner_rail | 520 x 18 x 28 | hdpe | Inside guide rail that keeps the ball away from the central blocker |
| blocker_skirt | 210 x 20 x 60 | hdpe | Clearance wall keeping the ball from clipping the blocker corner |
| goal_tray | 160 x 120 x 35 | hdpe | Terminal tray overlapping the goal zone |

**Estimated Total Weight**: 445.7 g
**Estimated Total Cost**: $48.50

## 3. Assembly Strategy

1. Place `route_base` inside the seeded build zone with its long axis centered slightly above the blocker band so the passive path can stay on the positive-Y side of `central_blocker`.
2. Mount `entry_catcher` near the spawn side, then mount `outer_rail` and `inner_rail` so the path bends around the blocker on the positive-Y side without crossing the forbid zone.
3. Mount `blocker_skirt` beside the routed turn and terminate the path inside `goal_tray` so the ball settles inside the goal zone with a passive capture lip.

## 4. Assumption Register

- `ASSUMP-001`: `projectile_ball` radius is held within the declared 28-30 mm static randomization band.
- `ASSUMP-002`: The passive rail path can retain the ball with side-wall capture alone; no motion or actuation is required.
- `ASSUMP-003`: The positive-Y route remains clear of `central_blocker` because the rails and tray are staged above the blocker band in the review geometry.
- `ASSUMP-004`: The manufacturing estimate in `assembly_definition.yaml` is the authoritative budget contract for coder entry.

## 5. Detailed Calculations

| ID | Problem / Decision | Result | Impact |
| -- | -- | -- | -- |
| CALC-001 | Capture envelope versus spawn jitter | `entry_catcher` mouth and `route_base` width cover the full runtime jitter band with margin | Keeps the projectile from escaping during seeded spawn variation |
| CALC-002 | Positive-Y blocker clearance | Rail centers stay on the positive-Y side of `central_blocker` with z-separated review geometry | Preserves the routed turn without entering the forbid zone |
| CALC-003 | Mass and budget closure | Total estimated weight is 445.7 g and total cost is $48.50 | Keeps the solution below the benchmark customer caps |

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

### CALC-002: Positive-Y Blocker Clearance

#### Problem Statement

The passive transfer must pass around `central_blocker` without entering the forbid volume.

#### Assumptions

- The route stays on the positive-Y side of the blocker band.
- The drafting geometry is review evidence, not the final solution geometry.

#### Derivation

The staged rail centers sit above the blocker band and keep the turn corridor on the positive-Y side while preserving a simple passive path.

#### Worst-Case Check

Even with the widest part envelopes, the staged geometry remains outside the blocker footprint.

#### Result

The route preserves a clearance margin while still pointing into `goal_tray`.

#### Design Impact

This makes the passive transfer readable in review and feasible for the downstream coder.

#### Cross-References

- `outer_rail`, `inner_rail`, and `blocker_skirt` in `assembly_definition.yaml`
- `central_blocker` in `benchmark_definition.yaml`

### CALC-003: Mass and Budget Closure

#### Problem Statement

The engineer-owned assembly must stay under the benchmark customer caps.

#### Assumptions

- The deterministic total mass and cost from `assembly_definition.yaml` are authoritative.
- No COTS components are used in this seed.

#### Derivation

The part-table total weight sums to 445.7 g and the cost table sums to $48.50.

#### Worst-Case Check

Both totals are comfortably below the benchmark caps of 1250 g and $64.00.

#### Result

The plan remains inside budget and leaves substantial margin for implementation variation.

#### Design Impact

The coder can preserve the passive routing strategy without needing to redesign for cost or weight.

#### Cross-References

- `assembly_definition.yaml`
- `benchmark_definition.yaml`

## 6. Critical Constraints / Operating Envelope

- `route_base` stays within the build zone and supports the full routing span.
- `entry_catcher` is staged on the spawn side with enough opening width to capture the jittered spawn band.
- `outer_rail` and `inner_rail` stay on the positive-Y side of `central_blocker`.
- `blocker_skirt` protects the corner turn without introducing a powered axis.
- `goal_tray` captures the ball at the end of the passive route and remains inside the build zone.
- Total estimated weight remains at 445.7 g, well below the 1250 g cap.
- Total estimated cost remains at $48.50, well below the $64.00 cap.

## 7. Cost & Weight Budget

| Item | Weight (g) | Cost ($) |
| -- | -- | -- |
| route_base | 359.0 | 16.0 |
| entry_catcher | 21.5 | 5.5 |
| outer_rail | 17.0 | 6.25 |
| inner_rail | 16.5 | 6.0 |
| blocker_skirt | 14.0 | 5.0 |
| goal_tray | 17.7 | 9.75 |
| **TOTAL** | **445.7** | **48.5** |

**Budget Margin**: 804.3 g and $15.50 remaining versus the benchmark caps.

## 8. Risk Assessment

| Risk | Likelihood | Impact | Mitigation |
| -- | -- | -- | -- |
| Ball clips the blocker corner under jitter | Medium | High | Use a dedicated blocker skirt and keep the routed bend outside the forbid volume |
| Ball exits the outer rail on the long bend | Medium | Medium | Increase rail height through the turn and keep the bend radius shallow |
| Goal tray receives the ball too fast | Low | Medium | Drop the tray slightly below the rail exit to absorb speed |
