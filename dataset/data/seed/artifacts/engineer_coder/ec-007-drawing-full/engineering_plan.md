# Engineering Plan

## 1. Solution Overview

Use a passive low-angle chute with tall guide walls and a shaped bypass around the `center_collision_block` to move the `slider_cube` into the goal zone. The geometry avoids relying on friction and instead constrains the cube with continuous walls and gentle transitions.

The `low_friction_route` subassembly is a single passive routing path with no powered axes.

## 2. Parts List

| Part | Dimensions (mm) | Material | Purpose |
| -- | -- | -- | -- |
| slide_base | 620 x 140 x 10 | aluminum_6061 | Freestanding base for the low-friction guide path |
| entry_box | 160 x 120 x 38 | hdpe | Wide entry pocket that catches the jittered cube without rebound |
| guide_wall_left | 420 x 18 x 42 | hdpe | Left continuous wall guiding the cube around the blocker |
| guide_wall_right | 390 x 18 x 42 | hdpe | Right continuous wall shaping the bypass path |
| blocker_bypass_panel | 200 x 18 x 65 | hdpe | Panel that keeps the cube from sliding into the central forbid block |
| goal_pocket | 110 x 90 x 32 | hdpe | Final pocket settling the cube in the goal zone |

**Estimated Total Weight**: 399.35 g
**Estimated Total Cost**: $39.50

## 3. Assembly Strategy

1. Mount `entry_box` on `slide_base` so the jittered cube is captured before it reaches the routed section.
2. Mount the two guide walls and `blocker_bypass_panel` to create a smooth, continuous path around the seeded forbid zone.
3. Mount `goal_pocket` to capture the goal zone so the cube settles inside the target volume. The geometry explicitly occupies the goal zone to prevent escape.

## 4. Assumption Register

- `ASSUMP-001`: `slider_cube` spawn jitter stays within the declared [14, 12, 3] mm band.
- `ASSUMP-002`: The drafting scripts are exploded review layouts only; they preserve the same inventory and dimensions but are not literal stack coordinates.
- `ASSUMP-003`: The passive chute path is static and reviewable without any engineer-owned motion DOFs.
- `ASSUMP-004`: The manufacturing estimate in `assembly_definition.yaml` is the authoritative budget contract for coder entry.
- `ASSUMP-005`: `aluminum_6061` density is 2.7 g/cm3; `hdpe` density is 0.95 g/cm3.

## 5. Detailed Calculations

| ID | Problem / Decision | Result | Impact |
| -- | -- | -- | -- |
| CALC-001 | Capture envelope versus spawn jitter | Required half-width is `14 mm + 12 mm = 26 mm`; `entry_box` provides `60 mm` half-width | Keeps the cube inside the passive inlet during runtime jitter |
| CALC-002 | Blocker keepout feasibility | The route must stay outside the `center_collision_block` AABB `[110, 220] x [-90, 90] x [0, 120]` while preserving the bypass | Prevents the passive corridor from clipping the forbid volume |
| CALC-003 | Base mass | `318.60 g` | Dominant weight contribution, still well under cap |
| CALC-004 | Remaining part masses | `19.95 + 15.20 + 14.25 + 13.30 + 18.05 = 80.75 g` | Confirms the smaller HDPE parts stay lightweight |
| CALC-005 | Mass and budget closure | Total estimated weight is `399.35 g` and total cost is `$39.50` | Keeps the solution below the benchmark customer caps |

### CALC-001: Capture Envelope Versus Spawn Jitter

#### Problem Statement

The capture inlet must tolerate the declared spawn jitter while still feeding the passive chute path.

#### Assumptions

- The cube spawn position stays inside the declared jitter band.
- The inlet acts as a passive capture pocket rather than a powered sorter.

#### Derivation

The capture face is sized wider than the jittered spawn envelope.

#### Worst-Case Check

The worst-case lateral and vertical spawn offsets still fit inside the inlet opening.

#### Result

The capture face absorbs the full jitter band.

#### Design Impact

The cube settles into the chute path before reaching the bypass section.

#### Cross-References

- `entry_box` in `assembly_definition.yaml`
- `slider_cube` in `benchmark_definition.yaml`

### CALC-002: Blocker Keepout Feasibility

#### Problem Statement

The passive transfer must pass around `center_collision_block` without entering the forbid volume.

#### Assumptions

- The route stays on one side of the blocker band.
- The drafting geometry is review evidence, not the final solution geometry.

#### Derivation

The staged wall centers sit outside the blocker AABB and keep the turn corridor on one side while preserving a simple passive path.

#### Worst-Case Check

Even with the widest part envelopes, the staged geometry remains outside the blocker footprint.

#### Result

The route preserves a clearance margin while still pointing into `goal_pocket`.

#### Design Impact

This makes the passive transfer readable in review and feasible for the downstream coder.

#### Cross-References

- `guide_wall_left`, `guide_wall_right`, `blocker_bypass_panel` in `assembly_definition.yaml`
- `center_collision_block` in `benchmark_definition.yaml`

### CALC-003: Base Mass

#### Problem Statement

Determine the mass contribution of `slide_base`.

#### Assumptions

- `slide_base` volume is 118000 mm3.
- `aluminum_6061` density is 2.7 g/cm3.

#### Derivation

- 118.00 cm3 x 2.7 g/cm3 = 318.60 g.

#### Worst-Case Check

This is the largest single-part mass in the assembly, so it dominates the weight budget.

#### Result

`slide_base` mass = 318.60 g.

#### Design Impact

The base plate dominates the weight budget but leaves margin.

#### Cross-References

- `ASSUMP-005`, `CALC-005`, `slide_base`

### CALC-004: Remaining Part Masses

#### Problem Statement

Determine the mass contribution of the remaining parts.

#### Assumptions

- HDPE density is 0.95 g/cm3.
- The part volumes in `assembly_definition.yaml` are deterministic.

#### Derivation

- `entry_box`: 21.00 cm3 x 0.95 = 19.95 g.
- `guide_wall_left`: 16.00 cm3 x 0.95 = 15.20 g.
- `guide_wall_right`: 15.00 cm3 x 0.95 = 14.25 g.
- `blocker_bypass_panel`: 14.00 cm3 x 0.95 = 13.30 g.
- `goal_pocket`: 19.00 cm3 x 0.95 = 18.05 g.
- Sum = 80.75 g.

#### Worst-Case Check

The remaining parts are all minor contributors and do not threaten the mass cap.

#### Result

Remaining part mass total = 80.75 g.

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

- 318.60 + 80.75 = 399.35 g.
- 14.50 + 5.00 + 5.75 + 5.25 + 4.50 + 4.50 = 39.50 USD.

#### Worst-Case Check

- 399.35 g is below the 980.0 g benchmark cap.
- $39.50 is below the $50.00 benchmark cap.

#### Result

The plan remains inside budget with substantial margin.

#### Design Impact

The coder can preserve the passive routing strategy without needing to redesign for cost or weight.

#### Cross-References

- `assembly_definition.yaml`, `benchmark_definition.yaml`

## 6. Critical Constraints / Operating Envelope

- `slide_base` stays within the build zone and supports the full routing span.
- `entry_box` is staged on the spawn side with enough opening width to capture the jittered spawn band.
- `guide_wall_left` and `guide_wall_right` form a passive chute corridor around the `center_collision_block`.
- `blocker_bypass_panel` shields the route corner without introducing a powered axis.
- `goal_pocket` captures the cube at the end of the passive route and overlaps the seeded goal zone.
- Total estimated weight remains at 399.35 g, well below the 980.0 g cap.
- Total estimated cost remains at $39.50, well below the $50.00 cap.

## 7. Cost & Weight Budget

| Item | Weight (g) | Cost ($) |
| -- | -- | -- |
| slide_base | 318.60 | 14.50 |
| entry_box | 19.95 | 5.00 |
| guide_wall_left | 15.20 | 5.75 |
| guide_wall_right | 14.25 | 5.25 |
| blocker_bypass_panel | 13.30 | 4.50 |
| goal_pocket | 18.05 | 4.50 |
| **TOTAL** | **399.35** | **39.50** |

**Budget Margin**: 580.65 g and $10.50 remaining versus the benchmark caps.

## 8. Risk Assessment

| Risk | Likelihood | Impact | Mitigation |
| -- | -- | -- | -- |
| Preview geometry drifts away from the final corridor | Medium | High | Keep the drafted shapes as the authoritative inventory and rebuild the same passive family in the implementation script |
| Cube slides too fast and clips the blocker | Medium | High | Keep the routed path continuous and use a tall `blocker_bypass_panel` at the critical corner |
| Cube rebounds out of the goal under low friction | Medium | Medium | Use a closed `goal_pocket` instead of an open tray |
| Hidden environment contact violates the no-drill rule | Low | High | Keep all geometry referenced from `slide_base` and leave explicit clearance to the `low_friction_benchmark` and `environment_fixture` |
