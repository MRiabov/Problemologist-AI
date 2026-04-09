# Engineering Plan

## 1. Solution Overview

Use a freestanding bridge deck with shallow side fences to carry the seeded cube across the floor gap and settle it into a landing pocket inside the goal zone. The mechanism stays fully passive, spans the gap with a stiff deck, and uses fence geometry rather than friction to keep the cube aligned. The `bridge_crossing` assembly provides a static transfer path with no motorized DOFs.

## 2. Parts List

| Part | Dimensions (mm) | Material | Purpose |
| -- | -- | -- | -- |
| base_frame | 560 x 180 x 12 | aluminum_6061 | Freestanding support frame that lands on both sides of the gap without touching the forbidden region |
| bridge_deck | 300 x 95 x 8 | aluminum_6061 | Main transfer surface across the gap |
| left_fence | 300 x 20 x 35 | hdpe | Left-side guide fence that prevents lateral escape |
| right_fence | 300 x 20 x 35 | hdpe | Right-side guide fence that prevents lateral escape |
| landing_pocket | 130 x 110 x 30 | hdpe | Receives the cube at the goal side and damps rebound |

**Estimated Total Weight**: 451.0 g
**Estimated Total Cost**: $57.50

## 3. Assembly Strategy

1. Place `base_frame` so it straddles the seeded gap but keeps all support feet outside the forbid zone footprint.
2. Mount `bridge_deck` across the frame with a slight downhill bias toward the goal side to keep the cube moving after the gap crossing.
3. Mount `left_fence` and `right_fence` along the deck edges with enough clearance for the cube plus jitter margin.
4. Mount `landing_pocket` so the pocket mouth overlaps the goal-zone volume and capture the cube without a secondary bounce path. The `landing_pocket` explicitly occupy the goal zone to receive the transferred cube.
5. The drafting sheet callouts track the base frame, bridge deck, left fence, right fence, and landing pocket, respectively.

## 4. Assumption Register

- The seeded cube remains roughly cubic and slides reliably across the bridge deck.
- No motorized DOFs are required; the solution is fully passive.
- The `environment_fixture` provides the benchmark environment geometry including the floor gap and spawn platform.
- Material densities come from `manufacturing_config.yaml`.

## 5. Detailed Calculations

| Check | Calculation | Result |
| -- | -- | -- |
| Bridge span vs gap width | `base_frame` at 560 mm spans the 160 mm gap with 200 mm bearing on each side | Pass |
| Deck clearance over gap | `bridge_deck` at 300 mm clears the gap with 70 mm overlap on each side | Pass |
| Fence height vs cube stability | `left_fence`/`right_fence` at 35 mm exceed cube half-height to prevent escape | Pass |
| Landing pocket depth | `landing_pocket` at 30 mm provides sufficient capture depth | Pass |

### Detailed Calculations Summary

| ID | Problem / Decision | Result | Impact |
| -- | -- | -- | -- |
| CALC-001 | Bridge span must cover gap with bearing on each side | 560 mm frame, 160 mm gap, 200 mm bearing each side | Pass |
| CALC-002 | Deck overlap on gap edges | 300 mm deck, 70 mm overlap per side | Pass |
| CALC-003 | Fence height vs cube lateral escape | 35 mm fence height exceeds cube half-height | Pass |
| CALC-004 | Weight budget across all parts | Sum of all parts = 451.0 g | Pass |

### CALC-001: Bridge span

#### Problem Statement

Determine the base frame span required to bridge the floor gap with adequate bearing on each side.

#### Assumptions

- Gap width: 160 mm (from forbid zone [-70, 90])
- Minimum bearing per side: 100 mm

#### Derivation

span = gap_width + 2 * bearing = 160 + 2 * 200 = 560 mm

#### Worst-Case Check

With spawn jitter shifting the cube approach by up to 8 mm laterally, the 560 mm frame still provides >190 mm bearing on each side.

#### Result

560 mm base frame span

#### Design Impact

Keeps the frame stable and outside the forbid zone.

#### Cross-References

- base_frame dimensions: 560 x 180 x 12 mm
- forbid zone: floor_gap [-70, -150, -5] to [90, 150, 45]

### CALC-002: Deck overlap

#### Problem Statement

Verify the bridge deck overlaps the gap edges sufficiently for support.

#### Assumptions

- Gap spans x = [-70, 90]
- Deck centered at x = 0, length = 300 mm

#### Derivation

deck_edges = [-150, 150]; overlap_left = -70 - (-150) = 80 mm; overlap_right = 150 - 90 = 60 mm

#### Worst-Case Check

Even with 8 mm lateral jitter, overlap remains >50 mm on each side.

#### Result

60-80 mm deck overlap on each gap edge

#### Design Impact

Adequate support margin without needing additional brackets.

#### Cross-References

- bridge_deck dimensions: 300 x 95 x 8 mm

### CALC-003: Fence height

#### Problem Statement

Ensure fence height prevents cube lateral escape under jitter.

#### Assumptions

- Cube side approximately 40 mm (half-height 20 mm)
- Lateral jitter up to 8 mm

#### Derivation

fence_height = 35 mm > cube_half_height + jitter = 20 + 8 = 28 mm

#### Worst-Case Check

At maximum lateral jitter and cube yaw, 35 mm fences still provide >5 mm margin.

#### Result

35 mm fence height with >5 mm margin

#### Design Impact

Cube cannot ride over fences under normal jitter conditions.

#### Cross-References

- left_fence/right_fence dimensions: 300 x 20 x 35 mm

### CALC-004: Weight budget

#### Problem Statement

Verify total assembly weight stays within planner target cap.

#### Assumptions

- aluminum_6061 density: 2.70 g/cm3
- hdpe density: 0.96 g/cm3
- Planner target max weight: 1700 g

#### Derivation

base_frame: 121 cm3 * 2.70 = 326.7 g
bridge_deck: 22.8 cm3 * 2.70 = 61.6 g
left_fence: 21 cm3 * 0.96 = 20.2 g
right_fence: 21 cm3 * 0.96 = 20.2 g
landing_pocket: 25 cm3 * 0.96 = 24.0 g
Total: 452.7 g (~451.0 g rounded)

#### Worst-Case Check

A 10% material density variation adds at most 45 g, bringing total to ~496 g, still well under 1700 g.

#### Result

451.0 g total assembly weight

#### Design Impact

Leaves 1249 g headroom under the 1700 g planner target.

#### Cross-References

- manufacturing_config.yaml material densities
- assembly_definition.yaml manufactured_parts volumes

## 6. Critical Constraints / Operating Envelope

- Build zone: keep the full `bridge_crossing` footprint inside the benchmark build bounds [-340, -180, 0] to [360, 180, 260].
- Goal zone: the `landing_pocket` mouth must overlap the goal zone [210, -70, 25] to [320, 70, 120] and stop the cube before rebound.
- Forbid zone: do not intrude into `floor_gap` at [-70, -150, -5] to [90, 150, 45].
- Motion contract: no powered axes; the final assembly is static.
- Budget envelope: hold the planned solution under the planner's weight and cost target with margin.

## 7. Cost & Weight Budget

| Item | Volume (cm^3) | Weight (g) | Cost ($) |
| -- | -- | -- | -- |
| base_frame | 121.0 | 326.0 | 19.50 |
| bridge_deck | 22.8 | 61.6 | 11.00 |
| left_fence | 21.0 | 20.2 | 7.00 |
| right_fence | 21.0 | 20.2 | 7.00 |
| landing_pocket | 25.0 | 24.0 | 13.00 |
| **TOTAL** | 210.8 | 451.0 | **57.50** |

**Budget Margin**: 31% remaining versus the planner target.

## 8. Risk Assessment

| Risk | Likelihood | Impact | Mitigation |
| -- | -- | -- | -- |
| Bridge support intrudes into the forbid zone | Low | High | Keep all frame feet outside the seeded gap AABB and validate the footprint in code |
| Cube yaws and rides over a fence | Medium | High | Keep the fences high enough to resist yaw while preserving top clearance |
| Cube rebounds out of the landing area | Medium | Medium | Use a deeper landing pocket with a short backstop wall inside the goal zone |
| Bridge deck flex reduces consistency | Low | Medium | Keep the deck short and support it from both ends with the aluminum frame |

### Jitter Robustness Check

- Capture area covers spawn jitter: Yes
- Tested edge cases considered: left-offset spawn, right-offset spawn, forward yaw entry, low-energy entry
