## 1. Learning Objective

- Train passive rigid-body reasoning for a ball transfer benchmark where the
  payload moves from a left start deck to a right goal deck by gravity over a
  central void.
- Keep the family simple, reproducible, and fully rigid-body: no actuators,
  FEM, or fluids.

## 2. Static Geometry

- `left_start_deck`: the launch surface that anchors the initial sphere
  position.
- `right_goal_deck`: the capture surface that receives the sphere after it
  crosses the gap.
- `bridge_reference_table`: a central reference block that rises above the
  void and gives the family a stable geometric landmark without adding motion.
- `gap_floor_guard`: a low passive guard under the span that keeps the scene
  readable and discourages unsafe fallback paths.

## 3. Input Object

- Shape: `sphere`
- Label: `projectile_ball`
- Static randomization: radius in `[4, 6]` mm
- Nominal start position: `[-30, 0, 24]`
- Runtime jitter:
  - Position: `[2, 2, 1]` mm
  - Properties: keep mass and friction constant for the family

## 4. Objectives

- Goal zone:
  - Type: AABB
  - Location: `min [27, -6, 6]`, `max [37, 6, 14]`
- Forbid zones:
  - `central_void`: `min [-7, -10, 0]`, `max [7, 10, 14]`
- Build zone:
  - `min [-40, -28, 0]`
  - `max [40, 28, 48]`

## 5. Design

- The benchmark stays passive: the sphere crosses the span under gravity and
  the static geometry keeps the trajectory aligned with the goal deck.
- There are no benchmark-owned moving fixtures, actuators, FEM parts, or fluid
  features in this family.
- Any benchmark-side motion that would require a controller is out of scope and
  must be rejected rather than adapted.

## 6. Randomization

- Static: allow only the declared radius variation on the sphere and keep the
  benchmark fixtures fixed.
- Runtime: use the declared position jitter on the sphere spawn position to
  test robustness.

## 7. Build123d Strategy

- Use simple CSG primitives: two deck plates, one central landmark block, and a
  low guard block under the gap.
- Keep the geometry readable and aligned to world axes so the bridge path is
  obvious to the reviewer.

## 8. Cost & Weight Envelope

- `totals.estimated_unit_cost_usd`: `14.0`
- `totals.estimated_weight_g`: `15.1632`
- `totals.estimate_confidence`: `high`
- The estimate assumes a small passive fixture family built from common rigid
  materials and a lightweight sphere input object.

## 9. Part Metadata

- `left_start_deck`, `right_goal_deck`, `bridge_reference_table`, and
  `gap_floor_guard` must be static (`fixed: true`) and carry a known
  `material_id`.
- The moved object uses `material_id: abs`.
- No part in this family should introduce powered or deformable behavior.
