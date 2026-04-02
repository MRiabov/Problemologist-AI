## 1. Solution Overview

INT-033 engineering full-loop deterministic plan.

## 2. Parts List

- `environment_fixture`: passive geometry that keeps the benchmark rooted
  in a fixed assembly.
- `solution_plan_evidence`: planner-authored drafted part used to satisfy the
  engineer drafting contract.

## 3. Assembly Strategy

1. Place the moved object `projectile_ball` directly in the goal zone.
2. Keep `environment_fixture` fixed and leave `solution_plan_evidence` as the
   drafted planner artifact.

## 4. Cost & Weight Budget

- Estimated unit cost: $10
- Estimated weight: 100g

## 5. Risk Assessment

- Low complexity.
