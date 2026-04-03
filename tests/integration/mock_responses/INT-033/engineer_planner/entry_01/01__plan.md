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

## 4. Assumption Register

- Assumption: The planner relies on source-backed inputs that must be traceable.

## 5. Detailed Calculations

- CALC-001: The plan includes stable derivations rather than freeform guesses.

## 6. Critical Constraints / Operating Envelope

- Constraint: The mechanism must remain inside the derived operating limits.

## 7. Cost & Weight Budget

- Estimated unit cost: $10
- Estimated weight: 100g

## 8. Risk Assessment

- Low complexity.
