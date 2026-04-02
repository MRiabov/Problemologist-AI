## 1. Learning Objective

Test whether benchmark-owned motorized lift hardware can raise a ball from floor height onto a shelf while keeping the shelf-support keep-out clear and preserving the class-aware COTS import path.

## 2. Geometry

- `floor_plate`: fixed base plate centered near `[20, 0, 10]` mm.
- `support_tower`: vertical tower centered near `[230, 0, 105]` mm.
- `raised_goal_shelf`: raised target shelf centered near `[375, 0, 235]` mm.
- `lift_carriage`: moving plate centered near `[-130, 0, 60]` mm with a single `slide_z` axis.
- `drive_motor`: benchmark-owned servo fixture imported through `ServoMotor.from_catalog_id("ServoMotor_DS3218")`.

## 3. Objectives

- Move the projectile ball into the goal zone on the shelf.
- Keep the ball out of the shelf-support clearance keep-out.
- Do not add any benchmark-side motion beyond the single lift carriage axis.

## 4. Electronics and COTS Fixture

- The benchmark includes one benchmark-owned servo fixture, `ServoMotor_DS3218`.
- Preserve the benchmark-owned power supply and wiring constraints from the planner handoff.
- Keep the drive motor read-only to the engineer; it is fixture geometry, not solution-owned geometry.

## 5. Randomization

- Static radius jitter for the projectile ball.
- Runtime position jitter for the projectile ball.
- Keep motion bounds robust under the seeded jitter ranges.

## 6. Implementation Notes

- Use Build123d primitives.
- Preserve `benchmark_plan_evidence_script.py` and `benchmark_plan_technical_drawing_script.py` as read-only planner context.
- Keep `benchmark_script.py` import-safe with no `__main__` block and no in-module review submission call.
- Run validate/simulate/review submission only from external shell self-check commands.
