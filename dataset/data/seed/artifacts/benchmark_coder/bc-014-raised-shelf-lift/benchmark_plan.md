## 1. Learning Objective

Test whether an engineer can design a mechanism that moves a payload from floor level onto a raised shelf while keeping a shelf-support keepout volume clear. The benchmark defines the task geometry, objectives, and constraints; the engineer must create a solution that reliably guides the `projectile_ball` into the elevated `goal_zone` without entering the `shelf_support_clearance` forbid zone.

## 2. Geometry

All dimensions in mm. Positions are world-frame centers unless noted.

- `environment_fixture`: fixed benchmark-owned floor plate centered near `(105, 0, 5)`, size `730 × 360 × 10`. Provides the mounting surface and reference frame for the benchmark.
- `projectile_ball`: payload, sphere, ABS plastic, radius `[44, 46]` mm, spawn position `(-200, 0, 60)`, runtime jitter `±[10, 8, 6]` mm.
- **Goal zone**: elevated shelf region, AABB min `(320, -55, 220)`, max `(430, 55, 300)`. Success when the projectile ball center enters this volume.
- **Forbid zone** `shelf_support_clearance`: vertical keepout volume, AABB min `(180, -95, 0)`, max `(280, 95, 210)`. Any contact = failure. Represents the structural support volume for a raised shelf that the engineer must not occupy.
- **Build zone**: AABB min `(-260, -180, 0)`, max `(470, 180, 340)`. Engineer solutions must stay within these bounds.
- **Simulation bounds**: AABB min `(-320, -220, -10)`, max `(520, 220, 380)`. Hard simulation boundaries.

## 3. Objectives

- **Goal**: The `projectile_ball` center enters the elevated goal zone AABB.
- **Forbid**: No part of the simulation may contact `shelf_support_clearance`.
- **Build zone**: All engineer-owned geometry must fit within the build zone.
- The benchmark does not prescribe the mechanism. The engineer may use ramps, lifts, conveyors, or any other approach that satisfies the objective.

## 4. Randomization

- Static: projectile ball radius varies in `[44, 46]` mm per benchmark variant. This tests whether the solution handles slight size variation.
- Runtime: projectile ball spawn position jitters by `±[10, 8, 6]` mm per simulation run. The solution must be robust to this positional uncertainty.
- The goal zone and forbid zone are fixed; they do not randomize.

## 5. Implementation Notes

- Use build123d primitives for all benchmark-owned geometry.
- The `environment_fixture` is benchmark-owned and read-only for engineering. It defines the floor plate and provides the reference coordinate frame.
- The `projectile_ball` material is `abs` from `manufacturing_config.yaml`; its mass and friction properties come from the material definition.
- Planner estimate: `$58.00` / `1033 g`. Runtime-derived caps: `$87.00` / `1550 g`.
- Electronics: 24 V DC supply available, max 10 A, max wire length 850 mm, wire restricted from the shelf-support clearance zone, circuit validation required. The benchmark declares `electronics_requirements` so the engineer must plan power delivery and wiring.
- The benchmark does not include moving benchmark-owned fixtures. All motion is engineer-owned.
- Keep `benchmark_plan_evidence_script.py` and `benchmark_plan_technical_drawing_script.py` as read-only planner context.
- Keep `benchmark_script.py` import-safe: no `__main__` block, no in-module review submission call.
- Run validate/simulate/review submission only from external shell self-check commands.
