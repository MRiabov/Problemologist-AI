## Learning Objective

- Show that a passive rigid-body benchmark can route a sphere into a goal
  zone using only static geometry.
- Keep the scenario simple enough to validate quickly, but specific enough that
  the geometry, input object, and objective zones are unambiguous.

## Geometry

- `environment_fixture`: a compact box-like passive scene shell around the
  origin with a floor, side walls, and a center redirecting surface.
- The static geometry should leave a clear gravity path that is still
  obstructed enough to require deliberate routing.
- No benchmark-owned moving fixtures, motors, or fluids are needed.

## Objectives

- Input object:
  - Shape: `sphere`
  - Label: `projectile_ball`
  - Static randomization: radius in `[1, 2]` mm
  - Nominal start position: `[0, 0, 0]`
  - Runtime jitter: `[0.5, 0.5, 0.5]`
- Goal zone: `min [6, -2, 0]`, `max [10, 2, 4]`
- Forbid zone: `min [3, -3, 0]`, `max [4, 3, 4]`
- Build zone: `min [-10, -10, -10]`, `max [10, 10, 10]`
- Simulation bounds: `min [-30, -30, -30]`, `max [30, 30, 30]`
- Success requires the sphere to reach the goal zone without crossing the
  forbid zone.

## Cost & Weight Budget

- Planner target unit cost: `$25`
- Planner target weight: `600 g`
- Benchmark/customer cap will be copied into the benchmark_definition file.
