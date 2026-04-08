# Description of ec-001 proposed fixture.

1. The environment is a smoke test meant to test that an agent is able to reason that an object must move over the goal.

## Benchmark plan:

All coordinates are at 0,0,0 with bottom at center of the bottom fixture.

1. The environment is 2m long (Y), 1m wide (X), 2m tall (Z).
2. There is a forbid zone with bottom center at (0,0,0) of size (0.5m, 1m, 0.5m) — a red translucent AABB spanning X=[-250,250], Y=[-500,500], Z=[0,500].
3. The payload spawns at (0, -750, 1500) — behind and above the forbid zone.
4. The goal zone is centered at approximately (0, 750, 250) with size (1m, 0.5m, 0.5m) — a green translucent AABB spanning X=[-500,500], Y=[550,1050], Z=[0,500].
5. There is a floor slab (1m × 2m × 20mm aluminum plate at Z=0).
6. The payload is a ball (sphere, 15-20mm radius, ABS material).
7. **No walls or side barriers are present in the benchmark geometry.** The forbid and goal zones are purely AABB volumes with no physical bounding geometry.

# Solution plan

To move the payload from starting point to finishing point using gravity only. The object will move by a long, thin plate with two side guide walls. Notably, the whole solution takes exactly one part to solve. Manufacturing is CNC. Cost relatively unconstrained.
The decline is about 30 degrees. Width of the plate is about 0.75m.
Simply move the payload into the zone.

The engineering planner and coders will have to be, as usual, very exact as to how they expect the ball to roll.
