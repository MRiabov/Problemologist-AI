# Description of ec-001 proposed fixture.

1. The environment is a smoke test meant to test that an agent is able to reason that an object must move over the goal.

## Benchmark plan:

All coordinates are at 0,0,0 with bottom at center of the bottom fixture.

1. Suppose the environment is 2m long (Y), 1m wide (x), 2m tall (z),
2. there is a forbid zone with bottom center at (0,0,0) of size (0.5, 1, 0.5), so it is at center of the eval space, 0,0,0, spanning to the full width of the env.

- forbid zone is surrounded by two walls from Y axis.

3. The payload would be spawned at 0, -0.75, 1.5, so behind and above the center
4. the goal at about (0, 0.75, 0.25) center with size being (1, 0.5, 0.5)
5. There is also floor of the fixture with a slab.
6. The payload is a ball that can be rolled easily.

# Solution plan

To move the payload from starting point to finishing point using gravity only. The object will move by a long, thin plate with two objects at sides. notably, the whole solution takes exactly one part to solve. Manufacturing is CNC 3d printing. Cost relatively unconstrained.
The decline is about 30 degrees. Width of the plate is about 0.75.
Simply move the payload into the zone.

The engineering planner and coders will have to be, as usual. very exact as on how they expect the ball to roll.
