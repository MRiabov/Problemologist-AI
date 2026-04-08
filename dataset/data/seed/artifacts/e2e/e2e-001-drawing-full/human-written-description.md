# E2E smoke benchmark prompt

This is a deliberately simplified smoke benchmark for exercising the full local benchmark -> engineer pipeline.

1. Treat the scene as a compact passive bridge-gap benchmark with the origin at the bottom center of the floor fixture.
2. Place a small central forbid zone around the origin and keep the right-side goal zone unobstructed.
3. Add a payload ball spawned on the left side so it starts above the left deck and must move toward the right deck.
4. Include a left start deck, a right goal deck, a central reference block, and a low floor guard to keep the geometry readable.
5. The intended solution is intentionally easy: a passive bridge or shallow transfer surface should roll the ball from the left deck into the right goal zone using gravity only.
6. For this smoke run, the prompt may be solution-revealing. The purpose is to force the system to progress through planner, reviewer, coder, and final review stages end-to-end, not to stress benchmark realism.
