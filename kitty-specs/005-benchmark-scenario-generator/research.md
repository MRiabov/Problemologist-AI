# Research: Benchmark Scenario Generator

## Benchmark Generation Philosophy

The goal is to create high-quality, randomized physical "puzzles" for evaluative and training purposes.

### 1. Verification Strategy: "Big Bang" Stability

**Decision**: 1000-step simulation validation on the worker.
**Rationale**:

- Every generated scenario must be physically viable.
- Before handover, the scenario is simulated in MuJoCo to ensure no geometric intersections and that the system reaches equilibrium under gravity.

### 2. Randomization Strategy

**Decision**: Non-uniform scaling and jittered placement.
**Rationale**:

- **Volume Scaling**: The benchmark volume size can vary 2x. Goal, forbid, and build zones are rescaled respectively.
- **Position Jitter**: Objects are spawned with +/- 40% position noise to ensure solvers are robust against variable input.
- **Scene Diversity**: The benchmark generator is responsible for ensuring the environment itself is randomized, not just the object positions.

### 3. Handover Protocol

**Decision**: Pre-rendered assets and MJCF handoff.
**Rationale**:

- Upon acceptance, the generator uploads the MJCF, meshes, and 24 multi-view pictures to S3.
- The Engineering Agent receives these as a "fixed" environment it cannot modify.
