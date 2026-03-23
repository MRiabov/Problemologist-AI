# TODO List - Benchmark Generator

<!--
This is your execution checklist for creating a valid benchmark.
Mark items [x] when complete.
-->

## Phase 1: Planning

- [ ] Define Learning Objective in `plan.md`
- [ ] Define Geometry (Static + Moving Parts) in `plan.md`
- [ ] Define Input Object & Objectives in `plan.md`
- [ ] Set Constraints & Randomization in `plan.md`

## Phase 2: Implementation (CAD)

- [ ] Create Static Environment (Walls, Base)
- [ ] Create Moving Parts (Motors, Sliders)
- [ ] Record the explicit motion axis, travel limit, and controller facts for each moving fixture
- [ ] Create Input Object (Projectile)
- [ ] Define Objectives (Goal Zone, Forbid Zones)
- [ ] Export to MJCF

## Phase 3: Validation

- [ ] Verify Geometry (No intersections)
- [ ] Verify Simulation Stability
- [ ] Verify Randomization passes parameters
- [ ] Check Constraints (Volume, etc.)

## Phase 4: Submission

- [ ] Final Review (Self-Correction)
- [ ] Fail closed if motion cannot be explained with explicit bounds and controller facts
- [ ] Submit for Engineer
