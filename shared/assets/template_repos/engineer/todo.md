# TODO List

<!--
For the CAD designer engineer (not lead technician/planner):
This is your execution checklist. Update it as you work.
Mark items [x] when complete, [-] when skipped.

WORKFLOW:
1. Read objectives.yaml to understand the goal
2. Read plan.md to understand the solution strategy
3. Implement each part from the Parts List
4. Assemble parts according to Assembly Strategy
5. Validate geometry (build zone, no intersections)
6. Run simulation to verify ball reaches goal
7. If simulation fails, iterate on geometry

TIPS:
- Start with the simplest part first
- Test individual parts before full assembly
- Use validate_geometry() after each major change
- Check build_zone bounds frequently
- Log decisions in journal.md
-->

## Phase 1: Setup

- [ ] Read objectives.yaml and extract key constraints
- [ ] Read plan.md and understand solution approach
- [ ] Note: goal_zone, build_zone, spawn position, jitter values

## Phase 2: Build Parts

- [ ] Create [part_1_name] - [brief purpose]
- [ ] Create [part_2_name] - [brief purpose]
- [ ] ... (add more as needed from plan.md Parts List)

## Phase 3: Assemble

- [ ] Position parts according to Assembly Strategy
- [ ] Verify all parts within build_zone bounds
- [ ] Check for geometric intersections

## Phase 4: Validate

- [ ] Run validate_geometry() - must pass
- [ ] Run validate_and_price() - check cost/weight limits
- [ ] Run simulation with default spawn
- [ ] Run simulation with jitter edge cases

## Phase 5: Submit

- [ ] Final simulation passes consistently
- [ ] Cost within budget: ☐
- [ ] Weight within budget: ☐
- [ ] Submit for review
