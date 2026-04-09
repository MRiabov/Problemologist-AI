# TODO List

## Phase 1: Setup

- [x] Read benchmark_definition.yaml and extract key constraints
- [x] Read plan.md and understand solution approach
- [x] Note: goal_zone [500, -12, 10] to [530, 12, 60], build_zone [-140, -160, 0] to [560, 160, 220], spawn [-80, 0, 145], jitter [10, 10, 5]

## Phase 2: Build Parts

- [x] Create ramp_base - Aluminum 6061 base plate for ramp structure
- [x] Create ramp_surface - HDPE long sloped surface guiding ball from spawn to goal zone
- [x] Create left_wall - HDPE left retaining wall preventing ball escape
- [x] Create right_wall - HDPE right retaining wall preventing ball escape
- [x] Create capture_lip - HDPE upper lip capturing ball at ramp entry
- [x] Create exit_funnel - HDPE narrowing funnel at ramp exit aligned to narrow goal zone

## Phase 3: Assemble

- [x] Position parts according to Assembly Strategy
- [x] Verify all parts within build_zone bounds
- [x] Check for geometric intersections

## Phase 4: Validate

- [x] Run validate_geometry() - must pass
- [x] Run validate_and_price() - check cost/weight limits
- [x] Run simulation with default spawn
- [x] Run simulation with jitter edge cases

## Phase 5: Submit

- [x] Final simulation passes consistently
- [x] Cost within budget: $55.75 / $100.00 planner target
- [x] Weight within budget: 850.2 g / 1800.0 g planner target
- [x] Submit for review
