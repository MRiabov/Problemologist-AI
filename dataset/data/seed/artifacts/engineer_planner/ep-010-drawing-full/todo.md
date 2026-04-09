# TODO List

## Phase 1: Setup

- [x] Read benchmark_definition.yaml and extract key constraints
- [x] Read plan.md and understand solution approach
- [x] Note: goal_zone [220, -45, 10] to [320, 45, 85], build_zone [-120, -160, 0] to [360, 160, 280], spawn [-20, 0, 220], jitter [8, 8, 6], forbid zone direct_drop_dead_zone [-60, -45, 0] to [60, 45, 130]

## Phase 2: Build Parts

- [x] Create ramp_base - Aluminum 6061 base plate for ramp structure
- [x] Create ramp_surface - HDPE sloped surface guiding ball from spawn around forbid zone to goal
- [x] Create left_wall - HDPE left retaining wall preventing ball escape
- [x] Create right_wall - HDPE right retaining wall preventing ball escape
- [x] Create capture_lip - HDPE upper lip capturing ball at ramp entry
- [x] Create goal_funnel - HDPE funnel at ramp exit aligned to goal zone

## Phase 3: Assemble

- [x] Position parts according to Assembly Strategy
- [x] Verify all parts within build_zone bounds
- [x] Check for geometric intersections
- [x] Verify ramp clears direct_drop_dead_zone forbid zone

## Phase 4: Validate

- [x] Run validate_geometry() - must pass
- [x] Run validate_and_price() - check cost/weight limits
- [x] Run simulation with default spawn
- [x] Run simulation with jitter edge cases

## Phase 5: Submit

- [x] Final simulation passes consistently
- [x] Cost within budget: $50.00 / $100.00 planner target
- [x] Weight within budget: 620.3 g / 1800.0 g planner target
- [x] Submit for review
