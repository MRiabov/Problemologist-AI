# TODO List

## Phase 1: Setup

- [x] Read benchmark_definition.yaml and extract key constraints
- [x] Read plan.md and understand solution approach
- [x] Note: goal_zone [360, -55, 15] to [470, 55, 100], build_zone [-460, -150, 0] to [520, 150, 220], spawn [-400, 0, 105], jitter [8, 6, 4], forbid zone center_speed_bump [60, -70, 0] to [150, 70, 55]

## Phase 2: Build Parts

- [x] Create ramp_base - Aluminum 6061 base plate for ramp structure
- [x] Create bridge_support - Aluminum elevated support clearing the forbid zone
- [x] Create ramp_surface - HDPE long sloped surface guiding ball from spawn to goal zone
- [x] Create left_wall - HDPE left retaining wall preventing ball escape
- [x] Create right_wall - HDPE right retaining wall preventing ball escape
- [x] Create capture_lip - HDPE upper lip capturing ball at ramp entry
- [x] Create goal_funnel - HDPE funnel at ramp exit aligned to goal zone

## Phase 3: Assemble

- [x] Position parts according to Assembly Strategy
- [x] Verify all parts within build_zone bounds
- [x] Check for geometric intersections
- [x] Verify bridge clears center_speed_bump forbid zone

## Phase 4: Validate

- [x] Run validate_geometry() - must pass
- [x] Run validate_and_price() - check cost/weight limits
- [x] Run simulation with default spawn
- [x] Run simulation with jitter edge cases

## Phase 5: Submit

- [x] Final simulation passes consistently
- [x] Cost within budget: $62.50 / $100.00 planner target
- [x] Weight within budget: 980.5 g / 1800.0 g planner target
- [x] Submit for review
