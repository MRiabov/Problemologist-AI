# Engineering Plan

## 1. Solution Overview

Use a passive catch-and-deflect chute that intercepts the projectile ball below its elevated start point, steers it around the seeded direct-drop dead zone, and drops it into the lower bin goal zone. The mechanism relies on fixed geometry only and keeps the ball away from the central dead-zone volume.

## 2. Parts List

| Part           | Dimensions (mm) | Material      | Purpose                                                        |
| -------------- | --------------- | ------------- | -------------------------------------------------------------- |
| catch_plate    | 260 x 130 x 10  | aluminum_6061 | Upper catch plate intercepting the falling ball                |
| left_wall      | 210 x 18 x 55   | hdpe          | Left wall of the deflection chute                              |
| right_wall     | 210 x 18 x 55   | hdpe          | Right wall of the deflection chute                             |
| turn_lip       | 120 x 70 x 18   | hdpe          | Curved lip steering the ball away from the dead zone           |
| lower_funnel   | 150 x 110 x 30  | hdpe          | Funnel guiding the ball into the lower bin                     |
| support_column | 180 x 30 x 120  | aluminum_6061 | Column holding the catch plate at the correct intercept height |

**Estimated Total Weight**: 930 g
**Estimated Total Cost**: $43.75

## 3. Assembly Strategy

1. Mount `catch_plate` on `support_column` so it intercepts the seeded falling ball before the trajectory enters `direct_drop_dead_zone`.
2. Mount `left_wall`, `right_wall`, and `turn_lip` on the catch plate to steer the ball toward the lower-bin side of the workspace.
3. Mount `lower_funnel` overlapping the seeded goal zone so the ball settles in the lower bin instead of bouncing out.

## 4. Cost & Weight Budget

| Item           | Weight (g) | Cost ($)  |
| -------------- | ---------- | --------- |
| catch_plate    | 220        | 12.5      |
| left_wall      | 17         | 5.0       |
| right_wall     | 17         | 5.0       |
| turn_lip       | 12         | 4.5       |
| lower_funnel   | 23         | 6.75      |
| support_column | 260        | 10.0      |
| **TOTAL**      | **549**    | **43.75** |

**Budget Margin**: 23% remaining versus the planner target.

## 5. Risk Assessment

| Risk                                           | Likelihood | Impact | Mitigation                                                                       |
| ---------------------------------------------- | ---------- | ------ | -------------------------------------------------------------------------------- |
| Ball misses the catch plate under spawn jitter | Medium     | High   | Oversize the catch plate relative to the seeded jitter envelope                  |
| Ball re-enters the dead zone after deflection  | Medium     | High   | Use a curved turn lip and chute walls that keep the path offset from the keepout |
| Ball rebounds out of the lower bin             | Low        | Medium | Use a funnel insert and goal overlap at the final pocket                         |
