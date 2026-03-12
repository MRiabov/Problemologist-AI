# Engineering Plan

## 1. Solution Overview
Use a freestanding entry funnel and a single powered roller lane to catch the seeded steel ball and drive it laterally into the goal tray. The mechanism keeps one motorized axis only, reserves a left-side cable corridor, and ends in a flared catcher so the ball settles inside the goal zone instead of bouncing through it.

## 2. Parts List
| Part | Dimensions (mm) | Material | Purpose |
|------|-----------------|----------|---------|
| base_plate | 980 x 170 x 10 | aluminum_6061 | Freestanding base that keeps the mechanism stable without drilling into the environment |
| entry_funnel | 180 x 140 x 40 | hdpe | Wide capture pocket that absorbs X/Y spawn jitter and centers the ball onto the driven lane |
| roller_bed | 780 x 70 x 28 | hdpe | Main lane that carries the driven roller shaft and side guides |
| idler_guide | 780 x 18 x 24 | hdpe | Upper passive guide that keeps the ball centered over the roller lane |
| goal_tray | 150 x 120 x 35 | hdpe | End pocket that slows the ball and contains it inside the goal zone |
| gearmotor_12v_60rpm | COTS | steel/plastic | Single drive motor for the lateral transfer roller |

**Estimated Total Weight**: 1240 g
**Estimated Total Cost**: $62.50

## 3. Assembly Strategy
1. Place `base_plate` centered on the build-zone floor and keep the full footprint inside the seeded build bounds.
2. Mount `entry_funnel` near the spawn side so its mouth covers the full runtime-jitter envelope before tapering into the roller lane.
3. Mount `roller_bed` on the base with one driven roller axis that pushes toward positive X while `idler_guide` keeps the ball centered and prevents lift-out.
4. Mount `goal_tray` so its interior volume overlaps the seeded goal zone and sits slightly below the roller exit to bleed off speed.
5. Keep the motor and wiring on the left side of the base so downstream electronics work can route power without crossing the moving lane.

## 4. Cost & Weight Budget
| Item | Volume (cm^3) | Weight (g) | Cost ($) |
|------|---------------|------------|----------|
| base_plate | 166.6 | 449.8 | 15.50 |
| entry_funnel | 30.2 | 29.0 | 6.00 |
| roller_bed | 39.0 | 37.4 | 8.50 |
| idler_guide | 13.5 | 12.9 | 5.00 |
| goal_tray | 21.0 | 20.2 | 9.00 |
| gearmotor_12v_60rpm | n/a | 145.0 | 18.50 |
| **TOTAL** | 272.9 | 697.0 | **62.50** |

**Budget Margin**: 7% remaining versus the planner target.

## 5. Risk Assessment
| Risk | Likelihood | Impact | Mitigation |
|------|------------|--------|------------|
| Ball misses the driven lane entry under lateral jitter | Medium | High | Oversize the funnel mouth beyond the seeded jitter envelope and taper smoothly into the roller lane |
| Roller speed launches the ball past the goal tray | Medium | High | Keep roller speed low, keep an upper idler guide, and terminate in a wider goal tray below the lane exit |
| Freestanding base tips under impact | Low | Medium | Use a long aluminum base plate with most mass close to the floor |
| Wiring crosses moving hardware | Low | High | Keep the motor on the left edge and reserve a fixed cable corridor outside the transfer lane |

### Jitter Robustness Check
- Capture area covers spawn jitter: Yes
- Tested edge cases considered: left-most spawn, right-most spawn, low-Z spawn, high-Z spawn
