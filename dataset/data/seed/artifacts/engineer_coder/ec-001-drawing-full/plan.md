# Engineering Plan

## 1. Solution Overview

Use a freestanding `base_plate` with a wide `entry_funnel`, a single powered
`roller_bed`, a passive `idler_guide`, and a shallow `goal_tray` to catch the
seeded `projectile_ball` and move it laterally into the goal capture volume.
The benchmark-owned `environment_fixture` stays read-only, the downstream
contact path stays centered on the single `transfer_lane`, and the only
powered axis remains the `ServoMotor_DS3218` drive.

## 2. Parts List

| Part | Dimensions (mm) | Material | Purpose |
| -- | -- | -- | -- |
| base_plate | 980 x 170 x 10 | aluminum_6061 | Freestanding base that keeps the mechanism stable without drilling into the environment |
| entry_funnel | 180 x 140 x 40 | hdpe | Wide capture pocket that absorbs the seeded spawn jitter and centers the ball onto the driven lane |
| roller_bed | 780 x 70 x 28 | hdpe | Main lane that carries the driven roller shaft and side guides |
| idler_guide | 780 x 18 x 24 | hdpe | Upper passive guide that keeps the ball centered over the roller lane |
| goal_tray | 150 x 120 x 35 | hdpe | End pocket that slows the ball and contains it inside the goal zone |
| ServoMotor_DS3218 | COTS | steel/plastic | Single drive motor for the lateral transfer roller |

**Estimated Total Weight**: 608.34 g
**Estimated Total Cost**: $62.00

## 3. Assembly Strategy

1. Place `base_plate` centered on the build-zone floor and keep the full
   footprint inside the seeded build bounds.
2. Mount `entry_funnel` near the spawn side so its mouth covers the full
   runtime-jitter envelope before tapering into the roller lane.
3. Mount `roller_bed` on the base with one driven roller axis that pushes
   toward positive X while `idler_guide` keeps the ball centered and prevents
   lift-out.
4. Mount `goal_tray` so its interior volume overlaps the seeded `goal_zone`
   in the lower capture band. The placement target is roughly
   `(430.0, 80.0, 37.5)` mm, which yields an `x` span of `355.0-505.0` mm,
   a `y` span of `20.0-140.0` mm, and a `z` span of `20.0-55.0` mm.
5. Keep the `ServoMotor_DS3218` and wiring on the left side of the base so
   downstream electronics work can route power without crossing the moving
   lane. A placement near `(-120.0, 80.0, 37.5)` mm preserves the left-side
   corridor while staying clear of the goal tray and roller lane.
6. The drafting sheet callouts `1`-`6` track the `base_plate`,
   `entry_funnel`, `roller_bed`, `idler_guide`, `goal_tray`, and the
   `ServoMotor_DS3218` drive, respectively.
7. Treat the coarse `motion_forecast` in `assembly_definition.yaml` as the
   reviewable start-to-finish trajectory for `transfer_lane`: it begins in a
   build-safe parked pose and ends with goal-zone entry.
8. Narrow that same `transfer_lane` motion again in
   `precise_path_definition.yaml`, keeping the moving-part set unchanged while
   tightening the cadence and explicit goal-contact proof.
9. If the current 2° downhill lane geometry is kept, the ball’s ideal rolling
   exit speed is `v = sqrt((10/7) * g * sin(2°) * 0.780 m) ≈ 0.618 m/s`, with
   an average travel speed of about `0.309 m/s` over the 780 mm lane. Keep
   the real motion in the 0.55-0.65 m/s exit band so it is brisk without
   overshooting the goal tray; the earlier 175-190 mm/s anchor interpolation
   rate is only the coarse lane-motion timeline, not the ball’s physical
   speed.

## 4. Assumption Register

| ID | Assumption | Source | Used By |
| -- | -- | -- | -- |
| ASSUMP-001 | `projectile_ball` static radius can reach 40.0 mm in the worst case. | `benchmark_definition.yaml:moved_object.static_randomization.radius` | `CALC-001`, `CALC-004` |
| ASSUMP-002 | Runtime spawn jitter is bounded by `runtime_jitter: [12.0, 8.0, 5.0]`. | `benchmark_definition.yaml:moved_object.runtime_jitter` | `CALC-001` |
| ASSUMP-003 | The `ServoMotor_DS3218` is the only powered DOF and the lane remains a single-axis transfer. | `assembly_definition.yaml:final_assembly` | `CALC-002`, `CALC-003`, `CALC-004` |
| ASSUMP-004 | Material densities are the current manufacturing-config values: aluminum_6061 = 2.7 g/cm^3 and hdpe = 0.95 g/cm^3. | `config/manufacturing_config.yaml` | `CALC-003` |
| ASSUMP-005 | The seeded `goal_zone` is the benchmark capture target and the ball only needs to settle into its lower band, not occupy the full 85 mm height at once. | `benchmark_definition.yaml:objectives.goal_zone` | `CALC-004` |

## 5. Detailed Calculations

| ID | Problem / Decision | Result | Impact |
| -- | -- | -- | -- |
| CALC-001 | Size the `entry_funnel` mouth to cover the worst-case ball envelope plus runtime jitter. | Worst-case envelope is 104 mm in X and 96 mm in Y; the 180 x 140 mm mouth leaves 76 mm and 44 mm of margin. | The funnel captures the seeded spawn variation without needing a wider base plate. |
| CALC-002 | Verify that the `base_plate` is long and heavy enough to keep the transfer lane stable. | The plate aspect ratio is 980 / 170 = 5.76, and the base mass is 449.82 g, which is 73.97 percent of the full assembly mass. | The freestanding solution stays stable without drilling or anchoring to the benchmark fixture. |
| CALC-003 | Compute the exact priced assembly totals from the part breakdown. | Exact totals are 62.00 USD and 608.34 g, which are below the 67.00 USD / 1300 g planner target. | The plan stays within budget with deterministic headroom. |
| CALC-004 | Check that `goal_tray` can overlap the seeded goal volume while still damping rebound. | With the tray centered at `(430.0, 80.0, 37.5)`, the tray spans `x=[355.0, 505.0]`, `y=[20.0, 140.0]`, `z=[20.0, 55.0]`; this overlaps the goal zone by `75.0 mm` in X, `35.0 mm` in Y, and the full `35.0 mm` in Z, while leaving `25.0 mm` of X headroom before the goal-zone max. | The ball can settle in the goal capture region instead of bouncing through it. |

### CALC-001: Capture envelope

#### Problem Statement

The `entry_funnel` must accept the largest seeded ball and the full runtime
jitter envelope without forcing a relayout of the transfer lane.

#### Assumptions

- `ASSUMP-001`: the ball radius can reach 40.0 mm.
- `ASSUMP-002`: runtime jitter is bounded by 12.0 mm in X, 8.0 mm in Y, and
  5.0 mm in Z.

#### Derivation

- Worst-case ball diameter: `2 * 40.0 = 80.0 mm`.
- Required X envelope: `80.0 + 2 * 12.0 = 104.0 mm`.
- Required Y envelope: `80.0 + 2 * 8.0 = 96.0 mm`.
- Funnel mouth margin in X: `180.0 - 104.0 = 76.0 mm`.
- Funnel mouth margin in Y: `140.0 - 96.0 = 44.0 mm`.
- Funnel mouth area: `180.0 * 140.0 = 25200.0 mm^2`.
- Worst-case ball envelope area: `104.0 * 96.0 = 9984.0 mm^2`.

#### Worst-Case Check

- The mouth margins remain positive in both lateral axes.
- The mouth area exceeds the worst-case envelope area by
  `25200.0 - 9984.0 = 15216.0 mm^2`.

#### Result

The funnel mouth is large enough to capture the seeded ball across the full
runtime jitter envelope.

#### Design Impact

Keep the `entry_funnel` mouth at the current size and preserve the centered
handoff into `roller_bed`.

#### Cross-References

- `plan.md#4-assumption-register`
- `plan.md#6-critical-constraints--operating-envelope`
- `plan.md#7-cost--weight-budget`

### CALC-002: Base stability

#### Problem Statement

The freestanding base must resist tipping while carrying the roller lane and
motor corridor.

#### Assumptions

- `ASSUMP-003`: the transfer lane is a single powered axis and the rest of the
  mechanism is passive.
- The `base_plate` mass is determined from its aluminum volume and the current
  manufacturing density.

#### Derivation

- Base footprint area: `980.0 * 170.0 = 166600.0 mm^2`.
- Footprint aspect ratio: `980.0 / 170.0 = 5.764706`.
- Base volume: `166600.0 mm^3 = 166.6 cm^3`.
- Base mass: `166.6 * 2.7 = 449.82 g`.
- Remaining non-base mass: `608.34 - 449.82 = 158.52 g`.

#### Worst-Case Check

- The base carries more mass than the rest of the assembly combined.
- The long, thin footprint keeps the center of mass low and spread along the
  support axis.

#### Result

The base plate geometry is sufficient for a freestanding build without
fasteners into the benchmark fixture.

#### Design Impact

Preserve the full `base_plate` length and do not shorten it to the point where
the transfer lane loses stability.

#### Cross-References

- `plan.md#2-parts-list`
- `plan.md#3-assembly-strategy`
- `plan.md#6-critical-constraints--operating-envelope`

### CALC-003: Exact budget total

#### Problem Statement

The priced assembly must match the deterministic totals written in
`assembly_definition.yaml`.

#### Assumptions

- `ASSUMP-004`: aluminum_6061 density is 2.7 g/cm^3 and hdpe density is
  0.95 g/cm^3.
- The `ServoMotor_DS3218` weight is the catalog value carried in the priced
  assembly.

#### Derivation

- `base_plate`: `166.6 cm^3 * 2.7 = 449.82 g` and `$15.50`.
- `entry_funnel`: `30.2 cm^3 * 0.95 = 28.69 g` and `$6.00`.
- `roller_bed`: `39.0 cm^3 * 0.95 = 37.05 g` and `$8.50`.
- `idler_guide`: `13.5 cm^3 * 0.95 = 12.83 g` and `$5.00`.
- `goal_tray`: `21.0 cm^3 * 0.95 = 19.95 g` and `$9.00`.
- `ServoMotor_DS3218`: `60.00 g` and `$18.00`.
- Exact total weight: `449.82 + 28.69 + 37.05 + 12.83 + 19.95 + 60.00 = 608.34 g`.
- Exact total cost: `15.50 + 6.00 + 8.50 + 5.00 + 9.00 + 18.00 = 62.00 USD`.
- Planner target margin: `67.00 - 62.00 = 5.00 USD`.
- Planner target weight margin: `1300.00 - 608.34 = 691.66 g`.

#### Worst-Case Check

- The exact totals are deterministic and cent-accurate.
- The plan remains below both the benchmark caps and the planner targets.

#### Result

The priced assembly is valid and leaves enough headroom for the coder stage.

#### Design Impact

Do not add hidden parts, duplicate the motor, or change the part sizes without
re-running the price and weight contract.

#### Cross-References

- `plan.md#2-parts-list`
- `plan.md#7-cost--weight-budget`
- `assembly_definition.yaml`

### CALC-004: Goal capture overlap

#### Problem Statement

The `goal_tray` must be able to sit in the goal capture region without
overshooting the benchmark volume or losing the ball on rebound.

#### Assumptions

- `ASSUMP-005`: the ball only needs to settle into the lower band of the goal
  zone.
- The `goal_tray` can be seated so its opening intersects the lower portion of
  the goal volume.

#### Derivation

- Goal zone X span: `530.0 - 430.0 = 100.0 mm`.
- Goal zone Y span: `55.0 - (-55.0) = 110.0 mm`.
- Goal zone Z span: `105.0 - 20.0 = 85.0 mm`.
- Tray center placement: `(430.0, 80.0, 37.5)` mm.
- Tray X span: `430.0 - 75.0 = 355.0 mm` to `430.0 + 75.0 = 505.0 mm`.
- Tray Y span: `80.0 - 60.0 = 20.0 mm` to `80.0 + 60.0 = 140.0 mm`.
- Tray Z span: `37.5 - 17.5 = 20.0 mm` to `37.5 + 17.5 = 55.0 mm`.
- X overlap with goal zone: `min(505.0, 530.0) - max(355.0, 430.0) = 75.0 mm`.
- Y overlap with goal zone: `min(140.0, 55.0) - max(20.0, -55.0) = 35.0 mm`.
- Z overlap with goal zone: `min(55.0, 105.0) - max(20.0, 20.0) = 35.0 mm`.

#### Worst-Case Check

- The tray footprint exceeds the goal-zone footprint in both planar axes.
- The tray height leaves room for the ball to dissipate energy before rebound.

#### Result

The goal tray can occupy the capture region without needing a larger or more
complex end effector.

#### Design Impact

Keep the tray shallow and wide, and preserve the goal-zone overlap window
while maintaining a clear lane corridor.

#### Cross-References

- `plan.md#3-assembly-strategy`
- `plan.md#6-critical-constraints--operating-envelope`
- `plan.md#8-risk-assessment`

## 6. Critical Constraints / Operating Envelope

| Limit ID | Limit | Bound | Basis |
| -- | -- | -- | -- |
| LIMIT-001 | `entry_funnel` capture margin | At least 76.0 mm X margin and 44.0 mm Y margin beyond the worst-case ball envelope | `CALC-001` |
| LIMIT-002 | `base_plate` stability | Keep the 980 x 170 mm footprint and the 449.82 g aluminum base mass unchanged | `CALC-002` |
| LIMIT-003 | Budget ceiling | Stay at or below 62.00 USD and 608.34 g, with no hidden parts or duplicate motors | `CALC-003` |
| LIMIT-004 | Goal capture geometry | Keep the `goal_tray` footprint at least 150 x 120 mm and seat it in the lower band of the goal zone near `x=430 mm` | `CALC-004` |

## 7. Cost & Weight Budget

| Item | Volume (cm^3) | Weight (g) | Cost ($) |
| -- | -- | -- | -- |
| base_plate | 166.6 | 449.82 | 15.50 |
| entry_funnel | 30.2 | 28.69 | 6.00 |
| roller_bed | 39.0 | 37.05 | 8.50 |
| idler_guide | 13.5 | 12.83 | 5.00 |
| goal_tray | 21.0 | 19.95 | 9.00 |
| ServoMotor_DS3218 | n/a | 60.00 | 18.00 |
| **TOTAL** | 270.3 | 608.34 | **62.00** |

**Budget Margin**: 5.00 USD and 691.66 g below the planner target; 23.00 USD
and 991.66 g below the benchmark caps.

## 8. Risk Assessment

| Risk | Likelihood | Impact | Mitigation |
| -- | -- | -- | -- |
| Ball misses the driven lane entry under corner jitter | Medium | High | Oversize the funnel mouth beyond the seeded jitter envelope and taper smoothly into the roller lane |
| Roller speed launches the ball past the goal tray | Medium | High | Keep the roller speed low, keep an upper idler guide, and terminate in a wider goal tray below the lane exit |
| Freestanding base tips under impact | Low | Medium | Use a long aluminum base plate with most mass close to the floor |
| Wiring crosses moving hardware | Low | High | Keep the `ServoMotor_DS3218` on the left edge and reserve a fixed cable corridor outside the transfer lane |

### Jitter Robustness Check

- Capture area covers spawn jitter: Yes
- Tested edge cases considered: left-most spawn, right-most spawn, low-Z
  spawn, high-Z spawn, corner-spawn envelope, and worst-case ball radius
