# Engineering Plan

<!-- 
This plan describes HOW you will solve the benchmark scenario defined in objectives.yaml.
The plan must respect ALL constraints: build_zone boundaries, max_unit_cost, max_weight.
Think mechanically: slopes guide gravity, walls channel flow, motors provide energy.
-->

## 1. Solution Overview

<!--
Describe your mechanical solution strategy in 2-3 sentences.
Focus on the CORE MECHANISM that guides the moved_object to the goal_zone.

EXAMPLE (Ball-to-Funnel Problem):
> A sloped ramp guides the ball from the spawn position toward a central funnel.
> The funnel narrows to channel the ball precisely into the goal zone below.
> Side walls prevent escape paths and ensure robust capture across jitter variations.
-->

- **Core Mechanism**: [e.g., Gravity-fed ramp with funnel collection]
- **Key Principle**: [e.g., Potential energy conversion, channeled flow, rotational sorting]
- **Robustness Strategy**: [e.g., Wide capture area, tapered guides, retry paths]

## 2. Parts List

<!--
List each part you will create. For EACH part, specify:
- Name (used in code as variable)
- Approximate dimensions (in mm)
- Material (must be compatible with manufacturing method)
- Purpose in the mechanism

CRITICAL: All parts must fit within the build_zone from objectives.yaml.
CRITICAL: Total weight must not exceed max_weight from objectives.yaml.

EXAMPLE:
| Part | Dimensions (mm) | Material | Purpose |
|------|-----------------|----------|---------|
| main_ramp | 80 × 40 × 5 | PLA | Guides ball from spawn toward funnel |
| funnel | Ø60 top → Ø15 bottom, H=30 | PLA | Collects ball and channels to goal |
| left_wall | 80 × 20 × 3 | PLA | Prevents ball escape on left side |
| right_wall | 80 × 20 × 3 | PLA | Prevents ball escape on right side |
-->

| Part | Dimensions (mm) | Material | Purpose |
|------|-----------------|----------|---------|
| [part_name] | [L × W × H] | [material] | [function] |

**Estimated Total Weight**: [X] g (max: see constraints.max_weight)
**Estimated Total Cost**: $[X] (max: see constraints.max_unit_cost)

## 3. Assembly Strategy

<!--
Describe how parts connect together and their spatial relationships.
Use RELATIVE positioning from a reference point (usually the goal_zone or center).
The CAD agent will need precise coordinates to position parts.

EXAMPLE:
1. Position the funnel CENTERED ABOVE the goal_zone, with bottom edge at goal_zone.max.z + 5mm
2. Attach main_ramp to funnel rim, sloping UP toward moved_object.start_position
3. Place left_wall along -X edge of main_ramp, extending 20mm above ramp surface
4. Place right_wall along +X edge of main_ramp, extending 20mm above ramp surface
5. Verify all parts are WITHIN build_zone boundaries

KEY COORDINATES (from objectives.yaml):
- Goal zone: [min] to [max]
- Build zone: [min] to [max]
- Ball spawn: [X, Y, Z] ± [jitter]
-->

1. [First assembly step with coordinates]
2. [Second assembly step]
3. [Position verification step]

## 4. Cost & Weight Budget

<!--
Break down estimated costs and weights. Use the manufacturing config for pricing.
Leave margin for iteration (aim for 80% of limits).

EXAMPLE:
| Item | Volume (cm³) | Weight (g) | Cost ($) |
|------|--------------|------------|----------|
| main_ramp | 16.0 | 19.2 | 2.40 |
| funnel | 8.5 | 10.2 | 1.28 |
| left_wall | 4.8 | 5.8 | 0.72 |
| right_wall | 4.8 | 5.8 | 0.72 |
| **TOTAL** | 34.1 | 41.0 | **$5.12** |

Constraints from objectives.yaml:
- max_unit_cost: $[X]
- max_weight: [X] g
-->

| Item | Volume (cm³) | Weight (g) | Cost ($) |
|------|--------------|------------|----------|
| [part] | [vol] | [wt] | [cost] |
| **TOTAL** | | | |

**Budget Margin**: [X]% remaining (recommended: keep ≥20% margin)

## 5. Risk Assessment

<!--
Identify what could go wrong and your mitigation strategy.
Focus on GEOMETRIC risks (ball escaping, not reaching goal) and 
MANUFACTURING risks (parts too thin, overhangs, cost overruns).

EXAMPLE:
| Risk | Likelihood | Impact | Mitigation |
|------|------------|--------|------------|
| Ball bounces off ramp | Medium | High | Add lip at ramp edges, roughen surface |
| Ball gets stuck in funnel | Low | High | Increase funnel taper angle to 45° |
| Cost exceeds budget | Medium | Medium | Use thinner walls (2.5mm vs 3mm) |
| Ball escapes via jitter | High | High | Extend capture area beyond jitter range |
-->

| Risk | Likelihood | Impact | Mitigation |
|------|------------|--------|------------|
| [risk description] | [Low/Med/High] | [Low/Med/High] | [mitigation strategy] |

### Jitter Robustness Check

<!--
The moved_object has runtime_jitter and static_randomization.
Your solution MUST work across ALL possible spawn variations.

CALCULATION:
- Spawn position: [X, Y, Z] from objectives.yaml
- Runtime jitter: ±[X, Y, Z] mm
- Static randomization: [describe any radius/mass variations]

Your capture mechanism must cover:
- X range: [spawn.X - jitter.X] to [spawn.X + jitter.X]
- Y range: [spawn.Y - jitter.Y] to [spawn.Y + jitter.Y]
- Z range: [spawn.Z - jitter.Z] to [spawn.Z + jitter.Z]
-->

- Capture area covers spawn jitter: ☐ Yes / ☐ No
- Tested edge cases considered: [list which corners/extremes you've accounted for]
