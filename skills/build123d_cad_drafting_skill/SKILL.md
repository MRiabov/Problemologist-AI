---
name: build123d_cad_drafting_skill
description: Expert CAD modeling using build123d. Includes builder modes, semantic selectors, and MJCF-compatible boolean strategies.
---

# build123d CAD Drafting Expert

**MANDATORY**: Before planning any `build123d` implementation, you MUST use the `read_skill` tool to read this file (`SKILL.md`). It contains expert knowledge and links to critical patterns.

## Core Directives

1. **Builder Dominance**: Prefer `with BuildPart()`, `with BuildSketch()`, and `with BuildLine()`.
2. **Semantic Selectors**: Avoid indices. Use `faces()`, `edges()`, `vertices()` with `sort_by(Axis.Z)` or `last()`/`first()`.
3. **MJCF Compliance**: Ensure parts are non-intersecting if they belong to different simulation links.
4. **Assembly Labels**: Use `.label = "stator"` and `.label = "rotor"` for automatic motor/joint injection in MJCF.
5. **Label Namespace Hygiene**: Top-level authored labels must be unique and must not be `environment` or start with `zone_`. The simulator reserves those names for the scene root and generated objective bodies, and duplicate labels collide with MJCF mesh/body names.

## Placement And Rotation Contract

- Create top-level authored solids at the origin, then place them with `Location(...)`.
- For exported benchmark/engineering assemblies, the simulation stack reads pose from `child.location`.
- The orientation tuple inside `Location((x, y, z), (rx, ry, rz))` is in degrees, not radians.
- If a mechanism uses multiple top-level parts that must share the same slope or travel direction, encode compatible rotated locations for each of them. Do not rotate only one part and leave companion rails/walls flat.
- For chute/channel mechanisms, the floor and containment rails or walls must form one continuous corridor in the exported scene. Either place them with the same rotated frame or author them as one combined chute. A sloped floor with world-aligned rails/walls beside it is not a valid transport path.
- Verify downhill direction from the authored geometry, not from angle comments. If the object must travel from lower X to higher X, the support surface must be higher at the negative-X handoff and lower at the positive-X handoff in the final placed geometry. Reverse that relation when travel goes the other way.
- For X-aligned ramps or chutes rotated about Y, do not trust your intuition about the sign. Inspect the final placed part extents/vertices or exported scene and confirm that the spawn side is the high side before you accept the slope.
- Every child you place into `Compound(children=[...])` must be a distinct top-level object instance. Reusing one source shape is fine only if you derive separate placed copies from it with `.moved(...)`. When you need two rails, two walls, or mirrored supports from one builder result, always create each placement with `.moved(...)`; reserve `.move(...)` for a one-off final placement of a single object. Do not place the same mutable `Part` instance into the compound twice via repeated `.move(...)` calls.
- If a top-level part must be rotated in the exported scene, encode the rotation directly in the location, for example:

```python
ramp = ramp_builder.part.move(Location((0, 0, 0.06), (0, 2, 0)))
rail_upper = rail_builder.part.moved(Location((0, 0.06, 0.09), (0, 2, 0)))
rail_lower = rail_builder.part.moved(Location((0, -0.06, 0.09), (0, 2, 0)))
```

- If you first wrote `rail = rail_builder.part.move(...)` and later realize you need a mirrored or rotated companion rail, stop and rebuild that section with `.moved(...)` copies. Do not mutate the same builder result twice.

- Do not rely on `part.rotate(...)` as the final pose step for top-level exported parts. That can leave `child.location.orientation` unchanged, which makes the exported MJCF/scene appear unrotated even if the local build123d object looked rotated.

## Build-Zone Checks For Sloped Parts

- Validate sloped or rotated parts against the final rotated envelope, not just the nominal support heights or centerline endpoints.
- Include thickness, wall height, and top corners when checking whether an inclined floor or chute fits inside the allowed Z budget.
- When Z headroom is tight, prefer a thin transport floor with separate supports or rails instead of a tall wedge that rises from the ground.

## Support Continuity Contract

- For passive-transfer mechanisms, adjacent support parts must actually meet or overlap in the authored geometry where the object hands off between them.
- Do not rely on center-position arithmetic or visual near-contact. Check the real extents of the ramp/chute against the neighboring platform or support and remove any unsupported gap.

## Workspace Execution Contract

- In benchmark and engineer workspaces, shell verification commands already start in the seeded workspace root.
- Do not prepend hard-coded `cd /workspace`, `cd /home/user/workspace`, or host-specific repo paths before validation or simulation checks.

## References & Contents

### \[assembly.md\](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/.agent/skills/build123d_cad_drafting_skill/references/assembly.md) (Assembly & Physics)

- Motor Pattern (stator/rotor) (L11)
- Explicit Joint Control (L33)
- Bearings & Fasteners (L59)
- Environment Zones (Goal/Forbid) (L75)
- Collision Best Practices (L82)

### \[cheat_sheet.md\](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/.agent/skills/build123d_cad_drafting_skill/references/cheat_sheet.md) (Syntax Reference)

- Core Builders (L5)
- Sketching & Surfaces (L15)
- Modeling Operations 3D (L22)
- Feature Selection & Modification (L30)
- Direct Geometry / Algebra Mode (L40)
- IO & Export (L49)
- Common Pitfalls for LLMs (L55)

### \[handpicked.md\](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/.agent/skills/build123d_cad_drafting_skill/references/handpicked.md) (Expert Patterns)

- Simple Parts & Boolean Operations (L5)
- Sketches & Extrusion (L21)
- Advanced Features: Holes & Locations (L33)
- Selection & Modification: Fillets/Chamfers (L48)
- Loft & Sweep (L61)
- Complex Geometry: The OCC Bottle (L77)
- Mirroring & Symmetries (L99)
- **Expert Pitfalls & Patterns (L113)**: `with Locations` Plural Trap, Implicit Contexts.

### \[reference.md\](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/.agent/skills/build123d_cad_drafting_skill/references/reference.md) (Exhaustive Examples)

- 1-2: Simple Plates & Holes (L81)
- 3-5: Prismatic Solids & Locations (L127)
- 6-8: Point Lists, Polygons, Polylines (L225)
- 9-10: Selectors, Fillets, Chamfers, Holes (L300)
- 11-13: Grid/Polar Locations, Workplanes, Counterbore (L360)
- 14-17: Sweep, Mirroring, Profiles (L453)
- 18-22: Workplanes on Faces/Vertices, Rotated Planes (L561)
- 23-28: Revolve, Loft, Shelling, Slitting (L691)
- *Full Table of Contents (36 examples) available inside the file.*

> [!TIP]
> Use `list_skill_files "build123d_cad_drafting_skill"` to see all available reference guides.
