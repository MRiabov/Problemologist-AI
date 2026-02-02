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

## References & Contents

### [assembly.md](file:///references/assembly.md) (Assembly & Physics)

- Motor Pattern (stator/rotor) (L5)
- Explicit Joint Control (L24)
- Bearings & Fasteners (L40)
- Environment Zones (Goal/Forbid) (L50)
- Collision Best Practices (L60)

### [cheat_sheet.md](file:///references/cheat_sheet.md) (Syntax Reference)

- Core Builders (L5)
- Sketching & Surfaces (L15)
- Modeling Operations 3D (L22)
- Feature Selection & Modification (L30)
- Direct Geometry / Algebra Mode (L40)
- IO & Export (L49)
- Common Pitfalls for LLMs (L55)

### [handpicked.md](file:///references/handpicked.md) (Expert Patterns)

- Simple Parts & Boolean Operations (L5)
- Sketches & Extrusion (L21)
- Advanced Features: Holes & Locations (L33)
- Selection & Modification: Fillets/Chamfers (L48)
- Loft & Sweep (L61)
- Complex Geometry: The OCC Bottle (L77)
- Mirroring & Symmetries (L99)
- **Expert Pitfalls & Patterns (L113)**: `with Locations` Plural Trap, Implicit Contexts.

### [reference.md](file:///references/reference.md) (Exhaustive Examples)

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
