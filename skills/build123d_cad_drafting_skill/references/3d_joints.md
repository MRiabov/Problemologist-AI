# 3D Joints

This document explains how the 3D placement joints work for part placement, repetition, and assembly pose.

These are placement joints, not physics joints.

Treat the joint frame or mating datum as the source of truth for pose. Derive every repeated or rotated placement from that relationship instead of placing parts by arbitrary world coordinates.

## 1. Repeated placements from one builder result

`Locations(...)` is the context manager for repeated placement. Use it to keep multiple solids aligned without manual coordinate bookkeeping.

```python
from build123d import Align, Box, BuildPart, Location, Locations

with BuildPart() as posts:
    with Locations(Location((0, 0, 0)), Location((30, 0, 0))):
        Box(4, 4, 20, align=(Align.CENTER, Align.CENTER, Align.MIN))
```

## 2. Rotated top-level placement in an assembly

Derive separate placed copies from the same builder result, then assemble them as distinct children.

```python
from build123d import Align, Box, BuildPart, Compound, Location

with BuildPart() as base:
    Box(20, 10, 4, align=(Align.CENTER, Align.CENTER, Align.MIN))
base_part = base.part
base_part.label = "base"

with BuildPart() as bracket:
    Box(4, 8, 12, align=(Align.CENTER, Align.CENTER, Align.MIN))
bracket_part = bracket.part.moved(Location((20, 0, 4), (0, 90, 0)))
bracket_part.label = "bracket"

assembly = Compound(children=[base_part, bracket_part])
```

## 3. Rule of thumb

- Use `Location(...)` as pose data and `Locations(...)` as the context manager.
- Use the joint or datum chain to calculate pose before calling `Location(...)`.
- Derive separate `.moved(...)` copies when you need multiple instances.
- Run `do_children_intersect()` on the final compound if placement risk matters.
