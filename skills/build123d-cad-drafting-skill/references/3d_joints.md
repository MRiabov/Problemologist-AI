# 3D Joints

This document explains how the 3D placement joints work for part placement, repetition, and assembly pose.

These are placement joints, not physics joints.

Treat the joint frame or mating datum as the source of truth for pose. Derive every repeated or rotated placement from that relationship instead of placing parts by arbitrary world coordinates. Prefer constrained placement against an existing part whenever one exists; use standalone `Location(...)` only when the assembly relation cannot be expressed more directly from a mate, joint, or repeated datum chain.

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

## 4. Face-constrained placement example

This pattern places a second part from the first part's face instead of hand-placing the second part with raw XYZ coordinates.

```python
from build123d import Align, Axis, Box, BuildPart, BuildSketch, Rectangle, extrude

with BuildPart() as base:
    Box(40, 30, 8, align=(Align.CENTER, Align.CENTER, Align.MIN))

base_top = base.part.faces().sort_by(Axis.Z)[-1]

with BuildPart() as boss:
    with BuildSketch(base_top):
        Rectangle(12, 10)
    extrude(6)
```

Here the `BuildSketch(...)` plane comes from `base.part.faces().sort_by(Axis.Z)[-1]`, so the second part is constrained to the first part's top face rather than anchored by a free-form coordinate triple.

## 5. Visual face selection tip

When you are looking for a face to mate against, inspect the model visually first and then use an axis sort to confirm the likely candidate.
The obvious mating face is often the first or last face along a principal axis, so `faces().sort_by(Axis.Z)[0]` or `faces().sort_by(Axis.Z)[-1]` is usually faster and less error-prone than trying to infer the face from coordinates alone.
