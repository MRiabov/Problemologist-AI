# 2D Joints

This document explains how the 2D sketch joints work for layout, tangency, and annotation workflows.

These helpers are for sketch layout. They are not MJCF joints or simulation joints.

## 1. Tangent bridge between repeated features

`Edge.make_constrained_lines(...)` can return multiple valid tangent solutions. Pick the one that matches the intended layout.

```python
from build123d import Edge, Location, Wire

left = Edge.make_circle(10)
right = left.moved(Location((30, 0, 0)))

tangent_choices = Edge.make_constrained_lines(left, right)
bridge = min(tangent_choices, key=lambda e: e.length)
path = Wire([bridge])
```

## 2. Fixed-radius tangent arc

`Edge.make_constrained_arcs(...)` is useful when the turn radius is known but the final endpoint should emerge from the joint relationship.

```python
from build123d import Axis, Edge, Vector, Wire

arc_choices = Edge.make_constrained_arcs(Axis.X, Vector(20, 12, 0), radius=8)
arc = arc_choices[0]
path = Wire([arc])
```

## 3. Dimensioning and extension lines

The drafting helpers can annotate a sketch when the geometry is already defined.

```python
from build123d import BuildSketch, Circle, Draft, DimensionLine, Edge, ExtensionLine, Unit

draft = Draft(unit=Unit.MM, decimal_precision=1)
span = Edge.make_line((0, 0, 0), (32, 0, 0))

with BuildSketch() as sk:
    Circle(1)
    DimensionLine(span, draft, sketch=sk.sketch, label="32 mm")
    ExtensionLine(span, offset=5, draft=draft, sketch=sk.sketch, label="32 mm")
```

## 4. Rule of thumb

- Use joint helpers when the sketch is relationship-driven.
- Use `BuildSketch`, `BuildLine`, and `BuildPart` for the actual CAD shape.
- Multiple solutions are normal; choose by length, orientation, or datum fit.
