---
name: build123d
description: Expert knowledge for CAD modeling using the build123d library.
---

# build123d CAD Expert

Use this skill when you need to design 3D parts, 2D sketches, or 1D lines using the `build123d` library.

## Core Instructions

1. **Prefer Builder Mode**: Use `with BuildPart()`, `with BuildSketch()`, and `with BuildLine()` contexts. This is more robust for complex geometry.
2. **Selector Mastery**: Use semantic selectors (`faces()`, `edges()`, `vertices()`) combined with sorting (`sort_by(Axis.Z)`) rather than index-based selection when possible.
3. **Context Sensitivity**: Be aware that many functions (like `extrude()`, `fillet()`) implicitly use the current builder's active object or selected features if not passed explicitly.
4. **Coordinate Systems**: Use `Location` and `Rotation` for precise placement. Use `Plane` for defining workplanes (e.g., `Plane.XY.offset(10)`).
5. **Booleans**: Leverage the `mode` parameter (`Mode.ADD`, `Mode.SUBTRACT`) within builders to perform operations incrementally.

## Common Code Patterns

### 3D Part with Sketch

```python
with BuildPart() as bp:
    with BuildSketch() as bs:
        Rectangle(10, 20)
    extrude(amount=5)
    # Fillet the top edges
    fillet(bp.edges().sort_by(Axis.Z)[-1], radius=1)
```

### Algebra Mode (Stateless)

```python
box = Box(10, 10, 10)
hole = Cylinder(2, 10)
result = box - hole
```

## References

- See `docs/llms.txt` for a condensed syntax reference.
- See `docs/examples.md` for detailed templates and common patterns.
- Use the `build123d_docs` skill to find more specific documentation if needed.
