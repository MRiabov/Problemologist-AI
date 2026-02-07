# build123d Concise LLM Reference

`build123d` is a Python CAD library using OpenCASCADE. It focuses on "CAD as code" with two main modes: Builder and Algebra.

## 1. Core Builders (Context Managers)

Builders maintain a current "working" object.

- `with BuildPart() as bp:` -> `bp.part` (3D Solid)
- `with BuildSketch() as bs:` -> `bs.sketch` (2D Face/Wire)
- `with BuildLine() as bl:` -> `bl.line` (1D Wire/Edge)

Commonly used with `Mode`: `ADD`, `SUBTRACT`, `INTERSECT`, `REPLACE`, `PRIVATE`.

## 2. Sketching & Surfaces

- **Primitives**: `Rectangle(w, h)`, `Circle(r)`, `RegularPolygon(r, n)`, `SlotCenterToCenter(c2c, d)`.
- **Placement**: `with BuildSketch(Plane.XY.offset(z)):`
- **Locations**: `with Locations(Location((x,y))): Circle(5)` places circle at (x,y). (Note: always use plural `Locations` as a context manager).
- **Arrays**: `GridLocations(dx, dy, nx, ny)`, `PolarLocations(radius, count)`.

## 3. Modeling Operations (3D)

- **Extrude**: `extrude(obj, amount)` or `extrude(amount)` (uses current sketch).
- **Revolve**: `revolve(obj, axis, angle)`.
- **Sweep**: `sweep(path)`.
- **Loft**: `loft()`.
- **Boolean**: `bp.part -= Box(10, 10, 10)` or `Box(10, 10, 10, mode=Mode.SUBTRACT)`.

## 4. Feature Selection & Modification

Selectors filter `Edges`, `Faces`, `Vertices`.

- `obj.faces().sort_by(Axis.Z)[-1]` -> Top face.
- `obj.edges().filter_by(Axis.X)` -> Edges parallel to X.
- `obj.edges().sort_by(SortBy.LENGTH)[-1]` -> Longest edge.
- **Modification**: `fillet(edges, radius)`, `chamfer(edges, radius)`.
- **Workplanes**: `with BuildSketch(bp.faces().sort_by(Axis.Z)[-1]):` -> Sketch on top face.

## 5. Direct Geometry (Algebra Mode)

Stateless creation:

- `box = Box(length, width, height)`
- `sphere = Sphere(radius)`
- `cylinder = Cylinder(radius, height)`
- `result = box - sphere + cylinder`

## 6. IO & Export

- `export_stl(obj, "file.stl")`
- `export_step(obj, "file.step")`
- `import_step("file.step")`

## 7. Common Pitfalls for LLMs

- **Builder Shadowing**: Don't define a variable with the same name as a builder (e.g., `BuildPart = ...`).
- **Context Awareness**: Operations like `extrude()` without an object argument *require* an active `BuildSketch` context.
- **Selectors**: Always ensure the object exists before selecting (e.g., `bp.part` must have faces).
- **Locations vs Location**: **CRITICAL**: Use `with Locations(...):` (plural) as a context manager. `with Location(...):` (singular) will fail because it does not support the context manager protocol.
- **Context Scope**: `with Locations(...)` only affects objects created *inside* the block.

---
*Reference version: 1.1 (Condensed with Examples)*

### Extended Resources

- **Examples**: See `docs/examples.md` for detailed code snippets and templates.
