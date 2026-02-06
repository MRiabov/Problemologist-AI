# build123d Detailed Examples

This document provides templates and examples for common CAD operations using `build123d`.

## 1. Simple Parts & Boolean Operations

### Plate with Hole (Builder Mode)

```python
with BuildPart() as p:
    Box(80, 60, 10)
    Cylinder(radius=11, height=10, mode=Mode.SUBTRACT)
```

### Plate with Hole (Algebra Mode)

```python
p = Box(80, 60, 10) - Cylinder(radius=11, height=10)
```

## 2. Sketches & Extrusion

### Prismatic Solid (Circle with Square Hole)

```python
with BuildPart() as p:
    with BuildSketch():
        Circle(60)
        Rectangle(40, 40, mode=Mode.SUBTRACT)
    extrude(amount=20)
```

## 3. Advanced Features (Holes & Locations)

### Counter-sink & Counter-bore with Polar Array

```python
with BuildPart() as p:
    Cylinder(radius=50, height=10)
    # Target the top face for holes
    with Locations(p.faces().sort_by(Axis.Z)[-1]):
        with PolarLocations(radius=40, count=4):
            CounterSinkHole(radius=4, counter_sink_radius=8)
        with PolarLocations(radius=40, count=4, start_angle=45):
            CounterBoreHole(radius=4, counter_bore_radius=8, counter_bore_depth=4)
```

## 4. Selection & Modification (Fillets/Chamfers)

### Grouped Edge Filleting

```python
with BuildPart() as p:
    Box(80, 60, 10)
    # Chamfer top edges (Z group -1)
    chamfer(p.edges().group_by(Axis.Z)[-1], length=4)
    # Fillet all vertical edges (parallel to Z)
    fillet(p.edges().filter_by(Axis.Z), radius=5)
```

## 5. Loft & Sweep

### Conical Loft (Builder Mode)

```python
with BuildPart() as p:
    Box(80, 80, 10)
    # Sketch on top face
    with BuildSketch(p.faces().sort_by(Axis.Z)[-1]) as s1:
        Circle(25)
    # Offset sketch from previous sketch face
    with BuildSketch(s1.faces()[0].offset(40)) as s2:
        Rectangle(15, 15)
    loft()
```

## 6. Complex Geometry (The OCC Bottle)

```python
L, w, t, b, h, n = 60.0, 18.0, 9.0, 0.9, 90.0, 6.0
with BuildPart() as bottle:
    with BuildSketch(Plane.XY.offset(-b)) as sk:
        with BuildLine() as ln:
            l1 = Line((0, 0), (0, w / 2))
            l2 = ThreePointArc(l1 @ 1, (L / 2.0, w / 2.0 + t), (L, w / 2.0))
            l3 = Line(l2 @ 1, ((l2 @ 1).X, 0, 0))
            mirror(ln.line)
        make_face()
    extrude(amount=h + b)
    fillet(bottle.edges(), radius=w / 6)
    with BuildSketch(bottle.faces().sort_by(Axis.Z)[-1]):
        Circle(t)
    extrude(amount=n)
    # Shelling operation
    necktopf = bottle.faces().sort_by(Axis.Z)[-1]
    offset(bottle.solids()[0], amount=-b, openings=necktopf)
```

## 7. Mirroring & Symmetries

### Symmetric Profile (I-Beam)

```python
with BuildPart() as ibeam:
    with BuildSketch(Plane.YZ) as sk:
        with BuildLine() as ln:
            Polyline([(0, 10), (10, 10), (10, 9), (0.5, 9), (0.5, -9), (10, -9), (10, -10), (0, -10)])
            mirror(ln.line, about=Plane.YZ)
        make_face()
    extrude(amount=100)
```

## 8. Expert Pitfalls & Patterns

### Plural `with Locations` vs Singular `Location`

* **The Trap**: `with Location((0,0,1)):` results in `AttributeError: __enter__`.
* **The Fix**: Use `with Locations(Location((0,0,1))):` even for a single position. `Locations` is the context manager; `Location` is just the data.

### Implicit Contexts

Many functions implicitly use the "active object" of the current builder:

```python
with BuildPart() as bp:
    Box(10, 10, 10)
    fillet(bp.edges(), radius=1) # bp.edges() contains edges of the Box
```

### Selector Chaining

```python
# Select the top-most face
top_face = bp.faces().sort_by(Axis.Z)[-1]
# Select all edges except the ones on the bottom plane
side_edges = bp.edges().filter_by(Axis.Z)
```
