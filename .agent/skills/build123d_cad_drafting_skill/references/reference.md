# Introductory Examples

The examples on this page can help you learn how to build objects with
build123d, and are intended as a general overview of build123d.

They are organized from simple to complex, so working through them in
order is the best way to absorb them.

::: note
::: title
Note
:::

Some important lines are omitted below to save space, so you will most
likely need to add 1 & 2 to the provided code below for them to work:

> 1. `from build123d import *`
>
> 2. If you are using build123d *builder mode* or *algebra mode*,
>
>     > - in *ocp_vscode* simply use e.g. `show(ex15)` to the end of
>     >     your design to view parts, sketches and curves. `show_all()`
>     >     can be used to automatically show all objects with their
>     >     variable names as labels.
>     > - in *CQ-editor* add e.g. `show_object(ex15.part)`,
>     >     `show_object(ex15.sketch)` or `show_object(ex15.line)` to
>     >     the end of your design to view parts, sketches or lines.
>
> 3. If you want to save your resulting object as an STL from *builder
>     mode*, you can use e.g. `export_stl(ex15.part, "file.stl")`.
>
> 4. If you want to save your resulting object as an STL from *algebra
>     mode*, you can use e.g. `export_stl(ex15, "file.stl")`
>
> 5. build123d also supports exporting to multiple other file formats
>     including STEP, see here for further information: [Import/Export
>     Formats](https://build123d.readthedocs.io/en/latest/import_export.html)
:::

## Table of Contents

1. Simple Rectangular Plate (L81)
2. Plate with Hole (L100)
3. An extruded prismatic solid (L127)
4. Building Profiles using lines and arcs (L160)
5. Moving the current working point (L199)
6. Using Point Lists (L225)
7. Polygons (L255)
8. Polylines (L278)
9. Selectors, Fillets, and Chamfers (L300)
10. Select Last and Hole (L328)
11. Face as plane & GridLocations (L360)
12. Defining an Edge with a Spline (L406)
13. CounterBoreHoles, CounterSinkHoles, PolarLocations (L425)
14. Position on line with '@', '%' and Sweep (L453)
15. Mirroring Symmetric Geometry (L497)
16. Mirroring 3D Objects (L521)
17. Mirroring From Faces (L541)
18. Creating Workplanes on Faces (L561)
19. Locating workplane on vertex (L584)
20. Offset Sketch Workplane (L623)
21. Workplanes in center of another shape (L643)
22. Rotated Workplanes (L663)
23. Revolve (L691)
24. Loft (L718)
25. Offset Sketch (L742)
26. Offset Part (Shelling) (L768)
27. Splitting an Object (L794)
28. Locating features based on Faces (L813)
29. Non-planar workplanes (L838)
30. Creating a helix (L862)
31. Holes on a sphere (L887)
32. Slots (L912)
33. Tangent Arcs (L937)
34. SVG Import (L962)
35. Text (L987)
36. Assembly (L1012)

---

## 1. Simple Rectangular Plate {#ex 1}

Just about the simplest possible example, a rectangular
`~objects_part.Box`{.interpreted-text role="class"}.

![image](assets/general_ex1.svg){.align-center}

- **Builder mode**

    > ::: {.literalinclude language="build123d" start-after="[Ex. 1]" end-before="[Ex. 1]"}
    > general_examples.py
    > :::

- **Algebra mode**

    > ::: {.literalinclude language="build123d" start-after="[Ex. 1]" end-before="[Ex. 1]"}
    > general_examples_algebra.py
    > :::

## 2. Plate with Hole {#ex 2}

A rectangular box, but with a hole added.

![image](assets/general_ex2.svg){.align-center}

- **Builder mode**

    > In this case we are using `~build_enums.Mode`{.interpreted-text
    > role="class"} `.SUBTRACT` to cut the
    > `~objects_part.Cylinder`{.interpreted-text role="class"} from the
    > `~objects_part.Box`{.interpreted-text role="class"}.
    >
    > ::: {.literalinclude language="build123d" start-after="[Ex. 2]" end-before="[Ex. 2]"}
    > general_examples.py
    > :::

- **Algebra mode**

    > In this case we are using the subtract operator `-` to cut the
    > `~objects_part.Cylinder`{.interpreted-text role="class"} from the
    > `~objects_part.Box`{.interpreted-text role="class"}.
    >
    > ::: {.literalinclude language="build123d" start-after="[Ex. 2]" end-before="[Ex. 2]"}
    > general_examples_algebra.py
    > :::

## 3. An extruded prismatic solid {#ex 3}

Build a prismatic solid using extrusion.

![image](assets/general_ex3.svg){.align-center}

- **Builder mode**

    > This time we can first create a 2D
    > `~build_sketch.BuildSketch`{.interpreted-text role="class"} adding
    > a `~objects_sketch.Circle`{.interpreted-text role="class"} and a
    > subtracted `~objects_sketch.Rectangle`{.interpreted-text
    > role="class"} and then use
    > `~build_part.BuildPart`{.interpreted-text role="class"}\'s
    > `~operations_part.extrude`{.interpreted-text role="meth"} feature.
    >
    > ::: {.literalinclude language="build123d" start-after="[Ex. 3]" end-before="[Ex. 3]"}
    > general_examples.py
    > :::

- **Algebra mode**

    > This time we can first create a 2D
    > `~objects_sketch.Circle`{.interpreted-text role="class"} with a
    > subtracted `~objects_sketch.Rectangle`{.interpreted-text
    > role="class"}[ and then use the
    > :meth:]{.title-ref}\~operations_part.extrude\` operation for
    > parts.
    >
    > ::: {.literalinclude language="build123d" start-after="[Ex. 3]" end-before="[Ex. 3]"}
    > general_examples_algebra.py
    > :::

## 4. Building Profiles using lines and arcs {#ex 4}

Sometimes you need to build complex profiles using lines and arcs. This
example builds a prismatic solid from 2D operations. It is not necessary
to create variables for the line segments, but it will be useful in a
later example.

![image](assets/general_ex4.svg){.align-center}

- **Builder mode**

    > `~build_sketch.BuildSketch`{.interpreted-text role="class"}
    > operates on closed Faces, and the operation
    > `~operations_sketch.make_face`{.interpreted-text role="meth"} is
    > used to convert the pending line segments from
    > `~build_line.BuildLine`{.interpreted-text role="class"} into a
    > closed Face.
    >
    > ::: {.literalinclude language="build123d" start-after="[Ex. 4]" end-before="[Ex. 4]"}
    > general_examples.py
    > :::

- **Algebra mode**

    > We start with an empty `~topology.Curve`{.interpreted-text
    > role="class"} and add lines to it (note that
    > `Curve() + [line1, line2, line3]` is much more efficient than
    > `line1 + line2 + line3`, see
    > `algebra_performance`{.interpreted-text role="ref"}). The
    > operation `~operations_sketch.make_face`{.interpreted-text
    > role="meth"} is used to convert the line segments into a Face.
    >
    > ::: {.literalinclude language="build123d" start-after="[Ex. 4]" end-before="[Ex. 4]"}
    > general_examples_algebra.py
    > :::

Note that to build a closed face it requires line segments that form a
closed shape.

## 5. Moving the current working point {#ex 5}

![image](assets/general_ex5.svg){.align-center}

- **Builder mode**

    > Using `~build_common.Locations`{.interpreted-text role="class"} we
    > can place one (or multiple) objects at one (or multiple) places.
    >
    > ::: {.literalinclude language="build123d" start-after="[Ex. 5]" end-before="[Ex. 5]"}
    > general_examples.py
    > :::

- **Algebra mode**

    > Using the pattern `Pos(x, y, z=0) * obj` (with
    > `geometry.Pos`{.interpreted-text role="class"}) we can move an
    > object to the provided position. Using
    > `Rot(x_angle, y_angle, z_angle) * obj` (with
    > `geometry.Rot`{.interpreted-text role="class"}) would rotate the
    > object.
    >
    > ::: {.literalinclude language="build123d" start-after="[Ex. 5]" end-before="[Ex. 5]"}
    > general_examples_algebra.py
    > :::

## 6. Using Point Lists {#ex 6}

Sometimes you need to create a number of features at various
`~build_common.Locations`{.interpreted-text role="class"}.

![image](assets/general_ex6.svg){.align-center}

- **Builder mode**

    > You can use a list of points to construct multiple objects at
    > once.
    >
    > ::: {.literalinclude language="build123d" start-after="[Ex. 6]" end-before="[Ex. 6]"}
    > general_examples.py
    > :::

- **Algebra mode**

    > You can use loops to iterate over these Locations or list
    > comprehensions as in the example.
    >
    > The algebra operations are vectorized, which means
    > `obj - [obj1, obj2, obj3]` is short for `obj - obj1 - obj2 - ob3`
    > (and more efficient, see `algebra_performance`{.interpreted-text
    > role="ref"}).
    >
    > ::: {.literalinclude language="build123d" start-after="[Ex. 6]" end-before="[Ex. 6]"}
    > general_examples_algebra.py
    > :::

## 7. Polygons {#ex 7}

![image](assets/general_ex7.svg){.align-center}

- **Builder mode**

    > You can create `~objects_sketch.RegularPolygon`{.interpreted-text
    > role="class"} for each stack point if you would like.
    >
    > ::: {.literalinclude language="build123d" start-after="[Ex. 7]" end-before="[Ex. 7]"}
    > general_examples.py
    > :::

- **Algebra mode**

    > You can apply locations to
    > `~objects_sketch.RegularPolygon`{.interpreted-text role="class"}
    > instances for each location via loops or list comprehensions.
    >
    > ::: {.literalinclude language="build123d" start-after="[Ex. 7]" end-before="[Ex. 7]"}
    > general_examples_algebra.py
    > :::

## 8. Polylines {#ex 8}

`~objects_curve.Polyline`{.interpreted-text role="class"} allows
creating a shape from a large number of chained points connected by
lines. This example uses a polyline to create one half of an i-beam
shape, which is `~operations_generic.mirror`{.interpreted-text
role="meth"} ed to create the final profile.

![image](assets/general_ex8.svg){.align-center}

- **Builder mode**

    > ::: {.literalinclude language="build123d" start-after="[Ex. 8]" end-before="[Ex. 8]"}
    > general_examples.py
    > :::

- **Algebra mode**

    > ::: {.literalinclude language="build123d" start-after="[Ex. 8]" end-before="[Ex. 8]"}
    > general_examples_algebra.py
    > :::

## 9. Selectors, Fillets, and Chamfers {#ex 9}

This example introduces multiple useful and important concepts. Firstly
`~operations_generic.chamfer`{.interpreted-text role="meth"} and
`~operations_generic.fillet`{.interpreted-text role="meth"} can be used
to \"bevel\" and \"round\" edges respectively. Secondly, these two
methods require an edge or a list of edges to operate on. To select all
edges, you could simply pass in `ex9.edges()`.

![image](assets/general_ex9.svg){.align-center}

- **Builder mode**

    > ::: {.literalinclude language="build123d" start-after="[Ex. 9]" end-before="[Ex. 9]"}
    > general_examples.py
    > :::

- **Algebra mode**

    > ::: {.literalinclude language="build123d" start-after="[Ex. 9]" end-before="[Ex. 9]"}
    > general_examples_algebra.py
    > :::

Note that `~topology.ShapeList.group_by`{.interpreted-text role="meth"}
`(Axis.Z)` returns a list of lists of edges that is grouped by their
z-position. In this case we want to use the `[-1]` group which, by
convention, will be the highest z-dimension group.

## 10. Select Last and Hole {#ex 10}

![image](assets/general_ex10.svg){.align-center}

- **Builder mode**

    > Using `~build_enums.Select`{.interpreted-text role="class"}
    > `.LAST` you can select the most recently modified edges. It is
    > used to perform a `~operations_generic.fillet`{.interpreted-text
    > role="meth"} in this example. This example also makes use of
    > `~objects_part.Hole`{.interpreted-text role="class"} which
    > automatically cuts through the entire part.
    >
    > ::: {.literalinclude language="build123d" start-after="[Ex. 10]" end-before="[Ex. 10]"}
    > general_examples.py
    > :::

- **Algebra mode**

    > Using the pattern `snapshot = obj.edges()` before and
    > `last_edges = obj.edges() - snapshot` after an operation allows to
    > select the most recently modified edges (same for `faces`,
    > `vertices`, \...). It is used to perform a
    > `~operations_generic.fillet`{.interpreted-text role="meth"} in
    > this example. This example also makes use of
    > `~objects_part.Hole`{.interpreted-text role="class"}. Different to
    > the *context mode*, you have to add the `depth` of the whole.
    >
    > ::: {.literalinclude language="build123d" start-after="[Ex. 10]" end-before="[Ex. 10]"}
    > general_examples_algebra.py
    > :::

## 11. Use a face as a plane for BuildSketch and introduce GridLocations {#ex 11}

![image](assets/general_ex11.svg){.align-center}

- **Builder mode**

    > `~build_sketch.BuildSketch`{.interpreted-text role="class"}
    > accepts a Plane or a Face, so in this case we locate the Sketch on
    > the top of the part. Note that the face used as input to
    > BuildSketch needs to be Planar or unpredictable behavior can
    > result. Additionally
    > `~build_common.GridLocations`{.interpreted-text role="class"} can
    > be used to create a grid of points that are simultaneously used to
    > place 4 pentagons.
    >
    > Lastly, `~operations_part.extrude`{.interpreted-text role="meth"}
    > can be used with a negative amount and `Mode.SUBTRACT` to cut
    > these from the parent.
    >
    > ::: {.literalinclude language="build123d" start-after="[Ex. 11]" end-before="[Ex. 11]"}
    > general_examples.py
    > :::

- **Algebra mode**

    > The pattern `plane * obj` can be used to locate an object on a
    > plane. Furthermore, the pattern `plane * location * obj` first
    > places the object on a plane and then moves it relative to plane
    > according to `location`.
    >
    > `~build_common.GridLocations`{.interpreted-text role="class"}
    > creates a grid of points that can be used in loops or list
    > comprehensions as described earlier.
    >
    > Lastly, `~operations_part.extrude`{.interpreted-text role="meth"}
    > can be used with a negative amount and cut (`-`) from the parent.
    >
    > ::: {.literalinclude language="build123d" start-after="[Ex. 11]" end-before="[Ex. 11]"}
    > general_examples_algebra.py
    > :::

Note that the direction implied by positive or negative inputs to amount
is relative to the normal direction of the face or plane. As a result of
this, unexpected behavior can occur if the extrude direction and
mode/operation (ADD / `+` or SUBTRACT / `-`) are not correctly set.

## 12. Defining an Edge with a Spline {#ex 12}

This example defines a side using a spline curve through a collection of
points. Useful when you have an edge that needs a complex profile.

![image](assets/general_ex12.svg){.align-center}

- **Builder mode**

    > ::: {.literalinclude language="build123d" start-after="[Ex. 12]" end-before="[Ex. 12]"}
    > general_examples.py
    > :::

- **Algebra mode**

    > ::: {.literalinclude language="build123d" start-after="[Ex. 12]" end-before="[Ex. 12]"}
    > general_examples_algebra.py
    > :::

## 13. CounterBoreHoles, CounterSinkHoles, and PolarLocations {#ex 13}

Counter-sink and counter-bore holes are useful for creating recessed
areas for fasteners.

![image](assets/general_ex13.svg){.align-center}

- **Builder mode**

    > We use a face to establish a location for
    > `~build_common.Locations`{.interpreted-text role="class"}.
    >
    > ::: {.literalinclude language="build123d" start-after="[Ex. 13]" end-before="[Ex. 13]"}
    > general_examples.py
    > :::

- **Algebra mode**

    > We use a face to establish a plane that is used later in the code
    > for locating objects onto this plane.
    >
    > ::: {.literalinclude language="build123d" start-after="[Ex. 13]" end-before="[Ex. 13]"}
    > general_examples_algebra.py
    > :::

`~build_common.PolarLocations`{.interpreted-text role="class"} creates a
list of points that are radially distributed.

## 14. Position on a line with \'@\', \'%\' and introduce Sweep {#ex 14}

build123d includes a feature for finding the position along a line
segment. This is normalized between 0 and 1 and can be accessed using
the `~topology.Mixin1D.position_at`{.interpreted-text role="meth"}
([@]{.title-ref}) operator. Similarly the
`~topology.Mixin1D.tangent_at`{.interpreted-text role="meth"}
([%]{.title-ref}) operator returns the line direction at a given point.

These two features are very powerful for chaining line segments together
without having to repeat dimensions again and again, which is error
prone, time consuming, and more difficult to maintain. The pending faces
must lie on the path, please see example 37 for a way to make this
placement easier.

![image](assets/general_ex14.svg){.align-center}

- **Builder mode**

    > The `~operations_generic.sweep`{.interpreted-text role="meth"}
    > method takes any pending faces and sweeps them through the
    > provided path (in this case the path is taken from the pending
    > edges from `ex14_ln`).
    > `~operations_part.revolve`{.interpreted-text role="meth"} requires
    > a single connected wire.
    >
    > ::: {.literalinclude language="build123d" start-after="[Ex. 14]" end-before="[Ex. 14]"}
    > general_examples.py
    > :::

- **Algebra mode**

    > The `~operations_generic.sweep`{.interpreted-text role="meth"}
    > method takes any faces and sweeps them through the provided path
    > (in this case the path is taken from `ex14_ln`).
    >
    > ::: {.literalinclude language="build123d" start-after="[Ex. 14]" end-before="[Ex. 14]"}
    > general_examples_algebra.py
    > :::

It is also possible to use tuple or `~geometry.Vector`{.interpreted-text
role="class"} addition (and other vector math operations) as seen in the
`l3` variable.

## 15. Mirroring Symmetric Geometry {#ex 15}

Here mirror is used on the BuildLine to create a symmetric shape with
fewer line segment commands. Additionally the \'@\' operator is used to
simplify the line segment commands.

`(l4 @ 1).Y` is used to extract the y-component of the `l4 @ 1` vector.

![image](assets/general_ex15.svg){.align-center}

- **Builder mode**

    > ::: {.literalinclude language="build123d" start-after="[Ex. 15]" end-before="[Ex. 15]"}
    > general_examples.py
    > :::

- **Algebra mode**

    > Combine lines via the pattern `Curve() + [l1, l2, l3, l4, l5]`
    >
    > ::: {.literalinclude language="build123d" start-after="[Ex. 15]" end-before="[Ex. 15]"}
    > general_examples_algebra.py
    > :::

## 16. Mirroring 3D Objects {#ex 16}

Mirror can also be used with BuildPart (and BuildSketch) to mirror 3D
objects. The `Plane.offset()` method shifts the plane in the normal
direction (positive or negative).

![image](assets/general_ex16.svg){.align-center}

- **Builder mode**

    > ::: {.literalinclude language="build123d" start-after="[Ex. 16]" end-before="[Ex. 16]"}
    > general_examples.py
    > :::

- **Algebra mode**

    > ::: {.literalinclude language="build123d" start-after="[Ex. 16]" end-before="[Ex. 16]"}
    > general_examples_algebra.py
    > :::

## 17. Mirroring From Faces {#ex 17}

Here we select the farthest face in the Y-direction and turn it into a
`~geometry.Plane`{.interpreted-text role="class"} using the `Plane()`
class.

![image](assets/general_ex17.svg){.align-center}

- **Builder mode**

    > ::: {.literalinclude language="build123d" start-after="[Ex. 17]" end-before="[Ex. 17]"}
    > general_examples.py
    > :::

- **Algebra mode**

    > ::: {.literalinclude language="build123d" start-after="[Ex. 17]" end-before="[Ex. 17]"}
    > general_examples_algebra.py
    > :::

## 18. Creating Workplanes on Faces {#ex 18}

Here we start with an earlier example, select the top face, draw a
rectangle and then use Extrude with a negative distance.

![image](assets/general_ex18.svg){.align-center}

- **Builder mode**

    > We then use `Mode.SUBTRACT` to cut it out from the main body.
    >
    > ::: {.literalinclude language="build123d" start-after="[Ex. 18]" end-before="[Ex. 18]"}
    > general_examples.py
    > :::

- **Algebra mode**

    > We then use `-=` to cut it out from the main body.
    >
    > ::: {.literalinclude language="build123d" start-after="[Ex. 18]" end-before="[Ex. 18]"}
    > general_examples_algebra.py
    > :::

## 19. Locating a workplane on a vertex {#ex 19}

Here a face is selected and two different strategies are used to select
vertices. Firstly `vtx` uses
`~topology.ShapeList.group_by`{.interpreted-text role="meth"} and
`Axis.X` to select a particular vertex. The second strategy uses a
custom defined Axis `vtx2Axis` that is pointing roughly in the direction
of a vertex to select, and then
`~topology.ShapeList.sort_by`{.interpreted-text role="meth"} this custom
Axis.

![image](assets/general_ex19.svg){.align-center}

- **Builder mode**

    > Then the X and Y positions of these vertices are selected and
    > passed to `~build_common.Locations`{.interpreted-text
    > role="class"} as center points for two circles that cut through
    > the main part. Note that if you passed the variable `vtx` directly
    > to `~build_common.Locations`{.interpreted-text role="class"} then
    > the part would be offset from the workplane by the vertex
    > z-position.
    >
    > ::: {.literalinclude language="build123d" start-after="[Ex. 19]" end-before="[Ex. 19]"}
    > general_examples.py
    > :::

- **Algebra mode**

    > Then the X and Y positions of these vertices are selected and used
    > to move two circles that cut through the main part. Note that if
    > you passed the variable `vtx` directly to
    > `~geometry.Pos`{.interpreted-text role="class"} then the part
    > would be offset from the workplane by the vertex z-position.
    >
    > ::: {.literalinclude language="build123d" start-after="[Ex. 19]" end-before="[Ex. 19]"}
    > general_examples_algebra.py
    > :::

## 20. Offset Sketch Workplane {#ex 20}

The `plane` variable is set to be coincident with the farthest face in
the negative x-direction. The resulting Plane is offset from the
original position.

![image](assets/general_ex20.svg){.align-center}

- **Builder mode**

    > ::: {.literalinclude language="build123d" start-after="[Ex. 20]" end-before="[Ex. 20]"}
    > general_examples.py
    > :::

- **Algebra mode**

    > ::: {.literalinclude language="build123d" start-after="[Ex. 20]" end-before="[Ex. 20]"}
    > general_examples_algebra.py
    > :::

## 21. Create a Workplanes in the center of another shape {#ex 21}

One cylinder is created, and then the origin and z_dir of that part are
used to create a new Plane for positioning another cylinder
perpendicular and halfway along the first.

![image](assets/general_ex21.svg){.align-center}

- **Builder mode**

    > ::: {.literalinclude language="build123d" start-after="[Ex. 21]" end-before="[Ex. 21]"}
    > general_examples.py
    > :::

- **Algebra mode**

    > ::: {.literalinclude language="build123d" start-after="[Ex. 21]" end-before="[Ex. 21]"}
    > general_examples_algebra.py
    > :::

## 22. Rotated Workplanes {#ex 22}

It is also possible to create a rotated workplane, building upon some of
the concepts in an earlier example.

![image](assets/general_ex22.svg){.align-center}

- **Builder mode**

    > Use the `~geometry.Plane.rotated`{.interpreted-text role="meth"}
    > method to rotate the workplane.
    >
    > ::: {.literalinclude language="build123d" start-after="[Ex. 22]" end-before="[Ex. 22]"}
    > general_examples.py
    > :::

- **Algebra mode**

    > Use the operator `*` to relocate the plane (post-multiplication!).
    >
    > ::: {.literalinclude language="build123d" start-after="[Ex. 22]" end-before="[Ex. 22]"}
    > general_examples_algebra.py
    > :::

`~build_common.GridLocations`{.interpreted-text role="class"} places 4
Circles on 4 points on this rotated workplane, and then the Circles are
extruded in the \"both\" (positive and negative) normal direction.

## 23. Revolve {#ex 23}

Here we build a sketch with a
`~objects_curve.Polyline`{.interpreted-text role="class"},
`~objects_curve.Line`{.interpreted-text role="class"}, and a
`~objects_sketch.Circle`{.interpreted-text role="class"}. It is
absolutely critical that the sketch is only on one side of the axis of
rotation before Revolve is called. To that end, `split` is used with
`Plane.ZY` to keep only one side of the Sketch.

It is highly recommended to view your sketch before you attempt to call
revolve.

![image](assets/general_ex23.svg){.align-center}

- **Builder mode**

    > ::: {.literalinclude language="build123d" start-after="[Ex. 23]" end-before="[Ex. 23]"}
    > general_examples.py
    > :::

- **Algebra mode**

    > ::: {.literalinclude language="build123d" start-after="[Ex. 23]" end-before="[Ex. 23]"}
    > general_examples_algebra.py
    > :::

## 24. Loft {#ex 24}

Loft is a very powerful tool that can be used to join dissimilar shapes.
In this case we make a conical-like shape from a circle and a rectangle
that is offset vertically. In this case
`~operations_part.loft`{.interpreted-text role="meth"} automatically
takes the pending faces that were added by the two BuildSketches. Loft
can behave unexpectedly when the input faces are not parallel to each
other.

![image](assets/general_ex24.svg){.align-center}

- **Builder mode**

    > ::: {.literalinclude language="build123d" start-after="[Ex. 24]" end-before="[Ex. 24]"}
    > general_examples.py
    > :::

- **Algebra mode**

    > ::: {.literalinclude language="build123d" start-after="[Ex. 24]" end-before="[Ex. 24]"}
    > general_examples_algebra.py
    > :::

## 25. Offset Sketch {#ex 25}

![image](assets/general_ex25.svg){.align-center}

- **Builder mode**

    > BuildSketch faces can be transformed with a 2D
    > `~operations_generic.offset`{.interpreted-text role="meth"}.
    >
    > ::: {.literalinclude language="build123d" start-after="[Ex. 25]" end-before="[Ex. 25]"}
    > general_examples.py
    > :::

- **Algebra mode**

    > Sketch faces can be transformed with a 2D
    > `~operations_generic.offset`{.interpreted-text role="meth"}.
    >
    > ::: {.literalinclude language="build123d" start-after="[Ex. 25]" end-before="[Ex. 25]"}
    > general_examples_algebra.py
    > :::

They can be offset inwards or outwards, and with different techniques
for extending the corners (see `~build_enums.Kind`{.interpreted-text
role="class"} in the Offset docs).

## 26. Offset Part To Create Thin features {#ex 26}

Parts can also be transformed using an offset, but in this case with a
3D `~operations_generic.offset`{.interpreted-text role="meth"}. Also
commonly known as a shell, this allows creating thin walls using very
few operations. This can also be offset inwards or outwards. Faces can
be selected to be \"deleted\" using the `openings` parameter of
`~operations_generic.offset`{.interpreted-text role="meth"}.

Note that self intersecting edges and/or faces can break both 2D and 3D
offsets.

![image](assets/general_ex26.svg){.align-center}

- **Builder mode**

    > ::: {.literalinclude language="build123d" start-after="[Ex. 26]" end-before="[Ex. 26]"}
    > general_examples.py
    > :::

- **Algebra mode**

    > ::: {.literalinclude language="build123d" start-after="[Ex. 26]" end-before="[Ex. 26]"}
    > general_examples_algebra.py
    > :::

## 27. Splitting an Object {#ex 27}

You can split an object using a plane, and retain either or both halves.
In this case we select a face and offset half the width of the box.

![image](assets/general_ex27.svg){.align-center}

- **Builder mode**

    > ::: {.literalinclude language="build123d" start-after="[Ex. 27]" end-before="[Ex. 27]"}
    > general_examples.py
    > :::

- **Algebra mode**

    > ::: {.literalinclude language="build123d" start-after="[Ex. 27]" end-before="[Ex. 27]"}
    > general_examples_algebra.py
    > :::

## 28. Locating features based on Faces {#ex 28}

![image](assets/general_ex28.svg){.align-center}

- **Builder mode**

    > We create a triangular prism with
    > `~build_enums.Mode`{.interpreted-text role="class"} `.PRIVATE` and
    > then later use the faces of this object to cut holes in a sphere.
    >
    > ::: {.literalinclude language="build123d" start-after="[Ex. 28]" end-before="[Ex. 28]"}
    > general_examples.py
    > :::

- **Algebra mode**

    > We create a triangular prism and then later use the faces of this
    > object to cut holes in a sphere.
    >
    > ::: {.literalinclude language="build123d" start-after="[Ex. 28]" end-before="[Ex. 28]"}
    > general_examples_algebra.py
    > :::

We are able to create multiple workplanes by looping over the list of
faces.

## 29. The Classic OCC Bottle {#ex 29}

build123d is based on the OpenCascade.org (OCC) modeling Kernel. Those
who are familiar with OCC know about the famous 'bottle' example. We use
a 3D Offset and the openings parameter to create the bottle opening.

![image](assets/general_ex29.svg){.align-center}

- **Builder mode**

    > ::: {.literalinclude language="build123d" start-after="[Ex. 29]" end-before="[Ex. 29]"}
    > general_examples.py
    > :::

- **Algebra mode**

    > ::: {.literalinclude language="build123d" start-after="[Ex. 29]" end-before="[Ex. 29]"}
    > general_examples_algebra.py
    > :::

## 30. Bezier Curve {#ex 30}

Here `pts` is used as an input to both
`~objects_curve.Polyline`{.interpreted-text role="class"} and
`~objects_curve.Bezier`{.interpreted-text role="class"} and `wts` to
Bezier alone. These two together create a closed line that is made into
a face and extruded.

![image](assets/general_ex30.svg){.align-center}

- **Builder mode**

    > ::: {.literalinclude language="build123d" start-after="[Ex. 30]" end-before="[Ex. 30]"}
    > general_examples.py
    > :::

- **Algebra mode**

    > ::: {.literalinclude language="build123d" start-after="[Ex. 30]" end-before="[Ex. 30]"}
    > general_examples_algebra.py
    > :::

## 31. Nesting Locations {#ex 31}

Locations contexts can be nested to create groups of shapes. Here 24
triangles, 6 squares, and 1 hexagon are created and then extruded.
Notably `~build_common.PolarLocations`{.interpreted-text role="class"}
rotates any \"children\" groups by default.

![image](assets/general_ex31.svg){.align-center}

- **Builder mode**

    > ::: {.literalinclude language="build123d" start-after="[Ex. 31]" end-before="[Ex. 31]"}
    > general_examples.py
    > :::

- **Algebra mode**

    > ::: {.literalinclude language="build123d" start-after="[Ex. 31]" end-before="[Ex. 31]"}
    > general_examples_algebra.py
    > :::

## 32. Python For-Loop {#ex 32}

In this example, a standard python for-loop is used along with a list of
faces extracted from a sketch to progressively modify the extrusion
amount. There are 7 faces in the sketch, so this results in 7 separate
calls to `~operations_part.extrude`{.interpreted-text role="meth"}.

![image](assets/general_ex32.svg){.align-center}

- **Builder mode**

    > `~build_enums.Mode`{.interpreted-text role="class"} `.PRIVATE` is
    > used in `~build_sketch.BuildSketch`{.interpreted-text
    > role="class"} to avoid adding these faces until the for-loop.
    >
    > ::: {.literalinclude language="build123d" start-after="[Ex. 32]" end-before="[Ex. 32]"}
    > general_examples.py
    > :::

- **Algebra mode**

    > ::: {.literalinclude language="build123d" start-after="[Ex. 32]" end-before="[Ex. 32]"}
    > general_examples_algebra.py
    > :::

## 33. Python Function and For-Loop {#ex 33}

Building on the previous example, a standard python function is used to
return a sketch as a function of several inputs to progressively modify
the size of each square.

![image](assets/general_ex33.svg){.align-center}

- **Builder mode**

    > The function returns a
    > `~build_sketch.BuildSketch`{.interpreted-text role="class"}.
    >
    > ::: {.literalinclude language="build123d" start-after="[Ex. 33]" end-before="[Ex. 33]"}
    > general_examples.py
    > :::

- **Algebra mode**

    > The function returns a `Sketch` object.
    >
    > ::: {.literalinclude language="build123d" start-after="[Ex. 33]" end-before="[Ex. 33]"}
    > general_examples_algebra.py
    > :::

## 34. Embossed and Debossed Text {#ex 34}

![image](assets/general_ex34.svg){.align-center}

- **Builder mode**

    > The text \"Hello\" is placed on top of a rectangle and embossed
    > (raised) by placing a BuildSketch on the top face (`topf`). Note
    > that `~build_enums.Align`{.interpreted-text role="class"} is used
    > to control the text placement. We re-use the `topf` variable to
    > select the same face and deboss (indented) the text \"World\".
    > Note that if we simply ran
    > `BuildSketch(ex34.faces().sort_by(Axis.Z)[-1])` for both
    > `ex34_sk1 & 2` it would incorrectly locate the 2nd \"World\" text
    > on the top of the \"Hello\" text.
    >
    > ::: {.literalinclude language="build123d" start-after="[Ex. 34]" end-before="[Ex. 34]"}
    > general_examples.py
    > :::

- **Algebra mode**

    > The text \"Hello\" is placed on top of a rectangle and embossed
    > (raised) by placing a sketch on the top face (`topf`). Note that
    > `~build_enums.Align`{.interpreted-text role="class"} is used to
    > control the text placement. We re-use the `topf` variable to
    > select the same face and deboss (indented) the text \"World\".
    >
    > ::: {.literalinclude language="build123d" start-after="[Ex. 34]" end-before="[Ex. 34]"}
    > general_examples_algebra.py
    > :::

## 35. Slots {#ex 35}

![image](assets/general_ex35.svg){.align-center}

- **Builder mode**

    > Here we create a
    > `~objects_sketch.SlotCenterToCenter`{.interpreted-text
    > role="class"} and then use a
    > `~build_line.BuildLine`{.interpreted-text role="class"} and
    > `~objects_curve.RadiusArc`{.interpreted-text role="class"} to
    > create an arc for two instances of
    > `~objects_sketch.SlotArc`{.interpreted-text role="class"}.
    >
    > ::: {.literalinclude language="build123d" start-after="[Ex. 35]" end-before="[Ex. 35]"}
    > general_examples.py
    > :::

- **Algebra mode**

    > Here we create a
    > `~objects_sketch.SlotCenterToCenter`{.interpreted-text
    > role="class"} and then use a
    > `~objects_curve.RadiusArc`{.interpreted-text role="class"} to
    > create an arc for two instances of
    > `~operations_sketch.SlotArc`{.interpreted-text role="class"}.
    >
    > ::: {.literalinclude language="build123d" start-after="[Ex. 35]" end-before="[Ex. 35]"}
    > general_examples_algebra.py
    > :::

## 36. Extrude Until {#ex 36}

Sometimes you will want to extrude until a given face that could be non
planar or where you might not know easily the distance you have to
extrude to. In such cases you can use
`~operations_part.extrude`{.interpreted-text role="meth"}
`~build_enums.Until`{.interpreted-text role="class"} with `Until.NEXT`
or `Until.LAST`.

![image](assets/general_ex36.svg){.align-center}

- **Builder mode**

    > ::: {.literalinclude language="build123d" start-after="[Ex. 36]" end-before="[Ex. 36]"}
    > general_examples.py
    > :::

- **Algebra mode**

    > ::: {.literalinclude language="build123d" start-after="[Ex. 36]" end-before="[Ex. 36]"}
    > general_examples_algebra.py
    > :::

## 37. Scaling the models

The `scale` *function* has a signature as follows:

```
scale(objects: build123d.topology.shape_core.Shape | collections.abc.Iterable[build123d.topology.shape_core.Shape] | None = None, by: float | tuple[float, float, float] = 1, mode: ~build123d.build_enums.Mode = <Mode.REPLACE>)→ Curve | Sketch | Part | Compound[source]
```
