# build123d Technical Drawing Reference

Use this reference when turning an existing 3D build123d model into a sheet that people can read, inspect, or export to other CAD tools.

## Core API

- `TechnicalDrawing(...)` creates the border and title block. Set page size, title, sheet number, scale, and text sizing there.
- `project_to_viewport(...)` projects the 3D source into 2D visible and hidden edge sets from a chosen camera origin and up vector.
- `Draft(...)` controls dimension formatting and text presentation.
- `ExtensionLine(...)` turns model geometry into dimension markup.
- `Text(...)` places view labels, notes, and title-block text.
- `ExportSVG(...)` writes the authoritative vector sheet.
- `ExportDXF(...)` is useful when the downstream workflow needs a CAD interchange file.

## Layout Rules

- Start with the standard orthographic trio: front, top/plan, and right-side.
- Add an isometric only as a small orientation aid.
- Keep related views aligned so the sheet reads as one system.
- Leave margin around each view; do not pack the page edge-to-edge.
- Keep hidden edges visually quieter than visible edges.
- Use a single scale when possible. If you need a detail scale, make that choice explicit.

## Annotation Rules

- Dimension only what matters for fit, interface, or inspection.
- Prefer stable geometry targets: face-to-face distance, hole center spacing, diameters, and datum offsets.
- Do not invent sections, datums, or callouts that are not supported by the 3D model.
- Remove clutter before adding another label.

## Problemologist Workflow

- The drawing is derived from the authored 3D model, not the other way around.
- In planner/reviewer workspaces, call `render_technical_drawing()` before submit or approval.
- Treat the vector export as the authoritative sheet and the preview image as the inspection surface.

## Official Tutorial

- https://build123d.readthedocs.io/en/latest/tech_drawing_tutorial.html
