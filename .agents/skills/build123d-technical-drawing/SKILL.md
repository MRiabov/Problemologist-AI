---
name: build123d-technical-drawing
description: Author and review build123d technical drawings with `TechnicalDrawing`, projected orthographic views, dimensioning, line layering, and SVG/DXF export. Use when creating or editing drawing scripts, orthographic sheets, annotations, or `preview_drawing()` artifacts grounded in an existing 3D build123d model.
---

# Build123d Technical Drawing

## Overview

Turn an existing 3D build123d model into a clean technical drawing package. Keep the 3D model authoritative; the sheet is a projection, not a second source of geometry.

## Workflow

1. Confirm the source `Part` or `Compound` is stable enough to document.
2. Build the border and title block with `TechnicalDrawing(...)`.
3. Project the smallest useful view set from the model.
   - Use front, top/plan, and right-side views by default.
   - Add a small isometric only when it improves orientation.
4. Place views with consistent scale and enough whitespace to read cleanly.
5. Add only binding dimensions, labels, and callouts.
6. Keep visible and hidden geometry separated by layer or line style.
7. Export SVG for review and DXF when downstream CAD interchange is needed.
8. In Problemologist planner/reviewer workspaces, call `preview_drawing()` on the current revision before submission or approval.

## Quality Rules

- Prefer orthographic layout over decorative composition.
- Keep the title block legible and away from the main views.
- Use one scale unless a detail view genuinely needs a different one.
- Dimension interfaces, clearances, and inspection-critical features only.
- Never add sections, datums, or labels that cannot be traced back to the source model.
- If the sheet gets crowded, remove a view before adding more annotation.
- If the drawing is hard to explain without extra prose, the view set is probably wrong.

## Reference

- [technical_drawing.md](references/technical_drawing.md)
- Official tutorial: https://build123d.readthedocs.io/en/latest/tech_drawing_tutorial.html
