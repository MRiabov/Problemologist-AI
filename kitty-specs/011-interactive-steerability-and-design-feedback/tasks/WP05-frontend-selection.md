---
work_package_id: "WP05"
title: "Frontend Selection & Toggles"
lane: "planned"
dependencies: []
subtasks: ["T009"]
---

# WP05 - Frontend Selection & Toggles

## Objective
Update the `three-cad-viewer` integration to support multiple selection levels (Face, Part, Subassembly) and provide a UI toggle for the user to switch between them.

## Context
- Spec: `kitty-specs/011-interactive-steerability-and-design-feedback/spec.md` (User Story 1)
- UI: Design should follow existing Material/Bootstrap principles in the HereCRM project style.

## Subtasks

### T009: Implement Multi-Level Selection in CADViewer
**Purpose**: Allow users to select geometry at different abstractions.
**Steps**:
1. Modify `frontend/src/components/CADViewer/CADViewer.tsx`.
2. Implement a selection mode state: `selectionMode` (FACE | PART | SUBASSEMBLY).
3. Add a floating UI toggle (3-button group) to the viewer canvas.
4. Update the click handler:
    - **FACE**: Select the mesh with the `face_idx` prefix.
    - **PART**: Traverse up the scene graph from the clicked mesh to find the first group/object with a `part_id` or name.
    - **SUBASSEMBLY**: Traverse up to the first group that represents a subassembly in the BOM.
5. Highlight the entire selection and store its metadata in the current chat context state.
**Validation**:
- [ ] Toggling to "Part" mode and clicking a face highlights the whole part.
- [ ] Toggling to "Subassembly" mode highlights the group.

## Definition of Done
- Selection mode toggle is visible and functional in the CAD viewer.
- Correct metadata is captured for each selection level.
