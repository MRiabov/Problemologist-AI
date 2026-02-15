---
work_package_id: "WP05"
title: "Frontend Selection & Toggles"
lane: "doing"
dependencies: []
subtasks: ["T009"]
agent: "gemini-cli"
shell_pid: "203377"
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

## Activity Log

- 2026-02-15T10:26:55Z – Gemini – shell_pid=132921 – lane=doing – Started review via workflow command
- 2026-02-15T10:55:21Z – gemini-cli – shell_pid=132921 – lane=for_review – Implemented multi-level selection (FACE, PART, SUBASSEMBLY) in ModelViewer. Added floating UI toggle for selection modes. Implemented automatic scene graph traversal for hierarchical selection and visual highlighting of selected groups.
- 2026-02-15T11:32:05Z – gemini-cli – shell_pid=203377 – lane=doing – Started implementation via workflow command
