---
work_package_id: WP04
title: Visual Preview System
lane: "doing"
dependencies: [WP01, WP02]
base_branch: main
base_commit: 8c03b173e1fcc15efbccc86eac07f8eabae5d48f
created_at: '2026-02-01T16:27:20.913179+00:00'
subtasks:
- T015
- T016
- T017
- T018
- T019
shell_pid: "500185"
agent: "gemini"
assignee: "Antigravity"
review_status: "has_feedback"
reviewed_by: "MRiabov"
---

# WP04: Visual Preview System

**Goal**: Implement the rendering pipeline and description system.
**Role**: Provides the "eyes" for the agent.

## Subtasks

### T015: Create Description Database

**Purpose**: Store static text descriptions for parts.
**Location**: `src/assets/cots_descriptions.json`
**Steps**:

1. Create JSON file.
2. Structure: `{"partial_id": {"description": "...", "metadata": {...}}}`.
3. Add entries for Nema17, 608 Bearing, M6 Screw.
   - Example Nema17: "A standard stepper motor. Shaft aligned with Z-axis. Mounting holes on XY plane."

### T016: Load Descriptions

**Purpose**: Helper to fetch text for a part.
**Location**: `src/cots/utils.py` (or similar)
**Steps**:

1. Implement `load_descriptions()`.
2. Implement `get_description(part_id)`:
   - Exact match first.
   - Fallback to partial match (e.g., if ID is `...:Nema17`, match `Nema17` key).

### T017: Rendering Logic

**Purpose**: Convert `build123d` object to image.
**Location**: `src/cots/rendering.py`
**Steps**:

1. Implement `render_part(part_obj, output_path)`.
2. Use `build123d` export to STL/SVG or `vtk` if available.
3. Simple approach: Export SVG (easy, lightweight) or PNG via offscreen render. *Decision*: SVG is text, Agent needs image? VLM can read images. PNG is better. Use `export_png` if available or `vtk` offscreen.
   - *Fallback*: If rendering is too hard without display, return a placeholder path and log warning.

### T018: Integrate to Provider

**Purpose**: Connect rendering to `get_preview`.
**Location**: `src/cots/providers/bd_warehouse.py`
**Steps**:

1. In `get_preview`:
   - Instantiate the part using the factory.
   - Call `render_part`.
   - Call `get_description`.
   - Return populated `PartPreview`.

### T019: Caching

**Purpose**: Don't re-render static parts.
**Location**: `src/cots/rendering.py`
**Steps**:

1. Check if `hash(part_id).png` exists in temp/cache dir.
2. If yes, return path.

## Definition of Done

- `cots_descriptions.json` exists.
- `render_part` produces a file (or robustly handles failure).
- `get_preview` returns valid `PartPreview` object with image path and text.

## Activity Log

- 2026-02-01T16:43:15Z – unknown – shell_pid=313636 – lane=for_review – Implemented visual preview system with pyvista rendering and hash-based caching. Added description database. Rebased onto main.
- 2026-02-01T16:47:36Z – Antigravity – shell_pid=313636 – lane=doing – Started review via workflow command
- 2026-02-01T16:49:09Z – Antigravity – shell_pid=313636 – lane=planned – Changes requested: missing .gitignore, incomplete test assertions, and missing placeholder logic.
- 2026-02-01T17:28:08Z – Antigravity – shell_pid=313636 – lane=done – Review passed: Implementation is robust, tests pass, and previous feedback has been fully addressed. Rebased onto main.
- 2026-02-01T17:56:50Z – gemini – shell_pid=500185 – lane=doing – Started review via workflow command
