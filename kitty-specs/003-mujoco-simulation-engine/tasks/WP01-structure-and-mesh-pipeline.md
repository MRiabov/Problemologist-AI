---
work_package_id: WP01
title: Project Structure & Mesh Pipeline
lane: "for_review"
dependencies: []
subtasks:
  - T001
  - T002
  - T003
  - T004
agent: "Antigravity"
shell_pid: "60800"
---

# WP01: Project Structure & Mesh Pipeline

## Objective

Initialize the `simulation_engine` Python package and implement the core geometry processing pipeline. This foundation allows converting complex CAD models (from `build123d`) into physics-ready meshes with proper convex decomposition.

## Context

The standard `build123d` export produces visual meshes (STL). For physics simulation in MuJoCo, we need:

1. Valid convex hulls (concave shapes break collision detection).
2. Clean topology (no holes or degenerate faces).
3. A pipeline to process these efficiently on the fly.

This work package sets up the project structure and this critical geometry utility.

## Subtasks

### T001: Initialize Project Structure

**Goal**: Create the Python package structure and install dependencies.
**Implementation**:

1. Create directory `src/simulation_engine/`.
2. Create `src/simulation_engine/__init__.py`.
3. Update `pyproject.toml` (or `requirements.txt` equivalent) to include:
   - `mujoco>=3.1.0`
   - `fastapi`
   - `uvicorn`
   - `build123d`
   - `trimesh[easy]` (includes `scipy` for basic hull) or `pyacvd` if needed.
   - `numpy`
4. Create `tests/simulation_engine/` directory.

### T002: Implement MeshProcessor

**Goal**: Logic to accept `build123d` Compound/Solid and export to mesh.
**Implementation**:

- File: `src/simulation_engine/builder.py`
- Create class `MeshProcessor`.
- Method `export_stl(solid: Solid) -> bytes`: Use `build123d.export_stl` functionality.
- Method `load_mesh(stl_data: bytes) -> trimesh.Trimesh`: Load into trimesh object.

### T003: Implement Convex Decomposition

**Goal**: Convert generic meshes into convex hulls for stable physics.
**Implementation**:

- File: `src/simulation_engine/builder.py`
- Method `compute_convex_hull(mesh: trimesh.Trimesh) -> trimesh.Trimesh`:
  - Use `mesh.convex_hull` (built-in trimesh wrapper around qhull).
  - This is a "good enough" approximation for V1. Later we might use VHACD for concave decomposition (breaking 1 concave into N convex), but for now, simple convex hull is the requirement.
  - NOTE: If the user provides a concave definition (like a cup), a single convex hull covers the opening.
  - REFINEMENT: If we need better accuracy, check if `shapely` or `scipy` is available, but start with standard `trimesh.convex_hull`.

### T004: Unit Tests for Mesh Pipeline

**Goal**: Verify geometry flows correctly.
**Implementation**:

- File: `tests/simulation_engine/test_builder.py`
- Test 1: Create a `build123d.Box`. Export it. Verify `MeshProcessor` loads it.
- Test 2: Create a complex shape. Verify `compute_convex_hull` returns a watertight mesh with `is_convex=True`.

## Definition of Done

- [ ] `src/simulation_engine` exists and is importable.
- [ ] Dependencies are locked and installed.
- [ ] `builder.py` can take a `build123d` object and produce a Convex Trimesh.
- [ ] Tests pass.

## Activity Log

- 2026-02-01T07:36:10Z – Antigravity – shell_pid=60800 – lane=doing – Started implementation via workflow command
- 2026-02-01T07:38:35Z – Antigravity – shell_pid=60800 – lane=for_review – Ready for review
