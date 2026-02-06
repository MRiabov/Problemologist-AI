---
work_package_id: "WP01"
title: "Geometry Pipeline & Scene Builder"
lane: "doing"
dependencies: []
subtasks: ["T001", "T002", "T003", "T004", "T005", "T006"]
agent: "Gemini"
shell_pid: "716159"
---

# WP01: Geometry Pipeline & Scene Builder

## Objective

Enable the transformation of CAD models (`build123d` objects) into physics-ready MuJoCo assets (MJCF XML + STLs). This is the foundation of the simulation engine.

## Context

The `src/worker/simulation` module runs inside the Worker Node. We need a robust pipeline that takes geometry, processes it for physics stability (convex decomposition), and generates a valid `scene.xml` that MuJoCo can load.
Key challenge: MuJoCo handles concave objects poorly compared to convex hulls. We must decompose geometry or generate accurate hulls.

## Implementation Guide

### T001: Initialize Package & Dependencies

**Goal**: Set up `src/worker/simulation` and install libraries.

1. Create directories: `src/worker/simulation/` and `src/worker/utils/`.
2. Ensure `__init__.py` in directories.
3. Install/Verify dependencies in `pyproject.toml` or environment:
   - `mujoco>=3.1.0`
   - `build123d`
   - `trimesh[easy]` (includes scipy/networkx)
   - `scipy` (required for convex hull)

### T002: Implement MeshProcessor

**Goal**: Convert `build123d` Compound/Solid to STL/GLB.
**File**: `src/worker/simulation/builder.py`

1. Create `MeshProcessor` class.
2. Method: `process_geometry(part: Compound, filepath: Path) -> Path`:
   - Use `build123d.export_stl` or iterate faces to create a `trimesh.Trimesh` object.
   - Ideally, `trimesh` can load STL bytes directly from an in-memory export.
   - Save the raw mesh to `filepath`.

### T003: Implement Convex Decomposition Strategy

**Goal**: Ensure physics stability using convex hulls.
**File**: `src/worker/simulation/builder.py`

1. Add `compute_convex_hull(mesh: trimesh.Trimesh) -> trimesh.Trimesh` method.
2. Use `mesh.convex_hull` property (wrapper around `scipy.spatial.ConvexHull`).
3. **Advanced**: If `vhacd` is available (via `pybullet` or independent bin), use it for better concave approximation. For MVP, standard convex hull per sub-part is acceptable.
4. If a part is deeply concave (like a cup), it must be split into multiple convex meshes, OR marked as a static mesh (visual only) if no collisions needed inside.
   - For MVP: Treat every "Part" in the assembly as a separate convex hull.

### T004: Implement SceneCompiler

**Goal**: Generate MJCF XML structure.
**File**: `src/worker/simulation/builder.py`

1. Create `SceneCompiler` class.
2. Define base template (XML string) with:
   - `<statistic extent="2" center="0 0 1"/>`
   - `<visual> <headlight diffuse="0.6 0.6 0.6"/> ... </visual>`
   - `<worldbody> <light ... /> <geom name="floor" .../> </worldbody>`
3. Method: `add_body(name: str, mesh_path: str, pos: list, rot: list)`:
   - Appends `<body name="..."> <geom type="mesh" mesh="..."/> </body>` to worldbody.
4. Method: `save(path: Path)`:
   - Writes the final XML file.

### T005: Implement "Zone Logic"

**Goal**: Map specially named parts to Sites/Sensors.
**File**: `src/worker/simulation/builder.py` -> `SceneCompiler`

1. Update loop that iterates over assembly children.
2. Check `child.label` or name:
   - If `startswith("zone_goal")`: Do NOT add a collision geom. Add `<site name="..." rgba="0 1 0 0.3"/>` (transparent green ghost).
   - If `startswith("zone_forbid")`: Add `<site name="..." rgba="1 0 0 0.3"/>` (transparent red ghost). OR keep as geom but use a specific `contype/conaffinity` to detect but not block (phantom object).
   - **Recommendation**: Use `<site>` for logical zones, it's cheaper and intended for sensing.

### T006: Add Unit Tests

**File**: `tests/worker/simulation/test_builder.py` (create if needed)

1. Test `MeshProcessor`: Create a Box with build123d, convert to STL, assert file exists and size > 0.
2. Test `SceneCompiler`: Add a body, save XML, assert valid XML syntax.
3. Test `mujuco_loading`: string parse the generated XML with `mujoco.MjModel.from_xml_path` to prove it works.

## Validation

- [ ] `pytest tests/worker/simulation/test_builder.py` passes.
- [ ] Generated XML opens in MuJoCo viewer (using `python -m mujoco.viewer.mjviewer model.xml` manually if checked).

## Risks

- **Complex Geometry**: Deeply concave parts will act like "shrink-wrapped" bubbles if using simple convex hull.
  - *Mitigation*: Warn user/agent to split complex parts or use primitives.

## Activity Log

- 2026-02-06T08:06:52Z – gemini-agent – shell_pid=481649 – lane=doing – Started implementation via workflow command
- 2026-02-06T14:13:08Z – gemini-agent – shell_pid=481649 – lane=for_review – Ready for review: Implemented geometry pipeline and scene builder with VHACD support and zone logic improvements.
- 2026-02-06T15:39:28Z – Gemini – shell_pid=716159 – lane=doing – Started review via workflow command
