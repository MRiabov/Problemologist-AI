# Architecture Scaffold: WP2 Part 3 — Validation & Rendering

## Overview

This document scaffolds the architecture for WP2 Part 3 (Validation & Rendering). The focus is on providing visual and quantitative feedback for FEM (stress) and MPM (fluids) simulations.

## 1. Components

### 1.1 `worker/utils/rendering.py` (Enhancements)
- **`render_stress_heatmap(msh_path: Path, stress_field: StressField) -> Path`**:
    - Use `PyVista` or `matplotlib` + `trimesh` to map von Mises stress to a color map (blue to red).
    - Render from standard angles or user-specified camera.
- **`VideoRenderer.add_particles(positions: np.ndarray, colors: np.ndarray)`**:
    - Ensure the existing `VideoRenderer` can overlay Genesis MPM particles.

### 1.2 `worker/simulation/loop.py` (Metric Logic)
- **`SimulationLoop.calculate_flow_rate(gate_plane: dict) -> float`**:
    - Identify particles crossing a specified plane between steps.
    - Convert to L/s based on particle volume and timestep.
- **`SimulationLoop.check_fluid_containment(zone: dict) -> float`**:
    - Calculate the ratio of particles within a specified bounding box.

### 1.3 `shared/models/schemas.py` (Schema Extensions)
- **`SimulationResult`**:
    - Add `stress_summaries: List[StressSummary]`.
    - Add `fluid_metrics: List[FluidMetricResult]`.
    - Add `stress_heatmap_url: Optional[str]`.

## 2. Updated Task List (Part 3)

| Task ID | Description | File(s) |
|---|---|---|
| **T010.1** | Implement `preview_stress` using PyVista for heatmaps. | `worker/utils/rendering.py` |
| **T011.1** | Integrate fluid particle rendering in `VideoRenderer`. | `worker/utils/rendering.py` |
| **T012.1** | Extend `SimulationResult` with stress/fluid data. | `shared/models/schemas.py` |
| **T013.1** | Implement `flow_rate` and `containment` metrics. | `worker/simulation/loop.py` |

## 3. Integration Tests Plan

### 3.1 `tests/integration/test_physics_fluids.py`
- **Scenario**: A simple bucket filled with water.
- **Verification**:
    - Simulation starts with `backend: genesis`.
    - Particles are initialized in a volume.
    - `fluid_containment` metric returns > 0.99 for a static bucket.
    - `VideoRenderer` produces a file with particles visible.

### 3.2 `tests/integration/test_physics_fem.py`
- **Scenario**: A cantilever beam with a heavy load.
- **Verification**:
    - Mesh is tetrahedralized correctly.
    - `get_stress_report` returns non-zero von Mises stress.
    - `PART_BREAKAGE` is triggered if load exceeds material strength.
