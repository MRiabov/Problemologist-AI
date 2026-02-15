---
work_package_id: WP06
title: Observability, Benchmarks & UI
lane: planned
dependencies: []
subtasks: [T021, T022, T023]
---

# WP06: Observability, Benchmarks & UI

## Objective
Finalize the delivery by adding observability events, rendering stress heatmaps, and updating the benchmark generator to create physics-heavy challenges.

## Context
We need to monitor the new physics features and provide visual feedback to users and agents. We also need to generate benchmarks that specifically test these new capabilities.

## Detailed Guidance

### T021: Observability events
- Add new event types to the telemetry system:
  - `part_breakage`: logs part, stress, location.
  - `fluid_containment_check`: logs ratio and threshold.
  - `meshing_failure`: logs error and repair status.
  - `simulation_backend_selected`: logs mujoco vs genesis.

### T022: Stress heatmap rendering
- Implement `render_stress_heatmap(part_label: str, simulation_result: SimulationResult) -> Path`.
- Use a library like `matplotlib` or a 3D visualization library to map stress values to a colormap (blue to red).
- Overlay the heatmap onto the part's mesh and render 2D views (similar to existing renders).

### T023: Benchmark generator updates
- Update the benchmark generator (likely in `scripts/` or `controller/benchmark/`).
- Enable the generator to produce `objectives.yaml` with:
  - `physics` section (toggling genesis/fem).
  - `fluids` definitions.
  - `fluid_objectives` and `stress_objectives`.
- Create 5-10 "canonical" WP2 benchmarks (e.g., "Water Tank", "Bridge Truss", "Soft Gripper").

## Test Strategy
- Verify telemetry events are emitted during a breakage simulation.
- Verify stress heatmap images are generated and saved to the correct directory.
- Verify benchmark planner generates valid WP2 objective schemas.

## Definition of Done
- [ ] New observability events are live.
- [ ] Stress heatmaps are rendered and included in results.
- [ ] Benchmark generator produces fluid and stress-based challenges.

## Risks
- Telemetry flood from periodic stress summaries (keep it throttled).
- Heatmap rendering performance on large meshes.
- Generator producing unsolvable physics benchmarks.
