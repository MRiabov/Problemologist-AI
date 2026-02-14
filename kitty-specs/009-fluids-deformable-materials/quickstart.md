# Quickstart: WP2 Fluids & Deformable Materials

## Prerequisites

- **Genesis Physics Engine**: `gs` must be installed.
- **TetGen/Gmsh**: Required for volumetric meshing.
- **trimesh**: Required for mesh repair.

## Running a Simulation

1. **Define Material**: Ensure `manufacturing_config.yaml` has FEM fields for your material.
2. **Configure Objective**: In `objectives.yaml`, set:

    ```yaml
    physics:
      backend: "genesis"
      fem_enabled: true
    ```

3. **Run**:

    ```bash
    source .venv/bin/activate
    # Run a benchmark that uses these materials/objectives
    python -m worker.main --benchmark <benchmark_id>
    ```

## Visualizing Results

1. Simulation generates `simulation.mp4` with particle overlays.
2. `SimulationResult` JSON contains:
    - `stress_data`: Per-node stress arrays for 3D viz.
    - `fluid_data`: Particle positions.

## Debugging

- If simulation doesn't start, check `events.jsonl` for `meshing_failure` or `schema_validation_error`.
- For GPU issues, use `export GS_BACKEND=cpu` to force CPU mode (or via `simulation_config.yaml`).
