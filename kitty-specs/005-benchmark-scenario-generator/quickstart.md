# Quickstart: Benchmark Scenario Generator

The Benchmark Scenario Generator is an automated pipeline designed to bridge the gap between high-level physics "puzzle" concepts and stable, validated MuJoCo environments.

## How it Works

The system operates as a self-correcting loop, ensuring that every generated scenario is physically viable before it reaches the final output.

1. **Planning**: A specialized agent decomposes your functional request (e.g., "a sliding drawer") into a concrete geometric and kinematic plan.
2. **Implementation**: The coder agent generates a `build123d` Python script. This script must implement a `build(seed: int) -> str` function that returns the model's MJCF XML.
3. **Validation**: A headless MuJoCo instance runs the model for 1,000 steps. If the physics "explode" or the solver crashes, the validation fails.
4. **Visual Artifacts**: For every stable scenario, the system automatically renders visual previews (PNGs) from multiple camera angles.
5. **Refinement**: If errors are detected, a critic agent analyzes the simulation telemetry (like high velocities or interpenetration) and provides corrective instructions to the coder for a second attempt.

## Getting Started

### Prerequisites

The generator requires a running **Controller** and at least one **Worker** node.

### CLI Execution (via Controller)

Trigger a generation episode on the distributed system:

```bash
# Generate 5 variations of a sliding drawer benchmark
python -m src.generators.benchmark.manager \
    --prompt "A sliding drawer with a handle" \
    --count 5 \
    --mode distributed
```

### API Integration

```python
import requests

# Submit a generation request to the Controller
response = requests.post(
    "http://controller:8000/api/benchmarks/generate",
    json={
        "prompt": "Elevator mechanism with gravity safety",
        "tier": "kinematic",
        "variation_count": 3
    }
)
```

## Verification & Iteration

1. **Observability**: Follow the generator agent's reasoning in **Langfuse**.
2. **Worker Logs**: If validation fails, check the worker's stdout in the `StepTrace`.
3. **Assets**: Access generated MJCF and renders via the S3 links provided in the final `BenchmarkScenario` record.

## The "Big Bang" Stability Test

Safety is enforced via `src/generators/benchmark/validator.py`. A scenario is only considered "stable" if it passes these gates:

- **Kinematic Stability**: Linear velocity must remain below 100m/s. This prevents "explosions" caused by overlapping geometries.
- **Solver Integrity**: The state vector must remain free of NaN (Not-a-Number) values.
- **Zero-Force Settling**: The simulation is run without external actuators to ensure the environment reaches a natural equilibrium under gravity.

## Advanced Randomization: Non-Uniform Rescaling

To maximize dataset diversity, the system applies non-uniform random scaling to every variation:

- **Default Range**: Each axis (X, Y, Z) is scaled by a random factor between **0.5 and 2.0**.
- **Agent Overrides**: The Planner can specify custom bounds in the plan if the geometry requires specific constraints (e.g., "keep Z-scale at 1.0 to preserve table height").
- **Kinematic Consistency**: Scaling is applied at the CAD level, ensuring joints and pivots are correctly recalculated for the new proportions.

## Visual Previews

Rendered images are saved in the `images/` subdirectory of each scenario:

- **Defined Cameras**: If the MJCF model defines `<camera>` elements, an image is rendered for each.
- **Default View**: A fallback image is generated from the default free camera.

## Directory Structure

- `datasets/benchmarks/<tier>_<id>/`: Root directory for a generation session.
  - `template.py`: The generated build123d script.
  - `scene_<seed>.xml`: Validated MJCF model.
  - `manifest_<seed>.json`: Metadata including descriptions and asset paths.
  - `images/`: Rendered visual previews (PNG).
  - `assets/`: Exported meshes (STL).

## Human-in-the-Loop & Debugging

When a design fails to converge, you can intervene manually:

1. **Inspect the Code**: The `template.py` in the output directory can be run directly.
2. **Manual Validation**: Test any MJCF file against the stability gates:

    ```bash
    uv run python -m src.generators.benchmark.validator --xml path/to/scene.xml
    ```

3. **Refine Constraints**: If the agent is struggling, try providing more explicit joint limits or clearance requirements in the input prompt.
