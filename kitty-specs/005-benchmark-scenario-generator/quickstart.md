# Quickstart: Benchmark Scenario Generator

The Benchmark Scenario Generator is an automated pipeline designed to bridge the gap between high-level physics "puzzle" concepts and stable, validated MuJoCo environments.

## How it Works

The system operates as a self-correcting loop, ensuring that every generated scenario is physically viable before it reaches the final output.

1.  **Planning**: A specialized agent decomposes your functional request (e.g., "a sliding drawer") into a concrete geometric and kinematic plan.
2.  **Implementation**: The coder agent generates a `build123d` Python script. This script must implement a `build(seed: int) -> str` function that returns the model's MJCF XML.
3.  **Validation**: A headless MuJoCo instance runs the model for 1,000 steps. If the physics "explode" or the solver crashes, the validation fails.
4.  **Refinement**: If errors are detected, a critic agent analyzes the simulation telemetry (like high velocities or interpenetration) and provides corrective instructions to the coder for a second attempt.

## Getting Started

### Prerequisites

Ensure your environment includes:
*   **MuJoCo** (>= 3.4.0) for physics simulation.
*   **build123d** (>= 0.10.0) for procedural CAD generation.
*   **trimesh** (>= 4.11.1) for geometry processing.

### Basic Execution

To generate a new scenario via the command line, simply provide a descriptive string:

```bash
uv run python -m src.generators.benchmark.manager --scenario "A sliding drawer with a handle"
```

### Programmatic Integration

For integration into other services or agents, use the `BenchmarkManager`:

```python
from src.generators.benchmark.manager import BenchmarkManager

manager = BenchmarkManager()
result = manager.generate("A box containing three small spheres")

if result.stable:
    print(f"Scenario validated and saved to: {result.xml_path}")
else:
    print(f"Generation failed: {result.error}")
```

## The "Big Bang" Stability Test

Safety is enforced via `src/generators/benchmark/validator.py`. A scenario is only considered "stable" if it passes these three gates:

*   **Kinematic Stability**: Linear velocity must remain below 100m/s. This prevents "explosions" caused by overlapping geometries.
*   **Solver Integrity**: The state vector must remain free of NaN (Not-a-Number) values.
*   **Zero-Force Settling**: The simulation is run without external actuators to ensure the environment reaches a natural equilibrium under gravity.

## Human-in-the-Loop & Debugging

When a design fails to converge, you can intervene manually:

1.  **Inspect the Code**: The `workspace/` directory contains the `latest_attempt.py` script. You can run this directly to see the raw build123d output.
2.  **Manual Validation**: Test any MJCF file against the stability gates:
    ```bash
    uv run python -m src.generators.benchmark.validator --xml workspace/output.xml
    ```
3.  **Refine Constraints**: If the agent is struggling, try providing more explicit joint limits or clearance requirements in the input prompt.

## Example: From Request to Result

1.  **Input**: "Create a sliding drawer with a handle on the front."
2.  **Decomposition**: The Planner identifies a static `housing`, a mobile `drawer`, and a `SliderJoint`.
3.  **Initial Attempt**: The Coder generates geometry where the drawer and housing share a boundary.
4.  **Failure**: The Validator detects a collision explosion (velocity > 100m/s) because the parts were intersecting at spawn.
5.  **Correction**: The Critic identifies the intersection and suggests a 1.0mm clearance gap.
6.  **Success**: The Coder applies the gap; the second simulation settles perfectly.
7.  **Output**: The MJCF XML and its JSON metadata (ScenarioManifest) are persisted to the `output/` directory.
