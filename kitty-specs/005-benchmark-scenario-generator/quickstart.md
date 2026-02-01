# Quickstart: Benchmark Scenario Generator (Physics Puzzles)

The Benchmark Scenario Generator automates the creation of stable MuJoCo physics environments from high-level technical requirements.

## System Architecture

The generation process follows a deterministic self-correcting loop:

1.  **Planner**: Decomposes the functional request into a geometric and kinematic plan.
2.  **Coder**: Generates a build123d Python script. This script must implement a `build(seed: int) -> str` function that returns an MJCF XML string.
3.  **Validator**: Executes a headless MuJoCo simulation for 1,000 steps (1.0s at dt=0.001) to verify kinematic stability.
4.  **Critic**: Analyzes simulation telemetry (NaNs, excessive velocities) to provide corrective feedback if the initial design is unstable.

## Prerequisites

The system requires a Python 3.12+ environment with the following dependencies:
- `mujoco >= 3.4.0`
- `build123d >= 0.10.0`
- `trimesh[easy] >= 4.11.1`

## Operational Usage

### CLI Execution
To generate a new scenario from the terminal:

```bash
uv run python -m src.generators.benchmark.manager --scenario "A sliding drawer with a handle"
```

### Programmatic Integration
```python
from src.generators.benchmark.manager import BenchmarkManager

manager = BenchmarkManager()
result = manager.generate("A box containing three small spheres")

if result.stable:
    # xml_path contains the path to the validated MJCF file
    print(f"Validation Successful: {result.xml_path}")
else:
    # error contains the MuJoCo solver error or Big Bang violation
    print(f"Validation Failed: {result.error}")
```

## Validation Constraints

Scenarios are subjected to the "Big Bang" stability test in `src/generators/benchmark/validator.py`:

- **Kinematic Stability**: Linear velocity must not exceed 100m/s in a zero-external-force environment.
- **Solver Integrity**: The simulation must not produce NaN (Not-a-Number) values in the state vector.
- **Interpenetration**: Initial contact forces must be within normal bounds to prevent collision explosions.

## Technical Debugging (HITL)

If the generator fails to converge on a stable design:

1.  **Inspect Intermediate Artifacts**: Check the `workspace/` directory for the `latest_attempt.py` script.
2.  **Manual Verification**: Run the standalone validator against an MJCF file:
    ```bash
    uv run python -m src.generators.benchmark.validator --xml workspace/output.xml
    ```
3.  **Refine Constraints**: Adjust the `ScenarioManifest` in the input JSON to provide more explicit joint limits or clearances.

## Directory Structure

- `output/scenarios/`: Final validated MJCF XML files.
- `output/manifests/`: JSON metadata (labels, goals, constraints).
- `workspace/`: Temporary Python scripts, logs, and simulation telemetry.

## Example Workflow (End-to-End)

1.  **Input**: "Create a sliding drawer with a handle on the front."
2.  **Planner**: Identifies two bodies (`housing` and `drawer`) and a `SliderJoint` on the X-axis.
3.  **Coder**: Defines a hollow box (housing) and a smaller box (drawer) with a cylinder handle.
4.  **Validator (Attempt 1)**: Fails. The drawer spawned exactly touching the housing, causing a collision explosion (velocity > 100m/s).
5.  **Critic**: Instructs the Coder to add a 1mm clearance (gap) between the drawer and housing.
6.  **Coder**: Updates the script with 1mm offset.
7.  **Validator (Attempt 2)**: Passes. Simulation settles naturally under gravity.
8.  **Output**: Final MJCF XML and ScenarioManifest are persisted to `output/`.