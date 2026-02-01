# Quickstart: Benchmark Scenario Generator (Physics Puzzles)

This feature allows you to generate stable, validated MuJoCo physics puzzles from natural language descriptions.

## 🚀 Core Concepts

The generator uses a self-correcting loop:
1.  **Planner**: Decomposes your request into a geometric plan.
2.  **Coder**: Generates a `build123d` Python script that outputs MJCF XML.
3.  **Validator**: Runs the generated XML in a headless MuJoCo simulation for 1 second to ensure it doesn't "explode" or crash.
4.  **Critic**: (Optional) Fixes the code if validation fails.

## 🛠 Prerequisites

Ensure you have the following installed (managed via `uv`):
- `mujoco` (Physics Engine)
- `build123d` (CAD Library)
- `trimesh` (Geometry processing)

## 🏃 Running the Generator

You can trigger a generation using the Benchmark Manager.

### Option 1: CLI Execution
Run the manager directly from the project root:

```bash
uv run python -m src.generators.benchmark.manager --scenario "A sliding drawer with a handle"
```

### Option 2: Programmatic Usage
```python
from src.generators.benchmark.manager import BenchmarkManager

manager = BenchmarkManager()
result = manager.generate("A box containing three small spheres")

if result.stable:
    print(f"Success! MJCF saved to {result.xml_path}")
else:
    print(f"Failed to generate stable scenario: {result.error}")
```

## 🔍 Validation & Stability

Every scenario is automatically validated using the "Big Bang" test in `src/generators/benchmark/validator.py`:
- **Gravity Check**: Does it settle or collapse immediately?
- **Velocity Check**: Do any parts exceed 100m/s (indicates unstable physics)?
- **NaN Check**: Does the solver crash?

## 🔄 Example Workflow (End-to-End)

Here is how the generator handles a typical request:

1.  **User Request**: "Create a sliding drawer with a handle on the front."
2.  **Planner Agent**: 
    - "We need two main bodies: a fixed `housing` and a mobile `drawer`."
    - "The `drawer` needs a `SliderJoint` along the X-axis."
    - "The `handle` should be a separate child body or part of the drawer union."
3.  **Coder Agent**: Writes a `build123d` script that defines the housing as a hollow box and the drawer as a slightly smaller box with a cylinder handle.
4.  **Validator (Attempt 1)**: 
    - *Result*: **FAILED**. 
    - *Reason*: The drawer was spawned exactly touching the housing, causing a "collision explosion" (velocity > 100m/s).
5.  **Critic Agent**: "Add a 1mm clearance (gap) between the drawer and the housing to prevent initial penetration."
6.  **Coder Agent**: Updates the script with `offset` or smaller dimensions for the drawer.
7.  **Validator (Attempt 2)**: 
    - *Result*: **PASSED**. 
    - *Reason*: Simulation runs for 1.0s; the drawer settles naturally under gravity.
8.  **Output**: The stable MJCF XML is saved and the `ScenarioManifest` is generated.

## 🧑‍💻 Human-in-the-Loop (Debugging)

If the agent is failing to create a stable scenario:

1.  **Check the Workspace**: Look in the `workspace/` directory for the latest generated Python script.
2.  **Manual Tweak**: You can manually edit the Python script to fix joint limits or overlapping geometry.
3.  **Local Validation**: Run the validator against your modified XML:
    ```bash
    uv run python -m src.generators.benchmark.validator --xml workspace/latest.xml
    ```

## 📁 Output Structure

- `output/scenarios/`: Final validated MJCF XML files.
- `output/manifests/`: JSON metadata describing the puzzle (labels, goals, constraints).
- `workspace/`: Intermediate Python scripts and logs from the generation loop.
