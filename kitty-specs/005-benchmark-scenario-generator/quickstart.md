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
