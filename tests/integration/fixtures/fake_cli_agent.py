import argparse
import json
import os
import re
import shutil
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
ENGINEER_PLANNER_SEED = (
    REPO_ROOT / "dataset/data/seed/artifacts/engineer_coder/ec-004-raised-shelf-lift"
)
CLI_OUTPUT_START_MARKER = "__PROBLEMOLOGIST_OUTPUT_START__"
CLI_OUTPUT_END_MARKER = "__PROBLEMOLOGIST_OUTPUT_END__"


def _load_prompt(path: str) -> str:
    return Path(path).read_text(encoding="utf-8")


def _detect_node_type(prompt_text: str) -> str:
    match = re.search(
        r"You are the ([a-z_]+) node running inside the workspace root\.",
        prompt_text,
    )
    if match:
        return match.group(1).strip()
    return "generic"


def _should_emit_auth_prompt(prompt_text: str) -> bool:
    return "INT-190 CLI auth prompt fail closed" in prompt_text


def _copy_engineer_planner_artifacts() -> None:
    for name in (
        "plan.md",
        "todo.md",
        "benchmark_definition.yaml",
        "assembly_definition.yaml",
    ):
        shutil.copy2(ENGINEER_PLANNER_SEED / name, Path.cwd() / name)


def _write_benchmark_planner_artifacts() -> None:
    Path("plan.md").write_text(
        """# Benchmark Plan

## 1. Learning Objective
Create a simple gravity-routing problem with one forbid volume between spawn and goal.

## 2. Static Geometry
- support_wall: center [0, -35, 40], size [120, 6, 80]
- funnel_body: center [32, 10, 18], top radius 20, bottom radius 7, height 26

## 3. Input Objective
- moved_object: sphere radius 6
- start_position: [0, 0, 60]

## 4. Objectives
- goal_zone: min [28, 6, 1], max [40, 18, 9]
- forbid_zone vertical_drop_trap: min [-8, -8, 0], max [8, 8, 20]

## 5. Constraints
- build_zone: min [-40, -30, 0], max [40, 30, 80]

## 6. Randomization
- runtime_jitter: [2, 2, 1]
""",
        encoding="utf-8",
    )
    Path("todo.md").write_text(
        (
            "# TODO\n\n"
            "- [ ] Implement benchmark geometry\n"
            "- [ ] Validate goal and forbid zones\n"
        ),
        encoding="utf-8",
    )
    Path("benchmark_definition.yaml").write_text(
        """objectives:
  goal_zone:
    min: [28, 6, 1]
    max: [40, 18, 9]
  forbid_zones:
    - name: vertical_drop_trap
      min: [-8, -8, 0]
      max: [8, 8, 20]
  build_zone:
    min: [-40, -30, 0]
    max: [40, 30, 80]
  fluid_objectives: []
  stress_objectives: []
benchmark_parts:
  - part_id: environment_fixture
    label: environment_fixture
    metadata:
      fixed: true
      material_id: aluminum_6061
physics:
  backend: GENESIS
  fem_enabled: false
  compute_target: auto
fluids: []
simulation_bounds:
  min: [-60, -60, 0]
  max: [60, 60, 100]
moved_object:
  label: projectile_ball
  shape: sphere
  static_randomization:
    radius: [5, 7]
  start_position: [0, 0, 60]
  runtime_jitter: [2, 2, 1]
constraints:
  max_unit_cost: 45.0
  max_weight_g: 1100.0
randomization:
  static_variation_id: drop_ball_funnel_v2
  runtime_jitter_enabled: true
electronics_requirements: null
assembly_totals:
  estimated_unit_cost_usd: 18.0
  estimated_weight_g: 650.0
  budget_leeway_pct: 150.0
  weight_leeway_pct: 69.0
""",
        encoding="utf-8",
    )


def _emit_payload(payload: dict) -> None:
    print("CLI agent completed workspace edits.")
    print(CLI_OUTPUT_START_MARKER)
    print(json.dumps(payload))
    print(CLI_OUTPUT_END_MARKER)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--prompt-file", required=True)
    args = parser.parse_args()
    mode = os.getenv("FAKE_CLI_MODE", "").strip().lower()
    prompt_text = _load_prompt(args.prompt_file)
    node_type = _detect_node_type(prompt_text)

    if mode == "auth_prompt" or _should_emit_auth_prompt(prompt_text):
        print(
            "Opening authentication page in your browser. "
            "Do you want to continue? [Y/n]: "
        )
        return 0

    if node_type == "engineer_planner":
        _copy_engineer_planner_artifacts()
        _emit_payload({"summary": "CLI engineer planner completed."})
        return 0

    if node_type == "benchmark_planner":
        _write_benchmark_planner_artifacts()
        _emit_payload(
            {
                "reasoning": "Created a simple forbid-zone gravity benchmark.",
                "plan": {
                    "theme": "simple_gravity_forbid_zone",
                    "target_object_properties": {
                        "label": "projectile_ball",
                        "shape": "sphere",
                    },
                    "environment_perturbations": {
                        "runtime_jitter": [2, 2, 1],
                    },
                    "difficulty_score": 0.35,
                    "reasoning": "Benchmark is solvable and uses a single forbid zone.",
                },
            }
        )
        return 0

    print("Generic CLI response.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
