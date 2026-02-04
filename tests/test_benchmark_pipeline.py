import shutil
from pathlib import Path
from unittest.mock import patch

import pytest

from src.generators.benchmark.manager import execute_build

# Mock script that defines a build function
MOCK_SCRIPT = """
# Core build123d components
from build123d import (
    Box, Cylinder, Sphere, Torus, Cone, Wedge, 
    Compound, Solid, Part, Location, Rotation, Vector, Axis, Plane,
    Mode, Align, Unit, Shell
)

# Common build123d operations
from build123d import (
    fillet, chamfer, split, mirror, scale, 
    extrude, revolve, loft, sweep, offset
)

# standard builders (Use BuildPart for CSG)
from build123d import BuildPart, BuildSketch, BuildLine

import math
import random

def build(seed, scale_factors=(1,1,1)):
    with BuildPart() as p:
        Box(10, 10, 10)
    # The to_mjcf function is injected by execute_build
    return to_mjcf(env_compound=p.part, env_labels=[f"obstacle_box_{seed}"])
"""


@pytest.fixture
def clean_datasets():
    output_dir = Path("test_datasets")
    if output_dir.exists():
        shutil.rmtree(output_dir)
    yield output_dir
    if output_dir.exists():
        shutil.rmtree(output_dir)


@pytest.mark.benchmark
@pytest.mark.benchmark
@patch("src.generators.benchmark.manager.run_sandboxed_script")
def test_execute_build(mock_run):
    mock_run.return_value = {
        "mjcf": "<mujoco><worldbody><geom name='geom_obstacle_box'/></worldbody></mujoco>",
        "error": None,
    }
    mjcf, _ = execute_build(MOCK_SCRIPT, 42)
    assert (
        "geom_obstacle_box" in mjcf
    )  # matches the name in the MOCK_SCRIPT return logic
    assert "<mujoco" in mjcf


@pytest.mark.benchmark
@patch("src.generators.benchmark.manager.validate_mjcf")
@patch("src.generators.benchmark.manager.execute_build")
@patch("src.generators.benchmark.manager.render_scenario")
@patch("src.generators.benchmark.manager.generator_agent.invoke")
def test_generate_pipeline(
    mock_invoke, mock_render, mock_execute, mock_validate, clean_datasets
):
    # Mock agent returning a valid script
    mock_invoke.return_value = {
        "validation_passed": True,
        "code": MOCK_SCRIPT,
        "request": "Test Prompt",
    }

    # Mock execute_build to return dummy MJCF and asset path
    # returns (mjcf_xml, asset_dir_relative_path)
    mock_execute.return_value = ("<mujoco/>", "temp_assets")

    # Mock validate to pass
    mock_validate.return_value = {"is_valid": True, "error_message": None}

    # Mock render_scenario to return paths relative to scenario_dir
    def mock_render_side_effect(_xml, prefix, **_kwargs):
        # The prefix already includes the scenario_dir/images/...
        return [f"{prefix}_default.png"]

    mock_render.side_effect = mock_render_side_effect

    from typer.testing import CliRunner

    from src.generators.benchmark.manager import app

    runner = CliRunner()
    result = runner.invoke(
        app,
        [
            "--prompt",
            "Test Prompt",
            "--count",
            "2",
            "--output-dir",
            str(clean_datasets),
        ],
    )

    print(result.output)
    assert result.exit_code == 0
    # Rich console output capturing can be tricky in tests, relying on side effects (files) is more robust
    # assert "Template generated successfully!" in result.output

    # Check if files were created
    # Find the generated directory (it has a random suffix)
    dirs = list(clean_datasets.iterdir())
    assert len(dirs) == 1
    scenario_dir = dirs[0]

    assert (scenario_dir / "template.py").exists()

    # We requested 2 variations
    files = list(scenario_dir.iterdir())
    mjcf_files = [f for f in files if f.name.startswith("scene_")]
    manifest_files = [f for f in files if f.name.startswith("manifest_")]

    assert len(mjcf_files) == 2
    assert len(manifest_files) == 2

    # We can't easily check assets since we mocked execute_build and didn't create real files
    # But we can check that images were logically "rendered" (paths in manifest or just function called)
