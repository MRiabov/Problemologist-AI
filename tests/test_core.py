import shutil
from pathlib import Path
from unittest.mock import MagicMock

import pytest

from src.environment.runtime import ToolRuntime


@pytest.fixture
def runtime(tmp_path):
    # Setup a clean workspace for testing using tmp_path
    workspace = tmp_path / "test_workspace"
    workspace.mkdir()

    rt = ToolRuntime(
        workspace_dir=str(workspace),
        db=None,  # No DB for basic runtime tests
    )

    # Mock sim bridge
    rt.sim_bridge.load_template = MagicMock(return_value="<mujoco/>")
    rt.sim_bridge.inject_design = MagicMock(return_value="<mujoco/>")
    mock_res = MagicMock()
    mock_res.success = True
    mock_res.total_energy = 10.0
    mock_res.total_damage = 0.0
    rt.sim_bridge.run_simulation = MagicMock(return_value=mock_res)

    # Mock evaluator to avoid full evaluation overhead in core logic tests
    rt.evaluator.preview_design = MagicMock(return_value="preview.svg")

    # Mock validate_and_export for budget tests
    mock_report = MagicMock()
    mock_report.status = "pass"
    mock_report.stl_path = "model.stl"
    mock_report.error = None
    mock_report.cost_analysis.unit_cost = 50.0  # Default affordable
    rt.evaluator.validate_and_export = MagicMock(return_value=mock_report)

    yield rt

    # Cleanup
    if workspace.exists():
        shutil.rmtree(workspace)


def test_runtime_budget_rejection(runtime):
    # Set mock cost to be high
    runtime.evaluator.validate_and_export.return_value.cost_analysis.unit_cost = 200.0

    runtime.dispatch(
        "write_file",
        {
            "content": "from build123d import Box\npart = Box(10, 10, 10)",
            "path": "design.py",
        },
    )

    # Try submit with low budget
    output = runtime.dispatch(
        "submit_design", {"control_path": "control.py", "max_unit_cost": 100.0}
    )

    # Output is JSON string, checking for keywords
    assert "REJECTED" in output
    assert "exceeds budget" in output


def test_runtime_force_submit(runtime):
    # Set mock cost to be high
    runtime.evaluator.validate_and_export.return_value.cost_analysis.unit_cost = 200.0

    runtime.dispatch(
        "write_file",
        {
            "content": "from build123d import Box\npart = Box(10, 10, 10)",
            "path": "design.py",
        },
    )

    # Force submit should bypass budget check
    output = runtime.dispatch(
        "submit_design",
        {"force_submit": True, "control_path": "control.py", "max_unit_cost": 100.0},
    )

    assert "REJECTED" not in output
    assert "Submission Result: SUCCESS" in output


def test_runtime_dispatch_write(runtime):
    output = runtime.dispatch(
        "write_file",
        {
            "content": "from build123d import Box\npart = Box(10, 10, 10)",
            "path": "design.py",
        },
    )
    assert "Successfully wrote to design.py" in output
    assert (Path(runtime.workspace_dir) / "design.py").exists()


def test_runtime_dispatch_preview(runtime):
    runtime.dispatch(
        "write_file",
        {
            "content": "from build123d import Box\npart = Box(10, 10, 10)",
            "path": "design.py",
        },
    )

    output = runtime.dispatch("preview_design", {"filename": "design.py"})
    assert "preview.svg" in output


def test_runtime_instantiation():
    rt = ToolRuntime(workspace_dir="/tmp/test")
    assert isinstance(rt, ToolRuntime)
