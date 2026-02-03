import json
import shutil
from pathlib import Path
from unittest.mock import MagicMock

import pytest

from src.environment.core import CADEnv


@pytest.fixture
def env():
    # Setup a clean workspace for testing
    workspace = Path("test_workspace")
    if workspace.exists():
        shutil.rmtree(workspace)

    # Set a low budget for testing rejection
    env = CADEnv(
        workspace_dir=str(workspace),
        db_url="sqlite:///:memory:",
        max_unit_cost=100.0,
        target_quantity=1,
    )

    # Mock sim bridge
    env.sim_bridge.load_template = MagicMock(return_value="<mujoco/>")
    env.sim_bridge.inject_design = MagicMock(return_value="<mujoco/>")
    mock_res = MagicMock()
    mock_res.success = True
    mock_res.energy = 10.0
    mock_res.damage = 0.0
    env.sim_bridge.run_simulation = MagicMock(return_value=mock_res)

    yield env

    # Cleanup
    if workspace.exists():
        shutil.rmtree(workspace)


def test_env_budget_rejection(env):
    env.reset()
    env.max_unit_cost = 1.0  # Set low budget

    env.dispatch(
        "write_file",
        {
            "content": "from build123d import Box\npart = Box(10, 10, 10)",
            "path": "design.py",
        },
    )

    output = env.dispatch("submit_design", {})

    assert "REJECTED" in output
    assert "exceeds budget" in output
    # Check if reward was logged (it should be in tool_output if we added it, but let's check what core.py does)
    # Actually, in core.py reward is returned by _submit_design but only added to tool_output if terminated is True.
    # For rejection, terminated is False.

    # We can check the DB if needed, but for now we just verify rejection message.


def test_env_force_submit(env):
    env.reset()
    env.dispatch(
        "write_file",
        {
            "content": "from build123d import Box\npart = Box(10, 10, 10)",
            "path": "design.py",
        },
    )

    # Force submit should bypass budget check
    output = env.dispatch(
        "submit_design", {"force_submit": True, "reason": "Testing bypass"}
    )

    assert "REJECTED" not in output
    assert "[TERMINATED]" in output
    assert "Reward:" in output


def test_env_reset(env):
    obs, _ = env.reset()
    assert "code" in obs
    assert obs["last_output"] == "Environment reset. Ready for new design."
    assert (Path(env.workspace_dir) / "design.py").exists()


def test_env_dispatch_write(env):
    env.reset()
    output = env.dispatch(
        "write_file",
        {
            "content": "from build123d import Box\npart = Box(10, 10, 10)",
            "path": "design.py",
        },
    )
    assert "Successfully wrote to design.py" in output
    assert "Box(10, 10, 10)" in env.last_obs["code"]


def test_env_dispatch_preview(env):
    env.reset()
    env.dispatch(
        "write_file",
        {
            "content": "from build123d import Box\npart = Box(10, 10, 10)",
            "path": "design.py",
        },
    )

    output = env.dispatch("preview_design", {"filename": "design.py"})
    assert "Preview generated:" in output
    assert env.last_obs["last_render"].endswith(".svg")
    assert (Path(env.workspace_dir) / env.last_obs["last_render"]).exists()


def test_env_dispatch_submit_invalid(env):
    env.reset()
    env.dispatch("write_file", {"content": "invalid python code", "path": "design.py"})

    output = env.dispatch("submit_design", {})
    assert "Error processing design" in output


def test_env_dispatch_submit_success(env):
    env.reset()
    env.dispatch(
        "write_file",
        {
            "content": "from build123d import Box\npart = Box(10, 10, 10)",
            "path": "design.py",
        },
    )

    output = env.dispatch("submit_design", {})
    assert "[TERMINATED]" in output
    assert "Submission Result: SUCCESS" in output


def test_cadenv_instantiation():
    env = CADEnv()
    assert isinstance(env, CADEnv)
