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

    # Use in-memory DB to avoid I/O errors and locking
    # db_path = Path("test_history.db")
    # if db_path.exists():
    #     db_path.unlink()

    # Set a low budget for testing rejection
    env = CADEnv(
        workspace_dir=str(workspace),
        db_url="sqlite:///:memory:",
        max_unit_cost=100.0,  # High enough for simple parts
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
    try:
        env.close()
    except:
        pass

    if workspace.exists():
        shutil.rmtree(workspace)

    for suffix in ["", "-shm", "-wal"]:
        p = Path(f"test_history.db{suffix}")
        if p.exists():
            p.unlink()


import json


def test_env_budget_rejection(env):
    env.reset()
    env.max_unit_cost = 1.0  # Set low budget for this specific test
    # Write a part that definitely costs more than $1.0 (e.g. 10x10x10 cube)
    # Default 3D print cost is 0.05 per mm3 -> 1000 * 0.05 = $50.0
    env.step(
        {
            "tool": "write_file",
            "arguments": json.dumps(
                {
                    "content": "from build123d import Box\npart = Box(10, 10, 10)",
                    "path": "design.py",
                }
            ),
        }
    )

    obs, reward, terminated, _, _ = env.step(
        {"tool": "submit_design", "arguments": "{}"}
    )

    if reward != -20.0:
        print(f"DEBUG: Reward={reward}, Output={obs['last_output']}")

    assert reward == -20.0
    assert "REJECTED" in obs["last_output"]
    assert "exceeds budget" in obs["last_output"]
    assert not terminated


def test_env_force_submit(env):
    env.reset()
    env.step(
        {
            "tool": "write_file",
            "arguments": json.dumps(
                {
                    "content": "from build123d import Box\npart = Box(10, 10, 10)",
                    "path": "design.py",
                }
            ),
        }
    )

    # Force submit should bypass budget check
    action = {
        "tool": "submit_design",
        "arguments": json.dumps({"force_submit": True, "reason": "Testing bypass"}),
    }
    obs, reward, terminated, _, _ = env.step(action)

    # It should pass budget check and reach simulation (which might fail or pass)
    assert "REJECTED" not in obs["last_output"]
    # reward would be from simulation
    assert terminated


def test_env_reset(env):
    obs, _ = env.reset()
    assert "code" in obs
    assert obs["last_output"] == "Environment reset. Ready for new design."
    assert (Path(env.workspace_dir) / "design.py").exists()


def test_env_step_write(env):
    env.reset()
    action = {
        "tool": "write_file",
        "arguments": json.dumps(
            {
                "content": "from build123d import Box\npart = Box(10, 10, 10)",
                "path": "design.py",
            }
        ),
    }
    obs, _, _, _, _ = env.step(action)
    assert "Successfully wrote to design.py" in obs["last_output"]
    assert "Box(10, 10, 10)" in obs["code"]


def test_env_step_preview(env):
    env.reset()
    # Write a simple script first
    env.step(
        {
            "tool": "write_file",
            "arguments": json.dumps(
                {
                    "content": "from build123d import Box\npart = Box(10, 10, 10)",
                    "path": "design.py",
                }
            ),
        }
    )

    obs, _, _, _, _ = env.step({"tool": "preview_design", "arguments": "{}"})
    assert "Preview generated:" in obs["last_output"]
    assert obs["last_render"].endswith(".svg")
    assert (Path(env.workspace_dir) / obs["last_render"]).exists()


def test_env_step_submit_invalid(env):
    env.reset()
    # Write invalid script
    env.step(
        {
            "tool": "write_file",
            "arguments": json.dumps(
                {"content": "invalid python code", "path": "design.py"}
            ),
        }
    )

    obs, reward, terminated, _, _ = env.step(
        {"tool": "submit_design", "arguments": "{}"}
    )
    assert reward == -10.0
    assert "Error processing design" in obs["last_output"]
    assert not terminated


def test_env_step_submit_success(env):
    env.reset()
    # Write valid script
    env.step(
        {
            "tool": "write_file",
            "arguments": json.dumps(
                {
                    "content": "from build123d import Box\npart = Box(10, 10, 10)",
                    "path": "design.py",
                }
            ),
        }
    )

    # Mock simulation to ensure it passes without actual MuJoCo complexities if needed,
    # but we have a bridge that should handle it if MuJoCo is installed.
    # Let's see if it works without mocking first.

    obs, _, terminated, _, _ = env.step({"tool": "submit_design", "arguments": "{}"})
    # If MuJoCo works and template exists, it should be success or failure but terminated
    assert terminated
    assert "Submission Result:" in obs["last_output"]


def test_env_registration():
    # Registration is removed, but we can check if direct instantiation works
    env = CADEnv()
    assert isinstance(env, CADEnv)
