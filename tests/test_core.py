import shutil
from pathlib import Path

import gymnasium as gym
import pytest

from src.environment.core import CADEnv


@pytest.fixture
def env():
    # Setup a clean workspace for testing
    workspace = Path("test_workspace")
    if workspace.exists():
        shutil.rmtree(workspace)

    db_path = Path("test_history.db")
    if db_path.exists():
        db_path.unlink()

    env = CADEnv(workspace_dir=str(workspace), db_url=f"sqlite:///{db_path}")
    yield env

    # Cleanup
    if workspace.exists():
        shutil.rmtree(workspace)

    for suffix in ["", "-shm", "-wal"]:
        p = Path(f"test_history.db{suffix}")
        if p.exists():
            p.unlink()


def test_env_reset(env):
    obs, _ = env.reset()
    assert "code" in obs
    assert obs["last_output"] == "Environment reset. Ready for new design."
    assert (Path(env.workspace_dir) / "design.py").exists()


def test_env_step_write(env):
    env.reset()
    action = {
        "tool": 0,  # write_script
        "arguments": "from build123d import Box\npart = Box(10, 10, 10)",
    }
    obs, _, _, _, _ = env.step(action)
    assert "Successfully wrote to design.py" in obs["last_output"]
    assert "Box(10, 10, 10)" in obs["code"]


def test_env_step_preview(env):
    env.reset()
    # Write a simple script first
    env.step(
        {"tool": 0, "arguments": "from build123d import Box\npart = Box(10, 10, 10)"}
    )

    obs, _, _, _, _ = env.step({"tool": 2, "arguments": ""})
    assert "Preview generated:" in obs["last_output"]
    assert obs["last_render"].endswith(".svg")
    assert (Path(env.workspace_dir) / obs["last_render"]).exists()


def test_env_step_submit_invalid(env):
    env.reset()
    # Write invalid script
    env.step({"tool": 0, "arguments": "invalid python code"})

    obs, reward, terminated, _, _ = env.step({"tool": 4, "arguments": ""})
    assert reward == -10.0
    assert "Error processing design" in obs["last_output"]
    assert not terminated


def test_env_step_submit_success(env):
    env.reset()
    # Write valid script
    env.step(
        {"tool": 0, "arguments": "from build123d import Box\npart = Box(10, 10, 10)"}
    )

    # Mock simulation to ensure it passes without actual MuJoCo complexities if needed,
    # but we have a bridge that should handle it if MuJoCo is installed.
    # Let's see if it works without mocking first.

    obs, _, terminated, _, _ = env.step({"tool": 4, "arguments": ""})
    # If MuJoCo works and template exists, it should be success or failure but terminated
    assert terminated
    assert "Submission Result:" in obs["last_output"]


def test_env_registration():
    env = gym.make("CADEnv-v0")
    assert isinstance(env.unwrapped, CADEnv)
