import os
import shutil
import pytest
import gymnasium as gym
from src.environment.core import CADEnv


@pytest.fixture
def env():
    # Setup a clean workspace for testing
    workspace = "test_workspace"
    if os.path.exists(workspace):
        shutil.rmtree(workspace)

    db_url = "sqlite:///test_history.db"
    if os.path.exists("test_history.db"):
        os.remove("test_history.db")

    env = CADEnv(workspace_dir=workspace, db_url=db_url)
    yield env

    # Cleanup
    if os.path.exists(workspace):
        shutil.rmtree(workspace)
    if os.path.exists("test_history.db"):
        os.remove("test_history.db")
    if os.path.exists("test_history.db-shm"):
        os.remove("test_history.db-shm")
    if os.path.exists("test_history.db-wal"):
        os.remove("test_history.db-wal")


def test_env_reset(env):
    obs, info = env.reset()
    assert "code" in obs
    assert obs["last_output"] == "Environment reset. Ready for new design."
    assert os.path.exists(os.path.join(env.workspace_dir, "design.py"))


def test_env_step_write(env):
    env.reset()
    action = {
        "tool": 0,  # write_script
        "arguments": "from build123d import Box\npart = Box(10, 10, 10)",
    }
    obs, reward, terminated, truncated, info = env.step(action)
    assert "Successfully wrote to design.py" in obs["last_output"]
    assert "Box(10, 10, 10)" in obs["code"]


def test_env_step_preview(env):
    env.reset()
    # Write a simple script first
    env.step(
        {"tool": 0, "arguments": "from build123d import Box\npart = Box(10, 10, 10)"}
    )

    obs, reward, terminated, truncated, info = env.step({"tool": 2, "arguments": ""})
    assert "Preview generated:" in obs["last_output"]
    assert obs["last_render"].endswith(".svg")
    assert os.path.exists(os.path.join(env.workspace_dir, obs["last_render"]))


def test_env_step_submit_invalid(env):
    env.reset()
    # Write invalid script
    env.step({"tool": 0, "arguments": "invalid python code"})

    obs, reward, terminated, truncated, info = env.step({"tool": 4, "arguments": ""})
    assert reward == -10.0
    assert "Error executing script" in obs["last_output"]
    assert not terminated


def test_env_step_submit_success(env, monkeypatch):
    env.reset()
    # Write valid script
    env.step(
        {"tool": 0, "arguments": "from build123d import Box\npart = Box(10, 10, 10)"}
    )

    # Mock simulation to ensure it passes without actual MuJoCo complexities if needed,
    # but we have a bridge that should handle it if MuJoCo is installed.
    # Let's see if it works without mocking first.

    obs, reward, terminated, truncated, info = env.step({"tool": 4, "arguments": ""})
    # If MuJoCo works and template exists, it should be success or failure but terminated
    assert terminated
    assert "Submission Result:" in obs["last_output"]


def test_env_registration():
    env = gym.make("CADEnv-v0")
    assert isinstance(env.unwrapped, CADEnv)
