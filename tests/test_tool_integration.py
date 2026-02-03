from src.environment.core import CADEnv
from src.environment.runtime import ToolRuntime


def test_agent_exposure(tmp_path):
    # Setup environment with a temporary workspace
    db_path = tmp_path / "test_history.db"
    workspace_dir = tmp_path / "workspace"
    env = CADEnv(db_url=f"sqlite:///{db_path}", workspace_dir=str(workspace_dir))

    # 1. Reset env
    env.reset()

    # 2. Write a script
    design_content = "from build123d import Box\np = Box(10, 10, 10)"
    env.step(
        {
            "tool": "write_file",
            "arguments": {"content": design_content, "path": "design.py"},
        }
    )

    # 3. Call manufacturability check via agent interface
    # tool 5 is check_manufacturability
    _obs, _reward, _terminated, _truncated, _info = env.step(
        {
            "tool": "check_manufacturability",
            "arguments": {"process": "cnc", "quantity": 10},
        }
    )

    # 4. Assert tool output is present in observations
    assert "last_output" in _obs
    assert '"status": "fail"' in _obs["last_output"]
    assert '"process": "cnc"' in _obs["last_output"]
    assert '"quantity": 10' in _obs["last_output"]
