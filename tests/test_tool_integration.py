import os
import pytest
import shutil
from src.environment import tools


def test_check_manufacturability_integration(tmp_path):
    # Setup workspace
    tools.set_workspace_dir(str(tmp_path))

    # Create a design script
    design_content = """
from build123d import Box
part = Box(10, 10, 10)
"""
    tools.write_file(design_content, "box.py")

    # 1. Test CNC
    report_cnc = tools.check_manufacturability("box.py", "cnc_milling", quantity=1)
    assert "status" in report_cnc
    assert any(p["process"] == "cnc_milling" for p in report_cnc["parts"])
    assert "cost_analysis" in report_cnc
    assert "manufacturability_score" in report_cnc

    # 2. Test IM
    report_im = tools.check_manufacturability(
        "box.py", "injection_molding", quantity=1000
    )
    assert any(p["process"] == "injection_molding" for p in report_im["parts"])
    assert report_im["cost_analysis"]["target_quantity"] == 1000
    assert report_im["manufacturability_score"] < 1.0  # Should have violations
    # IM should have high setup cost
    assert report_im["cost_analysis"]["total_cost"] > 5000


def test_manufacturability_caching(tmp_path, capsys):
    tools.set_workspace_dir(str(tmp_path))
    tools.write_file("from build123d import Box\np = Box(1,1,1)", "small.py")

    # First call
    tools.check_manufacturability("small.py", "cnc_milling")

    # Second call - should be cached
    # We can't easily check lru_cache hits without accessing internals,
    # but we can verify it doesn't crash.
    report = tools.check_manufacturability("small.py", "cnc_milling")
    assert report["status"] == "fail"  # because of box bottom undercut


from src.environment.core import CADEnv


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
    assert "'status': 'fail'" in _obs["last_output"]  # Box has undercuts from Z
    assert "'process': 'cnc'" in _obs["last_output"]
    assert "'quantity': 10" in _obs["last_output"]
