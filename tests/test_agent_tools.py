import pytest
import os
import shutil
from src.agent.tools.planning import update_plan
from src.agent.utils.workspace import Workspace

@pytest.fixture
def clean_workspace():
    workspace_path = "workspace"
    if os.path.exists(workspace_path):
        shutil.rmtree(workspace_path)

    ws = Workspace(workspace_path)
    yield ws

    if os.path.exists(workspace_path):
        shutil.rmtree(workspace_path)

def test_update_plan(clean_workspace):
    result = update_plan.invoke({"status": "In Progress", "notes": "Step 1 done"})
    assert "Plan updated" in result

    content = clean_workspace.read("plan.md")
    assert "Status: In Progress" in content
    assert "Step 1 done" in content
