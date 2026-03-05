import pytest

from controller.agent.benchmark.tools import get_benchmark_planner_tools
from controller.agent.tools import get_engineer_planner_tools
from shared.enums import AgentName


class _FakeFs:
    def __init__(self, files: dict[str, str]):
        self._files = files

    async def exists(self, path: str) -> bool:
        return path in self._files

    async def read_file(self, path: str) -> str:
        return self._files[path]

    # The planner toolset includes other tools that we do not call in these tests.
    async def list_files(self, path: str = "/"):
        return []

    async def write_file(self, path: str, content: str, overwrite: bool = False):
        self._files[path] = content
        return "ok"

    async def edit_file(self, path: str, edits):
        return "ok"

    async def grep(
        self, pattern: str, path: str | None = None, glob: str | None = None
    ):
        return []

    async def run_command(self, command: str):
        return "ok"

    async def inspect_topology(self, target_id: str, script_path: str = "script.py"):
        return {}


def _get_tool_by_name(tools: list, name: str):
    for t in tools:
        if getattr(t, "__name__", "") == name:
            return t
    raise AssertionError(f"Tool {name} not found")


@pytest.mark.asyncio
async def test_engineer_submit_plan_rejects_missing_required_files(monkeypatch):
    fs = _FakeFs({"plan.md": "# Solution Overview\n"})
    tools = get_engineer_planner_tools(fs, session_id="s1")
    submit_plan = _get_tool_by_name(tools, "submit_plan")

    # Should fail before validator call due to missing required files.
    result = await submit_plan()
    assert result["ok"] is False
    assert result["status"] == "rejected"
    assert "Missing required file: todo.md" in result["errors"]
    assert "Missing required file: assembly_definition.yaml" in result["errors"]
    assert result["node_type"] == AgentName.ENGINEER_PLANNER.value


@pytest.mark.asyncio
async def test_benchmark_submit_plan_validates_and_submits(monkeypatch):
    fs = _FakeFs(
        {
            "plan.md": "# Learning Objective\n",
            "todo.md": "- [ ] task\n",
            "objectives.yaml": "version: '1.0'\nobjectives:\n  primary: []\n",
        }
    )
    tools = get_benchmark_planner_tools(fs, session_id="s1")
    submit_plan = _get_tool_by_name(tools, "submit_plan")

    called = {}

    def _fake_validate(node_type, files_content_map):
        called["node_type"] = node_type
        called["files"] = sorted(files_content_map.keys())
        return True, []

    monkeypatch.setattr(
        "worker_heavy.utils.file_validation.validate_node_output",
        _fake_validate,
    )

    result = await submit_plan()
    assert result["ok"] is True
    assert result["status"] == "submitted"
    assert result["errors"] == []
    assert result["node_type"] == AgentName.BENCHMARK_PLANNER.value
    assert called["node_type"] == AgentName.BENCHMARK_PLANNER
    assert called["files"] == ["objectives.yaml", "plan.md", "todo.md"]
