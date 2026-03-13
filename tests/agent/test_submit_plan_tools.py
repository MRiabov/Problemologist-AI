import uuid

import pytest
from langchain_core.messages import AIMessage

from controller.agent.benchmark.tools import get_benchmark_planner_tools
from controller.agent.state import AgentState
from controller.agent.tools import (
    _invoke_cots_search_subagent,
    get_engineer_planner_tools,
)
from shared.enums import AgentName


class _FakeFs:
    def __init__(self, files: dict[str, str]):
        self._files = files
        self.client = type(
            "Client",
            (),
            {
                "session_id": "s1",
                "write_file": self._client_write_file,
            },
        )()
        self.policy = type(
            "Policy",
            (),
            {"get_allowed_tools": staticmethod(lambda _agent_role: None)},
        )()
        self.agent_role = AgentName.ENGINEER_PLANNER

    async def exists(self, path: str) -> bool:
        return path in self._files

    async def read_file(self, path: str) -> str:
        return self._files[path]

    async def _client_write_file(
        self,
        path: str,
        content: str,
        overwrite: bool = False,
        bypass_agent_permissions: bool = False,
    ):
        self._files[path] = content
        return True

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
            "benchmark_definition.yaml": "version: '1.0'\nobjectives:\n  primary: []\n",
            "assembly_definition.yaml": "version: '1.0'\nconstraints: {}\n",
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
    assert called["files"] == [
        "assembly_definition.yaml",
        "benchmark_definition.yaml",
        "plan.md",
        "todo.md",
    ]
    assert ".manifests/benchmark_plan_review_manifest.json" in fs._files


@pytest.mark.asyncio
async def test_engineer_tools_expose_cots_subagent_wrapper_not_raw_search(monkeypatch):
    fs = _FakeFs(
        {
            "assembly_definition.yaml": """
constraints:
  benchmark_max_unit_cost_usd: 100.0
  benchmark_max_weight_g: 1000.0
  planner_target_max_unit_cost_usd: 50.0
  planner_target_max_weight_g: 500.0
manufactured_parts:
  - part_name: bracket
    part_id: bracket
    manufacturing_method: CNC
    material_id: aluminum_6061
    quantity: 1
    part_volume_mm3: 1000.0
    stock_bbox_mm: {x: 10.0, y: 10.0, z: 10.0}
    stock_volume_mm3: 1000.0
    removed_volume_mm3: 0.0
    estimated_unit_cost_usd: 5.0
cots_parts: []
environment_drill_operations: []
final_assembly:
  - name: bracket
    config:
      dofs: []
totals:
  estimated_unit_cost_usd: 5.0
  estimated_weight_g: 25.0
  estimate_confidence: high
"""
        }
    )
    tools = get_engineer_planner_tools(fs, session_id="s1")
    tool_names = {getattr(tool, "__name__", "") for tool in tools}

    assert "invoke_cots_search_subagent" in tool_names
    assert "search_cots_catalog" not in tool_names

    captured = {}

    async def _fake_cots_search_node(state):
        captured["task"] = state.task
        return state.model_copy(
            update={"messages": [AIMessage(content="candidate result")]}
        )

    monkeypatch.setattr(
        "controller.agent.nodes.cots_search.cots_search_node",
        _fake_cots_search_node,
    )

    invoke_cots = _get_tool_by_name(tools, "invoke_cots_search_subagent")
    result = await invoke_cots(query="M3 bolt", category="fastener", limit=3)

    assert result == "candidate result"
    assert "M3 bolt" in captured["task"]
    assert "category=fastener" in captured["task"]


@pytest.mark.asyncio
async def test_benchmark_planner_cots_guard_uses_exact_phrase_matching(monkeypatch):
    fs = _FakeFs(
        {
            "benchmark_definition.yaml": """
benchmark_parts:
  - part_id: environment_fixture
    label: environment_fixture
moved_object:
  label: ball
  shape: sphere
"""
        }
    )
    tools = get_benchmark_planner_tools(fs, session_id="s1")
    invoke_cots = _get_tool_by_name(tools, "invoke_cots_search_subagent")

    captured: dict[str, str] = {}

    async def _fake_invoke(**kwargs):
        captured["query"] = kwargs["query"]
        return "candidate result"

    monkeypatch.setattr(
        "controller.agent.benchmark.tools._invoke_cots_search_subagent",
        _fake_invoke,
    )

    with pytest.raises(ValueError, match="Benchmark-owned fixtures"):
        await invoke_cots(
            query="price environment fixture bracket", category="fastener"
        )

    result = await invoke_cots(query="fixtured bracket", category="fastener")
    assert result == "candidate result"
    assert captured["query"] == "fixtured bracket"


@pytest.mark.asyncio
async def test_nested_cots_search_uses_uuid_episode_id(monkeypatch):
    captured: dict[str, str] = {}

    async def _fake_cots_search_node(state: AgentState):
        captured["session_id"] = state.session_id
        captured["episode_id"] = state.episode_id
        return state.model_copy(
            update={"messages": [AIMessage(content="candidate result")]}
        )

    monkeypatch.setattr(
        "controller.agent.nodes.cots_search.cots_search_node",
        _fake_cots_search_node,
    )

    result = await _invoke_cots_search_subagent(
        query="M3 bolt",
        category="fastener",
        session_id="INT-012-readable-session",
    )

    assert result == "candidate result"
    assert captured["session_id"] == "INT-012-readable-session"
    uuid.UUID(captured["episode_id"])
