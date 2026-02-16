from unittest.mock import AsyncMock, MagicMock, patch
from uuid import uuid4

import pytest

from controller.agent.benchmark.models import GenerationSession
from controller.agent.benchmark.nodes import coder_node
from controller.agent.benchmark.state import BenchmarkGeneratorState


@pytest.fixture
def mock_state():
    session = GenerationSession(session_id=uuid4(), prompt="Create a simple bracket")
    return BenchmarkGeneratorState(
        session=session,
        current_script="",
        simulation_result=None,
        review_feedback=None,
        plan={"theme": "bracket"},
        messages=[],
    )


@patch("controller.agent.benchmark.nodes.ChatOpenAI")
@pytest.mark.asyncio
async def test_coder_node_injects_objectives_yaml(mock_llm_cls, mock_state):
    valid_script = "def build(seed, scale=1.0): return None, ''"
    objectives_content = "theme: bracket\nobjectives:\n  zone_goal:\n    min: [0,0,0]"

    # Patch dependencies
    with (
        patch(
            "controller.agent.benchmark.nodes.SharedNodeContext.create"
        ) as mock_ctx_create,
        patch(
            "controller.agent.benchmark.nodes.create_react_agent"
        ) as mock_create_agent,
        patch("controller.agent.benchmark.nodes.get_benchmark_tools") as mock_get_tools,
    ):
        mock_ctx = MagicMock()
        mock_ctx_create.return_value = mock_ctx
        mock_ctx.worker_client = AsyncMock()
        mock_ctx.worker_client.exists.return_value = True
        mock_ctx.worker_client.read_file.side_effect = [
            objectives_content,
            valid_script,
        ]
        mock_ctx.get_callbacks.return_value = []

        mock_agent = AsyncMock()
        mock_agent.ainvoke.return_value = {"messages": [MagicMock(content="Done")]}
        mock_create_agent.return_value = mock_agent

        await coder_node(mock_state)

    # Verify agent was called with objectives_yaml in context
    mock_agent.ainvoke.assert_called_once()
    call_args = mock_agent.ainvoke.call_args
    messages = call_args.args[0]["messages"]
    system_prompt = messages[0].content
    assert "### Draft Objectives (YAML):" in system_prompt
    assert objectives_content in system_prompt
    assert "bracket" in system_prompt
