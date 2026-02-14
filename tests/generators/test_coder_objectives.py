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

    mock_response = MagicMock()
    mock_response.content = f"```python\n{valid_script}\n```"
    mock_response.tool_calls = []

    # Mock LLM
    mock_llm = mock_llm_cls.return_value
    mock_llm.ainvoke = AsyncMock(return_value=mock_response)
    mock_llm.bind_tools = MagicMock(return_value=mock_llm)

    with (
        patch("controller.agent.benchmark.nodes.WorkerClient") as mock_client_class,
        patch("controller.agent.benchmark.nodes.RemoteFilesystemMiddleware"),
    ):
        mock_client = mock_client_class.return_value
        # Mock objectives.yaml existence
        mock_client.list_files = AsyncMock(
            return_value=[MagicMock(path="objectives.yaml")]
        )
        mock_client.read_file = AsyncMock(
            side_effect=[objectives_content, valid_script]
        )

        await coder_node(mock_state)

    # Verify LLM was called with objectives_yaml in context
    call_args = mock_llm.ainvoke.call_args
    messages = call_args.args[0]
    system_prompt = messages[0].content
    assert "### Draft Objectives (YAML):" in system_prompt
    assert objectives_content in system_prompt
    assert "bracket" in system_prompt
