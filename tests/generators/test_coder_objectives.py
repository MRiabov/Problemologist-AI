import json
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


@pytest.mark.asyncio
async def test_coder_node_injects_objectives_yaml(mock_state):
    valid_script = "def build(seed, scale=1.0): return None, ''"
    objectives_content = "theme: bracket\nobjectives:\n  zone_goal:\n    min: [0,0,0]"

    mock_response = MagicMock()
    mock_response.content = f"```python\n{valid_script}\n```"

    with (
        patch("controller.agent.benchmark.nodes.ChatOpenAI"),
        patch(
            "controller.agent.benchmark.nodes.get_prompt",
            return_value="Base prompt {prompt}",
        ),
        patch(
            "controller.agent.benchmark.nodes.create_deep_agent"
        ) as mock_create_agent,
    ):
        mock_agent = mock_create_agent.return_value
        mock_agent.ainvoke = AsyncMock(return_value={"messages": [mock_response]})

        with patch(
            "controller.agent.benchmark.nodes.WorkerClient"
        ) as mock_client_class:
            mock_client = mock_client_class.return_value
            # Mock objectives.yaml existence
            mock_client.list_files = AsyncMock(
                return_value=[MagicMock(path="objectives.yaml")]
            )
            mock_client.read_file = AsyncMock(
                side_effect=[objectives_content, valid_script]
            )

            with patch("controller.agent.benchmark.nodes.RemoteFilesystemBackend"):
                await coder_node(mock_state)

        # Verify create_deep_agent was called with objectives_yaml in system prompt
        args, kwargs = mock_create_agent.call_args
        system_prompt = kwargs.get("system_prompt", "")
        assert "### Draft Objectives (YAML):" in system_prompt
        assert objectives_content in system_prompt
        assert "bracket" in system_prompt
