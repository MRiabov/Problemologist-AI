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

    # Patch dependencies
    with (
        patch(
            "controller.agent.benchmark.nodes.SharedNodeContext.lifecycle"
        ) as mock_lifecycle,
        patch(
            "controller.agent.benchmark.nodes.dspy.CodeAct"
        ) as mock_codeact,
        patch("controller.agent.benchmark.nodes.get_benchmark_tools") as mock_get_tools,
    ):
        mock_ctx = MagicMock()
        mock_lifecycle.return_value.__aenter__.return_value = mock_ctx
        mock_ctx.worker_client = AsyncMock()
        mock_ctx.worker_client.exists.return_value = True
        mock_ctx.worker_client.read_file.side_effect = [
            objectives_content,
            valid_script,
        ]
        mock_ctx.get_callbacks.return_value = []

        mock_program = MagicMock()
        mock_program.return_value = MagicMock(journal="Done")
        mock_codeact.return_value = mock_program

        await coder_node(mock_state)

    # Verify agent was called with objectives_yaml in context
    mock_program.assert_called_once()
    kwargs = mock_program.call_args.kwargs
    assert kwargs["objectives_yaml"] == objectives_content
    assert "bracket" in kwargs["plan"]
