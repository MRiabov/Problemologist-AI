from unittest.mock import AsyncMock, MagicMock, patch
from uuid import uuid4

import pytest

from controller.agent.benchmark.models import GenerationSession
from controller.agent.benchmark.nodes import coder_node
from controller.agent.benchmark.state import BenchmarkGeneratorState
from shared.simulation.schemas import RandomizationStrategy


@pytest.fixture
def mock_state():
    session = GenerationSession(session_id=uuid4(), prompt="Create a simple bracket")
    session.status = "idle"
    session.validation_logs = []
    return BenchmarkGeneratorState(
        session=session,
        current_script="",
        simulation_result=None,
        review_feedback=None,
        plan=RandomizationStrategy(theme="bracket", reasoning="test"),
        messages=[],
    )


@pytest.mark.asyncio
async def test_coder_node_injects_objectives_yaml(mock_state):
    valid_script = "def build(seed, scale=1.0): return None, ''"
    objectives_content = "theme: bracket\nobjectives:\n  zone_goal:\n    min: [0,0,0]"

    # Patch dependencies
    with (
        patch(
            "controller.agent.benchmark.nodes.SharedNodeContext.create"
        ) as mock_ctx_create,
        patch("controller.agent.benchmark.nodes.get_benchmark_tools") as mock_get_tools,
    ):
        mock_ctx = mock_ctx_create.return_value
        mock_ctx.worker_client = AsyncMock()
        mock_ctx.worker_client.exists.return_value = True
        mock_ctx.worker_client.read_file.side_effect = [
            objectives_content,
            valid_script,
        ]
        mock_ctx.worker_client.validate = AsyncMock(return_value=MagicMock(success=True))
        mock_ctx.worker_client.simulate = AsyncMock(return_value=MagicMock(success=True, artifacts={}))
        mock_ctx.get_callbacks.return_value = []

        from controller.agent.mock_llm import MockDSPyLM
        mock_ctx.dspy_lm = MockDSPyLM(session_id="benchmark")
        mock_ctx.pm = MagicMock()
        mock_ctx.pm.render.return_value = "Rendered prompt"

        await coder_node(mock_state)

    # Verify state updated
    assert len(mock_state.messages) > 0
    assert "Work summary:" in mock_state.messages[-1].content
