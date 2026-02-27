from unittest.mock import AsyncMock, MagicMock, patch
from uuid import uuid4

import pytest

from controller.agent.benchmark.models import GenerationSession, SessionStatus
from controller.agent.benchmark.nodes import coder_node
from controller.agent.benchmark.state import BenchmarkGeneratorState


@pytest.fixture
def mock_state():
    session = GenerationSession(session_id=uuid4(), prompt="Create a simple bracket")
    session.status = SessionStatus.PLANNING
    return BenchmarkGeneratorState(
        session=session,
        current_script="",
        simulation_result=None,
        review_feedback=None,
        plan={"theme": "bracket"},
        messages=[],
    )


@pytest.mark.asyncio(loop_scope="function")
async def test_coder_node_injects_objectives_yaml(mock_state):
    valid_script = "def build(seed, scale=1.0): return None, ''"
    objectives_content = "theme: bracket\nobjectives:\n  zone_goal:\n    min: [0,0,0]"

    # Patch dependencies
    with (
        patch(
            "controller.agent.benchmark.nodes.SharedNodeContext.create"
        ) as mock_ctx_create,
        patch(
            "controller.agent.nodes.base.BaseNode._run_program"
        ) as mock_run_program,
        patch("controller.agent.benchmark.nodes.get_benchmark_tools") as mock_get_tools,
    ):
        mock_ctx = MagicMock()
        mock_ctx_create.return_value = mock_ctx
        mock_ctx.worker_client = AsyncMock()
        mock_ctx.worker_client.exists.return_value = True
        mock_ctx.worker_client.read_file.side_effect = [
            objectives_content,
            valid_script,
            valid_script,
        ]

        mock_run_program.return_value = (
            MagicMock(journal="Done"),
            {"script.py": valid_script},
            "journal entry",
        )

        await coder_node(mock_state)

    # Verify agent was called with objectives_yaml in context
    mock_run_program.assert_called_once()
    call_args = mock_run_program.call_args
    inputs = call_args.args[3]
    assert objectives_content in inputs["objectives_yaml"]
    assert "bracket" in inputs["plan"]
