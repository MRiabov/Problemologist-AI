from unittest.mock import AsyncMock, MagicMock, patch

import pytest

from controller.agent.benchmark.models import GenerationSession
from controller.agent.benchmark.nodes import coder_node, planner_node
from controller.agent.benchmark.state import BenchmarkGeneratorState
from shared.simulation.schemas import RandomizationStrategy


@pytest.mark.asyncio
@patch("controller.agent.nodes.base.BaseNode._run_program")
async def test_planner_node_prompt_construction(mock_run_program):
    # Setup
    mock_session = MagicMock(spec=GenerationSession)
    mock_session.session_id = "00000000-0000-0000-0000-000000000000"
    mock_session.prompt = "A red ball bouncing"
    mock_session.status = "idle"
    mock_session.custom_objectives = {}

    state = BenchmarkGeneratorState(
        session=mock_session,
        current_script="",
        simulation_result=None,
        review_feedback=None,
        review_round=0,
        plan=None,
        messages=[],
    )

    # Patch SharedNodeContext.create
    with patch(
        "controller.agent.benchmark.nodes.SharedNodeContext.create"
    ) as mock_ctx_create:
        mock_ctx = MagicMock()
        mock_ctx_create.return_value = mock_ctx
        mock_ctx.worker_client = AsyncMock()
        mock_ctx.worker_client.exists.return_value = False
        mock_ctx.get_callbacks.return_value = []

        # Mock _run_program return: (prediction, artifacts, journal_entry)
        mock_run_program.return_value = (MagicMock(plan=RandomizationStrategy(theme="balls", reasoning="test")), {}, "")

        # Execute
        await planner_node(state)

        # Verify
        mock_run_program.assert_called_once()
        args, kwargs = mock_run_program.call_args
        inputs = args[3]
        assert "A red ball bouncing" in inputs["prompt"]


@pytest.mark.asyncio
async def test_coder_node_prompt_construction():
    # Setup
    mock_session = MagicMock(spec=GenerationSession)
    mock_session.session_id = "00000000-0000-0000-0000-000000000000"
    mock_session.prompt = "A blue cube"
    mock_session.status = "idle"
    mock_session.validation_logs = []
    mock_session.custom_objectives = {}

    state = BenchmarkGeneratorState(
        session=mock_session,
        current_script="",
        simulation_result=None,
        review_feedback=None,
        review_round=0,
        plan=RandomizationStrategy(theme="cubes", reasoning="test"),
        messages=[],
    )

    # Patch SharedNodeContext.create
    with (
        patch(
            "controller.agent.benchmark.nodes.SharedNodeContext.create"
        ) as mock_ctx_create,
    ):
        mock_ctx = MagicMock()
        mock_ctx_create.return_value = mock_ctx
        mock_ctx.worker_client = AsyncMock()
        mock_ctx.worker_client.exists.return_value = False
        mock_ctx.worker_client.validate = AsyncMock(return_value=MagicMock(success=True))
        mock_ctx.worker_client.simulate = AsyncMock(return_value=MagicMock(success=True, artifacts={}))
        mock_ctx.get_callbacks.return_value = []

        from controller.agent.mock_llm import MockDSPyLM
        mock_ctx.dspy_lm = MockDSPyLM(session_id="benchmark")
        mock_ctx.pm = MagicMock()
        mock_ctx.pm.render.return_value = "Rendered prompt"

        # Execute
        await coder_node(state)

        # Verify
        assert len(state.messages) > 0
        assert "Work summary:" in state.messages[-1].content
