from unittest.mock import AsyncMock, MagicMock, patch

import pytest

from controller.agent.benchmark.models import GenerationSession
from controller.agent.benchmark.nodes import coder_node, planner_node
from controller.agent.benchmark.state import BenchmarkGeneratorState
from shared.simulation.schemas import RandomizationStrategy


@pytest.mark.asyncio
async def test_planner_node_prompt_construction():
    # Setup
    mock_session = MagicMock(spec=GenerationSession)
    mock_session.session_id = "00000000-0000-0000-0000-000000000000"
    mock_session.prompt = "A red ball bouncing"
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

    # Patch SharedNodeContext.lifecycle and dspy.CodeAct
    with (
        patch(
            "controller.agent.benchmark.nodes.SharedNodeContext.lifecycle"
        ) as mock_lifecycle,
        patch("controller.agent.benchmark.nodes.dspy.CodeAct") as mock_codeact,
    ):
        mock_ctx = MagicMock()
        mock_lifecycle.return_value.__aenter__.return_value = mock_ctx
        mock_ctx.worker_client = AsyncMock()
        mock_ctx.get_callbacks.return_value = []

        mock_program = MagicMock()
        mock_program.return_value = MagicMock(
            plan=RandomizationStrategy(theme="balls", reasoning="test")
        )
        mock_codeact.return_value = mock_program

        # Execute
        await planner_node(state)

        # Verify
        mock_program.assert_called_once()
        kwargs = mock_program.call_args.kwargs
        assert kwargs["prompt"] == "A red ball bouncing"


@pytest.mark.asyncio
async def test_coder_node_prompt_construction():
    # Setup
    mock_session = MagicMock(spec=GenerationSession)
    mock_session.session_id = "00000000-0000-0000-0000-000000000000"
    mock_session.prompt = "A blue cube"
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

    # Patch SharedNodeContext.lifecycle and dspy.CodeAct
    with (
        patch(
            "controller.agent.benchmark.nodes.SharedNodeContext.lifecycle"
        ) as mock_lifecycle,
        patch("controller.agent.benchmark.nodes.dspy.CodeAct") as mock_codeact,
        patch("controller.agent.benchmark.nodes.get_benchmark_tools") as mock_get_tools,
    ):
        mock_ctx = MagicMock()
        mock_lifecycle.return_value.__aenter__.return_value = mock_ctx
        mock_ctx.worker_client = AsyncMock()
        mock_ctx.worker_client.exists.return_value = False
        mock_ctx.get_callbacks.return_value = []

        mock_program = MagicMock()
        mock_program.return_value = MagicMock(journal="Done")
        mock_codeact.return_value = mock_program

        # Execute
        await coder_node(state)

        # Verify
        mock_program.assert_called_once()
        kwargs = mock_program.call_args.kwargs
        assert "A blue cube" in kwargs["prompt"]
        assert "cubes" in kwargs["plan"]
