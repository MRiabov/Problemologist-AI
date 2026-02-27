from unittest.mock import AsyncMock, MagicMock, patch

import pytest

from controller.agent.benchmark.models import GenerationSession, SessionStatus
from controller.agent.benchmark.nodes import coder_node, planner_node
from controller.agent.benchmark.state import BenchmarkGeneratorState
from shared.simulation.schemas import RandomizationStrategy


@pytest.mark.asyncio(loop_scope="function")
async def test_planner_node_prompt_construction():
    # Setup
    mock_session = MagicMock(spec=GenerationSession)
    mock_session.session_id = "00000000-0000-0000-0000-000000000000"
    mock_session.prompt = "A red ball bouncing"
    mock_session.custom_objectives = {}
    mock_session.status = SessionStatus.PLANNING

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
    ) as mock_ctx_create, patch(
        "controller.agent.nodes.base.BaseNode._run_program"
    ) as mock_run_program:
        mock_ctx = MagicMock()
        mock_ctx_create.return_value = mock_ctx
        mock_ctx.worker_client = AsyncMock()

        mock_run_program.return_value = (
            MagicMock(plan=RandomizationStrategy(theme="balls", reasoning="test")),
            {},
            "journal entry",
        )

        # Execute
        await planner_node(state)

        # Verify
        mock_run_program.assert_called_once()
        call_args = mock_run_program.call_args
        inputs = call_args.args[3]
        assert "A red ball bouncing" in inputs["prompt"]


@pytest.mark.asyncio(loop_scope="function")
async def test_coder_node_prompt_construction():
    # Setup
    mock_session = MagicMock(spec=GenerationSession)
    mock_session.session_id = "00000000-0000-0000-0000-000000000000"
    mock_session.prompt = "A blue cube"
    mock_session.validation_logs = []
    mock_session.custom_objectives = {}
    mock_session.status = SessionStatus.PLANNING

    state = BenchmarkGeneratorState(
        session=mock_session,
        current_script="",
        simulation_result=None,
        review_feedback=None,
        review_round=0,
        plan=RandomizationStrategy(theme="cubes", reasoning="test"),
        messages=[],
    )

    # Patch SharedNodeContext.create and agent
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
        mock_ctx.worker_client.exists.return_value = False
        mock_ctx.worker_client.read_file.return_value = "script content"

        mock_run_program.return_value = (
            MagicMock(journal="Done"),
            {"script.py": "script content"},
            "journal entry",
        )

        # Execute
        await coder_node(state)

        # Verify
        mock_run_program.assert_called_once()
        call_args = mock_run_program.call_args
        inputs = call_args.args[3]

        assert "A blue cube" in inputs["prompt"]
        assert "cubes" in inputs["plan"]
