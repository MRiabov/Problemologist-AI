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

    # Patch SharedNodeContext.create
    with patch(
        "controller.agent.benchmark.nodes.SharedNodeContext.create"
    ) as mock_ctx_create:
        mock_ctx = MagicMock()
        mock_ctx_create.return_value = mock_ctx
        mock_ctx.worker_client = AsyncMock()
        mock_ctx.get_callbacks.return_value = []

        mock_llm = MagicMock()
        mock_ctx.llm = mock_llm

        structured_mock = AsyncMock()
        structured_mock.ainvoke.return_value = RandomizationStrategy(
            theme="balls", reasoning="test"
        )
        mock_llm.with_structured_output.return_value = structured_mock

        # Execute
        await planner_node(state)

        # Verify
        mock_llm.with_structured_output.assert_called_with(RandomizationStrategy)
        structured_mock.ainvoke.assert_called_once()
        call_args = structured_mock.ainvoke.call_args
        messages = call_args.args[0]
        system_prompt = messages[0].content
        assert (
            "You are an expert designer of spatial and geometric puzzles"
            in system_prompt
        )


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

    # Patch SharedNodeContext.create and agent
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
        mock_ctx.worker_client.exists.return_value = False
        mock_ctx.get_callbacks.return_value = []

        mock_agent = AsyncMock()
        mock_agent.ainvoke.return_value = {"messages": [MagicMock(content="Done")]}
        mock_create_agent.return_value = mock_agent

        # Execute
        await coder_node(state)

        # Verify
        mock_agent.ainvoke.assert_called_once()
        call_args = mock_agent.ainvoke.call_args
        # Inside coder_node, it passes a dict to agent.ainvoke
        agent_input = call_args.args[0]
        messages = agent_input["messages"]
        system_prompt = messages[0].content

        assert "A blue cube" in system_prompt
        assert "cubes" in system_prompt
