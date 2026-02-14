from unittest.mock import AsyncMock, MagicMock, patch

import pytest

from controller.agent.benchmark.models import GenerationSession
from controller.agent.benchmark.nodes import coder_node, planner_node
from controller.agent.benchmark.state import BenchmarkGeneratorState


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

    # Patch dependencies
    with (
        patch(
            "controller.agent.benchmark.nodes.RemoteFilesystemMiddleware"
        ) as mock_backend,
        patch("controller.agent.benchmark.nodes.ChatOpenAI") as mock_llm_cls,
        patch("controller.agent.benchmark.nodes.WorkerClient") as mock_client_cls,
    ):
        # Mock client
        mock_client = AsyncMock()
        mock_client_cls.return_value = mock_client

        # Mock LLM
        mock_llm = mock_llm_cls.return_value
        mock_llm.ainvoke = AsyncMock()
        mock_llm.ainvoke.return_value = MagicMock(content='{"theme": "balls"}')

        # Execute
        await planner_node(state)

        # Verify
        mock_llm.ainvoke.assert_called_once()
        call_args = mock_llm.ainvoke.call_args
        messages = call_args.args[0]
        system_prompt = messages[0].content

        assert "A red ball bouncing" not in system_prompt  # Prompt is in HumanMessage
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
        plan={"theme": "cubes"},
        messages=[],
    )

    # Patch dependencies
    with (
        patch(
            "controller.agent.benchmark.nodes.RemoteFilesystemMiddleware"
        ) as mock_backend,
        patch("controller.agent.benchmark.nodes.ChatOpenAI") as mock_llm_cls,
        patch("controller.agent.benchmark.nodes.WorkerClient") as mock_client_cls,
    ):
        # Mock client
        mock_client = AsyncMock()
        mock_client_cls.return_value = mock_client

        # Mock LLM
        mock_llm = mock_llm_cls.return_value
        mock_llm.ainvoke = AsyncMock()
        mock_llm.ainvoke.return_value = MagicMock(content="Done", tool_calls=[])
        mock_llm.bind_tools = MagicMock(return_value=mock_llm)

        # Execute
        await coder_node(state)

        # Verify
        mock_llm.ainvoke.assert_called_once()
        call_args = mock_llm.ainvoke.call_args
        messages = call_args.args[0]
        # In coder_node, system prompt is messages[0].content
        system_prompt = messages[0].content

        assert "A blue cube" in system_prompt
        assert "Original User Request" in system_prompt
        assert "cubes" in system_prompt
