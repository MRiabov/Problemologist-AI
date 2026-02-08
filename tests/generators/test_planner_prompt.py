import pytest
from unittest.mock import AsyncMock, MagicMock, patch
from controller.agent.benchmark.nodes import planner_node, coder_node
from controller.agent.benchmark.state import BenchmarkGeneratorState
from controller.agent.benchmark.models import GenerationSession


@pytest.mark.asyncio
async def test_planner_node_prompt_construction():
    # Setup
    mock_session = MagicMock(spec=GenerationSession)
    mock_session.session_id = "test-session"
    mock_session.prompt = "A red ball bouncing"

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
            "controller.agent.benchmark.nodes.create_deep_agent"
        ) as mock_create_agent,
        patch(
            "controller.agent.benchmark.nodes.RemoteFilesystemBackend"
        ) as mock_backend,
        patch("controller.agent.benchmark.nodes.ChatOpenAI") as mock_llm,
        patch("controller.agent.benchmark.nodes.WorkerClient") as mock_client_cls,
    ):
        # Mock client
        mock_client = AsyncMock()
        mock_client_cls.return_value = mock_client

        # Mock agent response
        mock_agent = AsyncMock()
        mock_agent.ainvoke.return_value = {
            "messages": [MagicMock(content='{"theme": "balls"}')]
        }
        mock_create_agent.return_value = mock_agent

        # Execute
        await planner_node(state)

        # Verify
        mock_create_agent.assert_called_once()
        call_args = mock_create_agent.call_args
        system_prompt = call_args.kwargs["system_prompt"]

        assert "A red ball bouncing" in system_prompt
        assert (
            "You are a mechanical engineering architect" not in system_prompt
        )  # Should utilize the new template
        assert (
            "You are an expert designer of spatial and geometric puzzles"
            in system_prompt
        )


@pytest.mark.asyncio
async def test_coder_node_prompt_construction():
    # Setup
    mock_session = MagicMock(spec=GenerationSession)
    mock_session.session_id = "test-session"
    mock_session.prompt = "A blue cube"
    mock_session.validation_logs = []

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
            "controller.agent.benchmark.nodes.create_deep_agent"
        ) as mock_create_agent,
        patch(
            "controller.agent.benchmark.nodes.RemoteFilesystemBackend"
        ) as mock_backend,
        patch("controller.agent.benchmark.nodes.ChatOpenAI") as mock_llm,
        patch("controller.agent.benchmark.nodes.WorkerClient") as mock_client_cls,
    ):
        # Mock client
        mock_client = AsyncMock()
        mock_client_cls.return_value = mock_client

        # Mock agent response
        mock_agent = AsyncMock()
        mock_agent.ainvoke.return_value = {"messages": [MagicMock(content="Done")]}
        mock_create_agent.return_value = mock_agent

        # Execute
        await coder_node(state)

        # Verify
        mock_create_agent.assert_called_once()
        call_args = mock_create_agent.call_args
        system_prompt = call_args.kwargs["system_prompt"]

        assert "A blue cube" in system_prompt
        assert "Original User Request" in system_prompt
        assert "cubes" in system_prompt
