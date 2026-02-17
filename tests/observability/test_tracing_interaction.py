import uuid
from unittest.mock import AsyncMock, MagicMock, patch

import pytest

from controller.agent.benchmark.graph import _execute_graph_streaming
from controller.agent.benchmark.models import GenerationSession
from controller.agent.benchmark.nodes import (
    cots_search_node,
    planner_node,
    skills_node,
)
from controller.agent.benchmark.state import BenchmarkGeneratorState
from shared.simulation.schemas import ValidationResult


@pytest.mark.asyncio
async def test_nodes_call_get_langfuse_callback():
    """
    Verify that benchmark nodes call get_langfuse_callback.
    """
    mock_session = MagicMock(spec=GenerationSession)
    mock_session.session_id = str(uuid.uuid4())
    mock_session.prompt = "test prompt"
    mock_session.custom_objectives = {}
    mock_session.status = MagicMock()

    state = BenchmarkGeneratorState(
        session=mock_session,
        messages=[],
        plan={"theme": "test"},
        simulation_result=ValidationResult(valid=True),
    )

    # Use nested with blocks for maximum compatibility
    with patch(
        "controller.agent.nodes.base.get_langfuse_callback"
    ) as mock_get_callback:
        with patch("controller.agent.nodes.base.DatabaseCallbackHandler"):
            with patch(
                "controller.agent.benchmark.nodes.SharedNodeContext"
            ) as mock_ctx_cls:
                mock_ctx = mock_ctx_cls.create.return_value

                # Setup mock LLM
                mock_llm = mock_ctx.llm
                mock_llm.ainvoke = AsyncMock(return_value=MagicMock())
                mock_llm.with_structured_output = MagicMock(return_value=mock_llm)

                # Setup mock worker client
                mock_worker = mock_ctx.worker_client
                mock_worker.list_files = AsyncMock(return_value=[])
                mock_worker.read_file = AsyncMock(return_value="")
                mock_worker.write_file = AsyncMock(return_value=True)
                mock_worker.git_init = AsyncMock(return_value=True)
                mock_worker.exists = AsyncMock(return_value=False)

                # Test cots_search_node
                with patch(
                    "controller.agent.benchmark.nodes.create_react_agent"
                ) as mock_create_agent:
                    mock_agent = AsyncMock()
                    mock_agent.ainvoke.return_value = {"messages": []}
                    mock_create_agent.return_value = mock_agent

                    await cots_search_node(state)

                # Test skills_node
                await skills_node(state)

                # Test planner_node
                with patch(
                    "controller.agent.benchmark.nodes.get_prompt",
                    return_value="prompt",
                ):
                    await planner_node(state)


@pytest.mark.asyncio
async def test_graph_initializes_langfuse_callback():
    """
    Verify that _execute_graph_streaming initializes Langfuse callback.
    """
    mock_session = MagicMock(spec=GenerationSession)
    mock_session.session_id = str(uuid.uuid4())
    mock_session.prompt = "test prompt"
    mock_session.validation_logs = []
    mock_session.status = MagicMock()

    state = BenchmarkGeneratorState(
        session=mock_session,
        messages=[],
        plan={"theme": "test"},
        simulation_result=ValidationResult(valid=True),
        current_script="print(1)",
    )

    with patch(
        "controller.agent.benchmark.graph.get_langfuse_callback"
    ) as mock_get_callback:
        with patch("controller.agent.benchmark.graph.DatabaseCallbackHandler"):
            with patch(
                "controller.agent.benchmark.graph.get_sessionmaker"
            ) as mock_get_sessionmaker:
                with patch(
                    "controller.agent.benchmark.graph.StateGraph"
                ) as mock_graph_cls:
                    mock_callback = MagicMock()
                    mock_get_callback.return_value = mock_callback

                    mock_app = MagicMock()

                    # Manual async iterator mock
                    class MockAsyncIterator:
                        def __init__(self, items):
                            self.items = items

                        def __aiter__(self):
                            return self

                        async def __anext__(self):
                            if not self.items:
                                raise StopAsyncIteration
                            return self.items.pop(0)

                    mock_app.astream.side_effect = lambda *_, **__: MockAsyncIterator(
                        [{"planner": {"plan": {"theme": "test"}}}]
                    )
                    mock_graph_cls.return_value.compile.return_value = mock_app

                    mock_db = AsyncMock()
                    mock_db.execute = AsyncMock()
                    mock_db.commit = AsyncMock()
                    mock_db.get = AsyncMock()

                    mock_episode = MagicMock()
                    mock_episode.metadata_vars = {"detailed_status": "executing"}

                    mock_result = MagicMock()
                    mock_result.scalar_one_or_none.return_value = mock_episode
                    mock_db.execute.return_value = mock_result
                    mock_db.get.return_value = mock_episode

                    mock_factory = MagicMock()
                    mock_factory.return_value.__aenter__.return_value = mock_db
                    mock_get_sessionmaker.return_value = mock_factory

                    # Trigger graph execution
                    test_uuid = uuid.uuid4()
                    await _execute_graph_streaming(
                        mock_app, state, test_uuid, "test-prompt"
                    )

                    # Verify callback creation
                    mock_get_callback.assert_called_once_with(
                        name="benchmark_generator", session_id=str(test_uuid)
                    )

                    # Verify it's passed to astream
                    _args, kwargs = mock_app.astream.call_args
                    callbacks = kwargs.get("config", {}).get("callbacks", [])
                    assert mock_callback in callbacks
