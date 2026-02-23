from unittest.mock import AsyncMock, MagicMock, patch
from uuid import uuid4

import pytest

from controller.agent.benchmark.graph import define_graph, run_generation_session
from controller.agent.benchmark.models import SessionStatus


@pytest.mark.asyncio
async def test_define_graph_compiles():
    graph = define_graph()
    assert graph is not None


@pytest.mark.asyncio
async def test_run_generation_session_mocked():
    prompt = "A simple gear pair"

    from shared.simulation.schemas import RandomizationStrategy

    mock_plan = RandomizationStrategy(theme="gears", reasoning="test")
    mock_script = "print('hello')"

    with (
        patch("controller.agent.benchmark.graph.define_graph") as mock_define,
        patch(
            "controller.agent.benchmark.graph.get_sessionmaker"
        ) as mock_get_sessionmaker,
    ):
        # Configure mock DB context manager
        mock_db_session = AsyncMock()
        mock_db_session.add = MagicMock() # Sync
        mock_get_sessionmaker.return_value.return_value.__aenter__.return_value = (
            mock_db_session
        )

        # Mock Episode for DB updates inside _execute_graph_streaming
        mock_episode = MagicMock()
        mock_episode.metadata_vars = {}
        mock_result = MagicMock()
        mock_result.scalar_one_or_none.return_value = mock_episode
        mock_db_session.execute.return_value = mock_result
        mock_db_session.get.return_value = mock_episode
        # Mock the compiled graph's astream method
        mock_app = MagicMock()

        async def mock_astream_gen(input_state, **kwargs):
            # Yield a state chunk
            yield {
                "planner": {
                    "session": {
                        "session_id": uuid4(),
                        "prompt": prompt,
                        "status": SessionStatus.ACCEPTED,
                        "validation_logs": [],
                    },
                    "plan": mock_plan,
                    "current_script": mock_script,
                    "simulation_result": {"valid": True},
                    "review_feedback": "Approved",
                }
            }

        mock_app.astream = mock_astream_gen
        mock_define.return_value = mock_app

        final_state = await run_generation_session(prompt)

        assert final_state.session.status == SessionStatus.PLANNED
        assert final_state.plan.theme == "gears"
        assert final_state.current_script == "print('hello')"


@pytest.mark.asyncio
async def test_run_generation_session_rejected():
    prompt = "A complex linkage"

    with (
        patch("controller.agent.benchmark.graph.define_graph") as mock_define,
        patch(
            "controller.agent.benchmark.graph.get_sessionmaker"
        ) as mock_get_sessionmaker,
    ):
        # Configure mock DB context manager
        mock_db_session = AsyncMock()
        mock_get_sessionmaker.return_value.return_value.__aenter__.return_value = (
            mock_db_session
        )

        # Mock Episode for DB updates inside _execute_graph_streaming
        mock_episode = MagicMock()
        mock_episode.metadata_vars = {}
        mock_result = MagicMock()
        mock_result.scalar_one_or_none.return_value = mock_episode
        mock_db_session.execute.return_value = mock_result
        mock_db_session.get.return_value = mock_episode
        mock_app = MagicMock()

        async def mock_astream_gen(input_state, **kwargs):
            yield {
                "reviewer": {
                    "session": {
                        "session_id": uuid4(),
                        "prompt": prompt,
                        "status": SessionStatus.PLANNING,
                        "validation_logs": [],
                    },
                    "plan": {"theme": "complex"},
                    "current_script": "",
                    "simulation_result": None,
                    "review_feedback": "Rejected: Too complex",
                }
            }

        mock_app.astream = mock_astream_gen
        mock_define.return_value = mock_app

        final_state = await run_generation_session(prompt)

        # Verify status update logic in run_generation_session
        assert final_state.session.status == SessionStatus.REJECTED
