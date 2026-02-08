from unittest.mock import AsyncMock, MagicMock, patch

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

    # Mock return values that match BenchmarkGeneratorState structure
    mock_plan = {"theme": "gears"}
    mock_script = "print('hello')"

    with patch("controller.agent.benchmark.graph.define_graph") as mock_define:
        # Mock the compiled graph's astream method
        mock_app = MagicMock()

        async def mock_astream_gen(input_state, **kwargs):
            # Yield a state chunk
            yield {
                "planner": {
                    "session": MagicMock(status=SessionStatus.accepted),
                    "plan": mock_plan,
                    "current_script": mock_script,
                    "simulation_result": {"valid": True},
                    "review_feedback": "Approved",
                }
            }

        mock_app.astream = mock_astream_gen
        mock_define.return_value = mock_app

        final_state = await run_generation_session(prompt)

        assert final_state["session"].status == SessionStatus.accepted
        assert final_state["plan"]["theme"] == "gears"
        assert final_state["current_script"] == "print('hello')"


@pytest.mark.asyncio
async def test_run_generation_session_rejected():
    prompt = "A complex linkage"

    with patch("controller.agent.benchmark.graph.define_graph") as mock_define:
        mock_app = MagicMock()

        async def mock_astream_gen(input_state, **kwargs):
            yield {
                "reviewer": {
                    "session": MagicMock(status=SessionStatus.planning),
                    "plan": {},
                    "current_script": "",
                    "simulation_result": None,
                    "review_feedback": "Rejected: Too complex",
                }
            }

        mock_app.astream = mock_astream_gen
        mock_define.return_value = mock_app

        final_state = await run_generation_session(prompt)

        # Verify status update logic in run_generation_session
        assert final_state["session"].status == SessionStatus.rejected
