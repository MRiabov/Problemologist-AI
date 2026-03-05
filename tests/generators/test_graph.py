from unittest.mock import AsyncMock, MagicMock, patch
from uuid import uuid4

import pytest

from controller.agent.benchmark.graph import (
    _validate_planner_handoff,
    _validate_reviewer_handoff,
    define_graph,
    run_generation_session,
)
from controller.agent.benchmark.models import SessionStatus
from shared.enums import ReviewDecision
from shared.simulation.schemas import RandomizationStrategy


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
        patch(
            "controller.agent.benchmark.graph._validate_planner_handoff",
            new_callable=AsyncMock,
            return_value=[],
        ),
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
                    "review_decision": ReviewDecision.REJECTED,
                }
            }

        mock_app.astream = mock_astream_gen
        mock_define.return_value = mock_app

        final_state = await run_generation_session(prompt)

        # Verify status update logic in run_generation_session
        assert final_state.session.status == SessionStatus.REJECTED


@pytest.mark.asyncio
@patch("controller.agent.benchmark.graph._read_session_markdown")
@patch("controller.agent.benchmark.graph._get_latest_planner_submission_result")
async def test_validate_planner_handoff_fails_when_submit_plan_missing(
    mock_submission_result, mock_read_markdown
):
    template_files = {
        "plan.md": "# 1. Solution Overview\n\n- test plan",
        "todo.md": "- [ ] test step",
        "objectives.yaml": (
            "objectives:\n"
            "  goal_zone:\n"
            "    min: [8.0, -2.0, 0.0]\n"
            "    max: [12.0, 2.0, 4.0]\n"
            "  forbid_zones: []\n"
            "  build_zone:\n"
            "    min: [-12.0, -12.0, -2.0]\n"
            "    max: [12.0, 12.0, 12.0]\n"
            "  fluid_objectives: []\n"
            "  stress_objectives: []\n"
            "simulation_bounds:\n"
            "  min: [-30.0, -30.0, -10.0]\n"
            "  max: [30.0, 30.0, 30.0]\n"
            "moved_object:\n"
            "  label: projectile_ball\n"
            "  shape: sphere\n"
            "  static_randomization:\n"
            "    radius: [1.0, 2.0]\n"
            "  start_position: [0.0, 0.0, 0.0]\n"
            "  runtime_jitter: [0.5, 0.5, 0.5]\n"
            "constraints:\n"
            "  max_unit_cost: 50.0\n"
            "  max_weight_g: 1200.0\n"
            "randomization:\n"
            "  static_variation_id: template-v1\n"
            "  runtime_jitter_enabled: true\n"
        ),
    }

    async def _read_side_effect(_session_id, path):
        return template_files.get(path)

    mock_read_markdown.side_effect = _read_side_effect
    mock_submission_result.return_value = (None, "submit_plan() tool trace not found")

    errors = await _validate_planner_handoff(
        session_id=uuid4(),
        plan=RandomizationStrategy(theme="t", reasoning="r"),
        custom_objectives=None,
    )

    assert any(str(err).startswith("planner_submission:") for err in errors)


@pytest.mark.asyncio
@patch("controller.agent.benchmark.graph._get_latest_submit_for_review_result")
async def test_validate_reviewer_handoff_fails_when_submit_for_review_missing(
    mock_submission_result,
):
    mock_submission_result.return_value = (
        None,
        "submit_for_review() tool trace not found",
    )

    errors = await _validate_reviewer_handoff(session_id=uuid4())

    assert errors == ["reviewer_submission: submit_for_review() tool trace not found"]


@pytest.mark.asyncio
@patch("controller.agent.benchmark.graph._get_latest_submit_for_review_result")
async def test_validate_reviewer_handoff_fails_when_submit_for_review_not_successful(
    mock_submission_result,
):
    mock_submission_result.return_value = (
        MagicMock(success=False),
        None,
    )

    errors = await _validate_reviewer_handoff(session_id=uuid4())

    assert errors == ["reviewer_submission: submit_for_review() returned success=false"]
