from unittest.mock import AsyncMock, MagicMock, patch
from uuid import uuid4

import pytest

from controller.agent.benchmark.models import GenerationSession
from controller.agent.benchmark.nodes import plan_reviewer_node, reviewer_node
from controller.agent.benchmark.state import BenchmarkGeneratorState
from shared.enums import ReviewDecision
from shared.models.schemas import ReviewResult


@pytest.fixture
def mock_worker():
    with patch("controller.agent.nodes.base.WorkerClient") as mock:
        instance = mock.return_value
        instance.read_file = AsyncMock(return_value="# Mock content")
        instance.exists = AsyncMock(return_value=True)
        instance.write_file = AsyncMock(return_value=True)
        instance.list_files = AsyncMock(return_value=[])
        yield instance


def _benchmark_state() -> BenchmarkGeneratorState:
    return BenchmarkGeneratorState(
        session=GenerationSession(
            session_id=uuid4(),
            prompt="Generate benchmark reviewer test",
        ),
        journal="benchmark journal",
    )


@pytest.mark.asyncio
@patch("controller.agent.benchmark.nodes.record_worker_events", new_callable=AsyncMock)
@patch("controller.agent.benchmark.nodes.BenchmarkPlanReviewerNode._run_program")
async def test_benchmark_plan_reviewer_emits_checklist_event(
    mock_run, mock_record_worker_events, mock_worker
):
    checklist = {
        "artifacts_complete": "pass",
        "cross_artifact_consistency": "pass",
    }
    mock_run.return_value = (
        MagicMock(
            review=ReviewResult(
                decision=ReviewDecision.APPROVED,
                reason="Plan looks valid.",
                checklist=checklist,
            )
        ),
        [],
        "\nBenchmark Plan Reviewer Journal",
    )

    result = await plan_reviewer_node(_benchmark_state())

    assert result.review_decision == ReviewDecision.APPROVED
    mock_record_worker_events.assert_awaited_once()
    event = mock_record_worker_events.await_args.kwargs["events"][0]
    assert event.checklist == checklist


@pytest.mark.asyncio
@patch("controller.agent.benchmark.nodes.record_worker_events", new_callable=AsyncMock)
@patch(
    "controller.agent.benchmark.nodes.BenchmarkReviewerNode._ensure_submit_for_review_succeeded",
    new_callable=AsyncMock,
)
@patch("controller.agent.benchmark.nodes.BenchmarkReviewerNode._run_program")
async def test_benchmark_execution_reviewer_emits_checklist_event(
    mock_run,
    mock_handover_ok,
    mock_record_worker_events,
    mock_worker,
):
    checklist = {
        "latest_revision_verified": "pass",
        "simulation_success": "pass",
        "robustness": "fail",
    }
    mock_handover_ok.return_value = None
    mock_run.return_value = (
        MagicMock(
            review=ReviewResult(
                decision=ReviewDecision.REJECT_CODE,
                reason="Robustness is insufficient.",
                checklist=checklist,
            )
        ),
        [],
        "\nBenchmark Reviewer Journal",
    )

    result = await reviewer_node(_benchmark_state())

    assert result.review_decision == ReviewDecision.REJECT_CODE
    mock_record_worker_events.assert_awaited_once()
    event = mock_record_worker_events.await_args.kwargs["events"][0]
    assert event.checklist == checklist
