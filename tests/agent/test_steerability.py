from unittest.mock import AsyncMock, MagicMock, patch
from uuid import uuid4

import pytest
from langchain_core.messages import HumanMessage

from controller.agent.benchmark.models import GenerationSession
from controller.agent.benchmark.state import BenchmarkGeneratorState
from controller.agent.state import AgentState, AgentStatus
from controller.graph.steerability_node import check_steering, steerability_node
from shared.models.steerability import (
    GeometricSelection,
    SteerablePrompt,
)


@pytest.mark.asyncio
async def test_steerability_node_engineer():
    session_id = str(uuid4())
    state = AgentState(
        session_id=session_id, messages=[], task="Test task", journal="Initial journal"
    )

    from shared.models.steerability import SelectionLevel

    steer_prompt = SteerablePrompt(
        text="Make it bigger",
        selections=[
            GeometricSelection(
                level=SelectionLevel.PART, target_id="P1", center=(0.0, 0.0, 0.0)
            )
        ],
    )

    with patch(
        "controller.graph.steerability_node.steerability_service"
    ) as mock_service:
        mock_service.get_queued_prompts = AsyncMock(return_value=[steer_prompt])
        mock_service.dequeue_prompt = AsyncMock(return_value=steer_prompt)

        update = await steerability_node(state)

        assert len(update["messages"]) == 1
        assert update["messages"][0].content == "Make it bigger"
        assert update["status"] == AgentStatus.PLAN_REJECTED
        assert "[User Steering] Make it bigger" in update["journal"]
        assert "steerability" in update["messages"][0].additional_kwargs


@pytest.mark.asyncio
async def test_steerability_node_benchmark():
    session_id = str(uuid4())  # Use string for consistency
    session = GenerationSession(session_id=session_id, prompt="Generate benchmark")
    # Pass as dict to satisfy steerability_node's current implementation
    state = {
        "session": session,
        "messages": [],
        "review_feedback": None,
        "plan": None,
        "current_script": "",
        "simulation_result": None,
        "review_round": 0,
    }

    steer_prompt = SteerablePrompt(text="Increase gravity")

    with patch(
        "controller.graph.steerability_node.steerability_service"
    ) as mock_service:
        mock_service.get_queued_prompts = AsyncMock(return_value=[steer_prompt])
        mock_service.dequeue_prompt = AsyncMock(return_value=steer_prompt)

        update = await steerability_node(state)

        assert len(update["messages"]) == 1
        assert update["messages"][0].content == "Increase gravity"
        assert update["review_feedback"] == "Steering: Increase gravity"


@pytest.mark.asyncio
async def test_check_steering():
    session_id = str(uuid4())
    state = AgentState(session_id=session_id)

    with patch(
        "controller.graph.steerability_node.steerability_service"
    ) as mock_service:
        mock_service.get_queued_prompts = AsyncMock(
            return_value=[SteerablePrompt(text="hi")]
        )
        res = await check_steering(state)
        assert res == "steer"

        mock_service.get_queued_prompts = AsyncMock(return_value=[])
        res = await check_steering(state)
        assert res == "next"


@pytest.mark.asyncio
@patch("controller.agent.nodes.planner.record_worker_events")
@patch("controller.agent.nodes.planner.PlannerNode._run_program")
@patch(
    "worker_heavy.utils.file_validation.validate_node_output", return_value=(True, [])
)
async def test_planner_node_steer_context(
    mock_validate, mock_run_program, mock_record_events
):
    from controller.agent.nodes.planner import planner_node

    session_id = str(uuid4())
    steer_msg = HumanMessage(
        content="Fix this",
        additional_kwargs={
            "steerability": {
                "selections": [
                    {"level": "PART", "target_id": "P1", "center": (1.0, 2.0, 3.0)}
                ],
                "code_references": [
                    {"file_path": "a.py", "start_line": 1, "end_line": 10}
                ],
                "mentions": ["@user"],
            }
        },
    )
    state = AgentState(session_id=session_id, messages=[steer_msg], task="Test task")

    with (
        patch("controller.agent.nodes.planner.SharedNodeContext") as mock_ctx_class,
    ):
        from controller.agent.nodes.base import SharedNodeContext

        mock_ctx = MagicMock(spec=SharedNodeContext)
        mock_ctx.session_id = session_id
        mock_ctx.worker_light_url = "http://worker"
        mock_ctx.pm = MagicMock()
        mock_ctx.dspy_lm = MagicMock()
        mock_ctx.worker_client = MagicMock()
        mock_ctx.worker_client.exists = AsyncMock(return_value=True)
        mock_ctx.worker_client.read_file = AsyncMock(return_value="content")
        mock_ctx.fs = MagicMock()
        mock_ctx.fs.read_file = AsyncMock(return_value="file content")

        mock_ctx_class.create.return_value = mock_ctx

        # Mock _run_program return value
        mock_prediction = MagicMock(summary="Plan generated")
        mock_run_program.return_value = (
            mock_prediction,
            {"plan.md": "plan", "todo.md": "todo"},
            "\nJournal",
        )

        await planner_node(state)

        # Verify _run_program called with inputs containing steer_context
        args, kwargs = mock_run_program.call_args
        # args[3] is the inputs dict in _run_program(self, program_cls, signature_cls, state, inputs, ...)
        inputs = args[3]
        assert "steer_context" in inputs
        assert "Geometric Selections:" in inputs["steer_context"]
        assert "Code References:" in inputs["steer_context"]
        assert "Mentions: @user" in inputs["steer_context"]


@pytest.mark.asyncio
@patch("controller.agent.benchmark.nodes.BenchmarkPlannerNode._run_program")
async def test_benchmark_planner_node_steer(mock_run_program):
    from controller.agent.benchmark.nodes import planner_node as benchmark_planner

    session_id = uuid4()
    session = GenerationSession(session_id=session_id, prompt="Generate benchmark")
    steer_msg = HumanMessage(content="Steering prompt")
    state = BenchmarkGeneratorState(
        session=session,
        messages=[steer_msg],
        plan=None,
        review_feedback=None,
        current_script="",
        simulation_result=None,
        review_round=0,
    )

    with (
        patch("controller.agent.benchmark.nodes.SharedNodeContext") as mock_ctx_class,
    ):
        mock_ctx = mock_ctx_class.create.return_value
        mock_ctx.worker_client.read_file = AsyncMock(return_value="{}")
        mock_ctx.worker_client.exists = AsyncMock(return_value=False)
        mock_ctx.worker_client.git_init = AsyncMock()

        # Mock _run_program
        from shared.simulation.schemas import RandomizationStrategy

        mock_prediction = MagicMock(
            plan=RandomizationStrategy(theme="test", reasoning="test")
        )
        mock_run_program.return_value = (mock_prediction, {}, "\nJournal")

        await benchmark_planner(state)

        # Verify _run_program called with inputs containing history with steer_msg
        args, kwargs = mock_run_program.call_args
        inputs = args[3]
        history = inputs["history"]
        assert "Steering prompt" in history
