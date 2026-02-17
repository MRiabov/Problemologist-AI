from unittest.mock import AsyncMock, MagicMock, patch
from uuid import uuid4

import pytest
from langchain_core.messages import HumanMessage, AIMessage

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
async def test_planner_node_steer_context():
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
        patch("controller.agent.nodes.base.PromptManager") as mock_pm_class,
        patch("controller.agent.nodes.base.ChatOpenAI") as mock_llm,
        patch("controller.agent.nodes.base.WorkerClient") as mock_worker,
        patch(
            "controller.agent.nodes.planner.create_react_agent"
        ) as mock_agent_factory,
    ):
        mock_pm = mock_pm_class.return_value
        mock_pm.render.return_value = "PLAN\nTODO\n- [ ] task"

        mock_llm_instance = mock_llm.return_value
        mock_llm_instance.ainvoke = AsyncMock(
            return_value=AIMessage(content="PLAN\nTODO\n- [ ] task")
        )

        mock_agent = AsyncMock()
        mock_agent.ainvoke.return_value = {
            "messages": [AIMessage(content="Plan generated")]
        }
        mock_agent_factory.return_value = mock_agent

        # Mock WorkerClient/Filesystem bits
        mock_worker_instance = mock_worker.return_value
        mock_worker_instance.write_file = AsyncMock()
        mock_worker_instance.read_file = AsyncMock(side_effect=lambda f: "content")

        await planner_node(state)

        # Verify render called with steer_context
        args, kwargs = mock_pm.render.call_args
        assert "steer_context" in kwargs
        assert "Geometric Selections:" in kwargs["steer_context"]
        assert "Code References:" in kwargs["steer_context"]
        assert "Mentions: @user" in kwargs["steer_context"]


@pytest.mark.asyncio
async def test_benchmark_planner_node_steer():
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
        mock_llm_instance = mock_ctx.llm
        mock_llm_instance.with_structured_output.return_value.ainvoke = AsyncMock(
            return_value=MagicMock(theme="test")
        )

        # Mock WorkerClient methods that might be called
        mock_worker_instance = mock_ctx.worker_client
        mock_worker_instance.read_file = AsyncMock(return_value="{}")
        mock_worker_instance.write_file = AsyncMock()
        mock_worker_instance.git_init = AsyncMock()
        mock_worker_instance.exists = AsyncMock(return_value=False)

        await benchmark_planner(state)

        # Verify ainvoke called with messages including steer_msg
        args, kwargs = (
            mock_llm_instance.with_structured_output.return_value.ainvoke.call_args
        )
        messages = args[0]
        assert any(
            m.content == "Steering prompt"
            for m in messages
            if isinstance(m, HumanMessage)
        )
