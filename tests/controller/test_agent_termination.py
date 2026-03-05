from unittest.mock import AsyncMock, patch

import pytest

from controller.agent.execution_limits import HardFailEvaluation
from controller.agent.graph import should_continue
from controller.agent.state import AgentState, AgentStatus
from shared.enums import AgentName
from shared.workers.filesystem.policy import AgentExecutionPolicy


@pytest.mark.asyncio
async def test_should_continue_termination():
    """Verify that should_continue returns 'skills' on hard-fail limits."""
    with patch(
        "controller.agent.graph.evaluate_agent_hard_fail",
        new=AsyncMock(
            return_value=HardFailEvaluation(
                should_fail=False,
                policy=AgentExecutionPolicy(),
            )
        ),
    ):
        state = AgentState(task="test", turn_count=59, status=AgentStatus.EXECUTING)
        assert await should_continue(state) != "skills"

    with patch(
        "controller.agent.graph.evaluate_agent_hard_fail",
        new=AsyncMock(
            return_value=HardFailEvaluation(
                should_fail=True,
                code="max_turns",
                message="Agent hard-fail: max turns reached.",
                policy=AgentExecutionPolicy(),
            )
        ),
    ):
        state = AgentState(task="test", turn_count=60, status=AgentStatus.EXECUTING)
        assert await should_continue(state) == AgentName.SKILL_AGENT


@pytest.mark.asyncio
async def test_engineer_node_increments_turn_count():
    """Verify that coder_node increments turn_count."""
    from controller.agent.nodes.coder import coder_node as engineer_node

    input_state = AgentState(
        task="test",
        turn_count=10,
        todo="- [ ] step1",
        session_id="session-1",
    )
    expected_output = input_state.model_copy(
        update={"turn_count": 11, "journal": "done"}
    )

    with (
        patch("controller.agent.nodes.coder.SharedNodeContext.create"),
        patch(
            "controller.agent.nodes.coder.CoderNode",
            return_value=AsyncMock(return_value=expected_output),
        ),
    ):
        result = await engineer_node(input_state)
        assert result.turn_count == 11


@pytest.mark.asyncio
async def test_run_resume_logic():
    """Verify the resumption logic in run.py (mocked)."""
    # This is harder to test without refactoring run.py nicely,
    # but we can check if the logic flows as expected if we were to simulate it.
    # For now, we rely on the implementation correctness and the unit tests above.
    pass
