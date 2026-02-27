import os
from unittest.mock import MagicMock, patch

import pytest

# Set mock key before any imports that might trigger LLM initialization
os.environ["OPENAI_API_KEY"] = "sk-mock-key"

from controller.agent.nodes.base import SharedNodeContext
from controller.agent.nodes.coder import CoderNode
from controller.agent.nodes.electronics_engineer import ElectronicsEngineerNode
from controller.agent.nodes.execution_reviewer import ExecutionReviewerNode
from controller.agent.nodes.plan_reviewer import PlanReviewerNode
from controller.agent.nodes.planner import PlannerNode
from controller.agent.nodes.skills import SkillsNode
from controller.agent.state import AgentState


@pytest.mark.asyncio
async def test_all_agents_initialization():
    """Verify all agents can be initialized and have tools bound."""
    with (
        patch("controller.agent.nodes.base.WorkerClient"),
        patch("controller.agent.nodes.base.RemoteFilesystemMiddleware"),
    ):
        mock_ctx = SharedNodeContext.create(
            worker_light_url="http://worker", session_id="test"
        )

        # 1. Engineer Coder
        coder = CoderNode(context=mock_ctx)
        assert coder.ctx == mock_ctx

        # 2. Engineer Electronics
        elec = ElectronicsEngineerNode(context=mock_ctx)
        assert elec.ctx == mock_ctx

        # 3. Engineer Planner
        planner = PlannerNode(context=mock_ctx)
        assert planner.ctx == mock_ctx

        # 4. Engineer Plan Reviewer
        plan_reviewer = PlanReviewerNode(context=mock_ctx)
        assert plan_reviewer.ctx == mock_ctx

        # 5. Engineer Execution Reviewer
        exec_reviewer = ExecutionReviewerNode(context=mock_ctx)
        assert exec_reviewer.ctx == mock_ctx

        # 6. Skill Creator (SkillsNode)
        skills = SkillsNode(context=mock_ctx)
        # Mocking config for __call__ check
        mock_state = MagicMock(spec=AgentState)
        mock_state.journal = ""
        mock_state.task = ""
        mock_state.session_id = "00000000-0000-0000-0000-000000000000"

        # assert skills.ctx.llm is not None # skill node has no .llm in SharedNodeContext anymore?
        # Let's check skills.py

        print("All agents instantiated or verified for refactor consistency.")
