import os
from unittest.mock import MagicMock, patch

import pytest

# Set mock key before any imports that might trigger LLM initialization
os.environ["OPENAI_API_KEY"] = "sk-mock-key"

from controller.agent.nodes.coder import CoderNode
from controller.agent.nodes.electronics_engineer import ElectronicsEngineerNode
from controller.agent.nodes.planner import PlannerNode
from controller.agent.nodes.reviewer import ReviewerNode
from controller.agent.nodes.skills import SkillsNode
from controller.agent.nodes.base import SharedNodeContext
from controller.agent.state import AgentState


@pytest.mark.asyncio
async def test_all_agents_initialization():
    """Verify all agents can be initialized and have tools bound."""
    with (
        patch("controller.agent.nodes.base.WorkerClient"),
        patch("controller.agent.nodes.base.RemoteFilesystemMiddleware"),
        patch(
            "controller.agent.benchmark.nodes.get_prompt", return_value="mock prompt"
        ),
    ):
        mock_ctx = SharedNodeContext.create(
            worker_url="http://worker", session_id="test"
        )

        # 1. Engineer Coder
        coder = CoderNode(context=mock_ctx)
        assert coder.agent is not None
        assert len(coder.tools) > 0

        # 2. Engineer Electronics
        elec = ElectronicsEngineerNode(context=mock_ctx)
        assert elec.agent is not None
        assert len(elec.tools) > 0

        # 3. Engineer Planner
        planner = PlannerNode(context=mock_ctx)
        assert planner.agent is not None
        assert len(planner.tools) > 0

        # 4. Engineer Reviewer
        reviewer = ReviewerNode(context=mock_ctx)
        assert reviewer.agent is not None
        assert len(reviewer.tools) > 0

        # 5. Skill Creator (SkillsNode)
        skills = SkillsNode(context=mock_ctx)
        # Mocking config for __call__ check
        mock_state = MagicMock(spec=AgentState)
        mock_state.journal = ""
        mock_state.task = ""
        mock_state.session_id = "00000000-0000-0000-0000-000000000000"

        assert skills.ctx.llm is not None

        print("All agents instantiated or verified for refactor consistency.")
