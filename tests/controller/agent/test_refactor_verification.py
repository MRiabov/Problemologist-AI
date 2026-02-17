import os
from unittest.mock import patch

import pytest

# Set mock key before any imports that might trigger LLM initialization
os.environ["OPENAI_API_KEY"] = "sk-mock-key"

from controller.agent.nodes.base import SharedNodeContext
from controller.agent.nodes.coder import CoderNode
from controller.agent.nodes.electronics_engineer import ElectronicsEngineerNode
from controller.agent.nodes.planner import PlannerNode
from controller.agent.nodes.reviewer import ReviewerNode
from controller.agent.nodes.skills import SkillsNode


@pytest.mark.asyncio
async def test_all_agents_initialization():
    """Verify all agents can be initialized and have tools bound."""
    with (
        patch("controller.agent.nodes.base.WorkerClient"),
        patch("controller.agent.nodes.base.RemoteFilesystemMiddleware"),
        patch("controller.agent.nodes.skills.GitManager"),
        patch("controller.agent.benchmark.nodes.WorkerClient", create=True),
        patch("controller.agent.benchmark.nodes.RemoteFilesystemMiddleware", create=True),
    ):
        ctx = SharedNodeContext.create(worker_url="http://mock", session_id="test")

        # 1. Engineer Coder
        coder = CoderNode(context=ctx)
        assert hasattr(coder, 'agent') or hasattr(coder, 'coder')
        assert len(coder.tools) > 0

        # 2. Engineer Electronics
        elec = ElectronicsEngineerNode(context=ctx)
        assert hasattr(elec, 'agent')
        assert len(elec.tools) > 0

        # 3. Engineer Planner
        planner = PlannerNode(context=ctx)
        assert hasattr(planner, 'agent') or hasattr(planner, 'planner')
        assert len(planner.tools) > 0

        # 4. Engineer Reviewer
        reviewer = ReviewerNode(context=ctx)
        assert hasattr(reviewer, 'agent') or hasattr(reviewer, 'reviewer')
        assert len(reviewer.tools) > 0

        # 5. Skill Creator (SkillsNode)
        skills = SkillsNode(context=ctx)
        assert skills.ctx.llm is not None

        print("All agents instantiated or verified for refactor consistency.")


if __name__ == "__main__":
    import asyncio

    asyncio.run(test_all_agents_initialization())
