import shutil
from pathlib import Path
from unittest.mock import AsyncMock, MagicMock, patch

import pytest
from langchain_core.messages import AIMessage

from controller.agent.nodes.skills import SkillsNode
from controller.agent.state import AgentState
from shared.type_checking import type_check


@type_check
@pytest.mark.asyncio
async def test_sidecar_node_suggest_skill():
    test_dir = Path("test_suggested_skills")
    if test_dir.exists():
        shutil.rmtree(test_dir)

    # Mock SharedNodeContext
    from controller.agent.nodes.base import SharedNodeContext

    mock_ctx = SharedNodeContext.create(
        worker_light_url="http://worker", session_id="test-session"
    )
    mock_ctx.dspy_lm = MagicMock()

    # Mock DSPy Program
    with patch("controller.agent.nodes.skills.dspy.CodeAct") as mock_codeact_cls:
        mock_program = MagicMock()
        mock_program.return_value = MagicMock(
            summary="Identified and recorded 1 new skills."
        )
        mock_codeact_cls.return_value = mock_program

        # Mock GitManager
        with patch("controller.agent.nodes.skills.GitManager") as mock_git:
            instance = mock_git.return_value
            instance.ensure_repo = MagicMock()
            instance.sync_changes = AsyncMock()

            node = SkillsNode(context=mock_ctx, suggested_skills_dir=str(test_dir))
            state = AgentState(
                task="Test task",
                journal="I struggled with Box until I imported it correctly.",
                session_id="test-session",
            )

            result = await node(state)

        assert "Sidecar Learner: Identified and recorded 1 new skills" in result.journal


@type_check
@pytest.mark.asyncio
async def test_sidecar_node_no_skill():
    test_dir = Path("test_suggested_skills_none")
    # Mock SharedNodeContext
    from controller.agent.nodes.base import SharedNodeContext

    mock_ctx = SharedNodeContext.create(
        worker_light_url="http://worker", session_id="test-session"
    )
    mock_ctx.dspy_lm = MagicMock()

    with patch("controller.agent.nodes.skills.dspy.CodeAct") as mock_codeact_cls:
        mock_program = MagicMock()
        mock_program.return_value = MagicMock(summary="No new skills identified.")
        mock_codeact_cls.return_value = mock_program

        # Mock GitManager
        with patch("controller.agent.nodes.skills.GitManager") as mock_git:
            instance = mock_git.return_value
            instance.ensure_repo = MagicMock()
            instance.sync_changes = AsyncMock()

            node = SkillsNode(context=mock_ctx, suggested_skills_dir=str(test_dir))
            state = AgentState(
                task="Easy task",
                journal="Everything worked perfectly.",
                session_id="test-session",
            )

            result = await node(state)

    assert "Sidecar Learner: No new skills identified" in result.journal

    if test_dir.exists():
        shutil.rmtree(test_dir)
