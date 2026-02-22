import shutil
from pathlib import Path
from unittest.mock import AsyncMock, MagicMock, patch

import pytest

from controller.agent.nodes.skills import SkillsNode
from controller.agent.state import AgentState
from shared.type_checking import type_check


@type_check
@pytest.mark.asyncio
@patch("controller.agent.nodes.skills.SkillsNode._run_program")
@patch("controller.agent.nodes.skills.GitManager")
@patch("controller.agent.nodes.skills.SharedNodeContext")
async def test_sidecar_node_suggest_skill(mock_ctx_cls, mock_git, mock_run_program):
    test_dir = Path("test_suggested_skills")
    if test_dir.exists():
        shutil.rmtree(test_dir)

    # Mock SharedNodeContext
    from controller.agent.nodes.base import SharedNodeContext

    mock_ctx = SharedNodeContext(
        worker_light_url="http://worker",
        session_id="test-session",
        pm=MagicMock(),
        dspy_lm=MagicMock(),
        worker_client=MagicMock(),
        fs=MagicMock(),
    )
    mock_ctx_cls.create.return_value = mock_ctx

    # Mock GitManager
    instance = mock_git.return_value
    instance.ensure_repo = MagicMock()
    instance.sync_changes = AsyncMock()

    # Mock _run_program
    mock_prediction = MagicMock(summary="Identified and recorded 1 new skills.")
    mock_run_program.return_value = (
        mock_prediction,
        {},
        "\nSidecar Learner: Identified and recorded 1 new skills.",
    )

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
@patch("controller.agent.nodes.skills.SkillsNode._run_program")
@patch("controller.agent.nodes.skills.GitManager")
@patch("controller.agent.nodes.skills.SharedNodeContext")
async def test_sidecar_node_no_skill(mock_ctx_cls, mock_git, mock_run_program):
    test_dir = Path("test_suggested_skills_none")
    if test_dir.exists():
        shutil.rmtree(test_dir)

    # Mock SharedNodeContext
    from controller.agent.nodes.base import SharedNodeContext

    mock_ctx = SharedNodeContext(
        worker_light_url="http://worker",
        session_id="test-session",
        pm=MagicMock(),
        dspy_lm=MagicMock(),
        worker_client=MagicMock(),
        fs=MagicMock(),
    )
    mock_ctx_cls.create.return_value = mock_ctx

    # Mock GitManager
    instance = mock_git.return_value
    instance.ensure_repo = MagicMock()
    instance.sync_changes = AsyncMock()

    # Mock _run_program
    mock_prediction = MagicMock(summary="No new skills identified.")
    mock_run_program.return_value = (
        mock_prediction,
        {},
        "\nSidecar Learner: No new skills identified.",
    )

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
