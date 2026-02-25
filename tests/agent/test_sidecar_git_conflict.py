import shutil
from pathlib import Path
from unittest.mock import MagicMock, patch

import pytest
from git import GitCommandError

from controller.agent.nodes.skills import SkillsNode
from shared.type_checking import type_check


@pytest.fixture
def mock_repo():
    with patch("controller.utils.git.Repo") as mock:
        repo_instance = mock.return_value
        repo_instance.git.status.return_value = "UU conflict.md"
        repo_instance.git.diff.return_value = "conflict.md"
        repo_instance.working_tree_dir = "."
        yield repo_instance


@pytest.fixture
def mock_dspy_lm():
    mock_lm = MagicMock()
    return mock_lm


@type_check
@pytest.mark.asyncio
async def test_sidecar_git_conflict_resolution(mock_repo, mock_dspy_lm):
    # Setup
    test_dir = Path("test_suggested_skills_conflict")
    if test_dir.exists():
        shutil.rmtree(test_dir)
    test_dir.mkdir(parents=True, exist_ok=True)
    (test_dir / ".git").mkdir()

    # Create a dummy conflict file
    conflict_file = test_dir / "conflict.md"
    conflict_file.write_text("<<<<<<< HEAD\nOurs\n=======\nTheirs\n>>>>>>>")

    # Mock environment variables
    with patch.dict("os.environ", {"GIT_REPO_URL": "https://github.com/test/repo.git"}):
        from controller.agent.nodes.base import SharedNodeContext

        mock_ctx = SharedNodeContext.create(
            worker_light_url="http://worker", session_id="test"
        )
        mock_ctx.dspy_lm = mock_dspy_lm

        node = SkillsNode(context=mock_ctx, suggested_skills_dir=str(test_dir))

        # Mock pull to raise GitCommandError
        mock_repo.git.pull.side_effect = GitCommandError("pull", "conflict")

        # Mock unmerged files
        mock_repo.git.diff.return_value = "conflict.md"

        # Mock DSPy Predict call
        with patch("dspy.ReAct") as mock_predict:
            mock_resolver = mock_predict.return_value
            mock_resolver.return_value = MagicMock(
                resolved_content="Resolved content without markers"
            )

            await node._sync_git("Test commit")

            # Verification
            mock_predict.assert_called()
            assert conflict_file.read_text() == "Resolved content without markers"
            mock_repo.git.add.assert_any_call("conflict.md")
            mock_repo.git.rebase.assert_called_with("--continue")

    # Cleanup
    if test_dir.exists():
        shutil.rmtree(test_dir)


@type_check
@pytest.mark.asyncio
async def test_sidecar_git_conflict_resolution_abort_on_failure(
    mock_repo, mock_dspy_lm
):
    # Setup
    test_dir = Path("test_suggested_skills_abort")
    if test_dir.exists():
        shutil.rmtree(test_dir)
    test_dir.mkdir(parents=True, exist_ok=True)
    (test_dir / ".git").mkdir()

    # Create a dummy conflict file
    conflict_file = test_dir / "conflict.md"
    conflict_file.write_text("<<<<<<< HEAD\nOurs\n=======\nTheirs\n>>>>>>>")

    # Mock environment variables
    with patch.dict("os.environ", {"GIT_REPO_URL": "https://github.com/test/repo.git"}):
        from controller.agent.nodes.base import SharedNodeContext

        mock_ctx = SharedNodeContext.create(
            worker_light_url="http://worker", session_id="test"
        )
        mock_ctx.dspy_lm = mock_dspy_lm

        node = SkillsNode(context=mock_ctx, suggested_skills_dir=str(test_dir))

        # Mock pull to raise GitCommandError
        mock_repo.git.pull.side_effect = GitCommandError("pull", "conflict")

        # Mock unmerged files
        mock_repo.git.diff.return_value = "conflict.md"

        # Mock DSPy Predict to raise Exception
        with patch("dspy.ReAct") as mock_predict:
            mock_resolver = mock_predict.return_value
            mock_resolver.side_effect = Exception("DSPy failed")

            with pytest.raises(Exception, match="DSPy failed"):
                await node._sync_git("Test commit")

            # Push was NOT called
            mock_repo.git.push.assert_not_called()

    # Cleanup
    if test_dir.exists():
        shutil.rmtree(test_dir)
