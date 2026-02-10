import shutil
from pathlib import Path
from unittest.mock import AsyncMock, MagicMock, patch

import pytest
from git import GitCommandError

from controller.agent.nodes.sidecar import SidecarNode
from controller.agent.state import AgentState
from shared.type_checking import type_check


@pytest.fixture
def mock_repo():
    with patch("controller.agent.nodes.sidecar.Repo") as mock:
        repo_instance = mock.return_value
        repo_instance.git.status.return_value = "UU conflict.md"
        repo_instance.git.diff.return_value = "conflict.md"
        yield repo_instance


@pytest.fixture
def mock_llm():
    with patch("controller.agent.nodes.sidecar.ChatOpenAI") as mock:
        instance = mock.return_value
        instance.ainvoke = AsyncMock()
        # Mocking the conflict resolution response
        instance.ainvoke.return_value = MagicMock(
            content="Resolved content without markers"
        )
        yield instance


@type_check
@pytest.mark.asyncio
async def test_sidecar_git_conflict_resolution(mock_repo, mock_llm):
    # Setup
    test_dir = Path("test_suggested_skills_conflict")
    if test_dir.exists():
        shutil.rmtree(test_dir)
    test_dir.mkdir(parents=True, exist_ok=True)

    # Create a dummy conflict file
    conflict_file = test_dir / "conflict.md"
    conflict_file.write_text("<<<<<<< HEAD\nOurs\n=======\nTheirs\n>>>>>>>")

    # Mock environment variables
    with patch.dict("os.environ", {"GIT_REPO_URL": "https://github.com/test/repo.git"}):
        node = SidecarNode(suggested_skills_dir=str(test_dir))

        # Ensure repo is mocked correctly
        node.repo = mock_repo

        # Mock pull to raise GitCommandError
        mock_repo.git.pull.side_effect = GitCommandError("pull", "conflict")

        # Manually invoke _sync_git to test the logic directly
        # or invoke via __call__ if we want to test the full flow
        # Let's test _sync_git directly as it's the one handling the conflict
        await node._sync_git("Test commit")

        # Verification

        # 1. Verify LLM was called
        mock_llm.ainvoke.assert_called()
        call_args = mock_llm.ainvoke.call_args[0][0]
        assert "Git Merge Conflict Resolver" in str(call_args[0].content)
        assert "<<<<<<< HEAD" in str(call_args[0].content)

        # 2. Verify resolved content was written
        assert conflict_file.read_text() == "Resolved content without markers"

        # 3. Verify git add was called
        mock_repo.git.add.assert_any_call("conflict.md")

        # 4. Verify rebase --continue was called
        mock_repo.git.rebase.assert_called_with("--continue")

        # 5. Verify push was called (after successful rebase)
        mock_repo.git.push.assert_called()

    # Cleanup
    if test_dir.exists():
        shutil.rmtree(test_dir)

@type_check
@pytest.mark.asyncio
async def test_sidecar_git_conflict_resolution_abort_on_failure(mock_repo, mock_llm):
    # Setup
    test_dir = Path("test_suggested_skills_abort")
    if test_dir.exists():
        shutil.rmtree(test_dir)
    test_dir.mkdir(parents=True, exist_ok=True)

    # Create a dummy conflict file
    conflict_file = test_dir / "conflict.md"
    conflict_file.write_text("<<<<<<< HEAD\nOurs\n=======\nTheirs\n>>>>>>>")

    # Mock environment variables
    with patch.dict("os.environ", {"GIT_REPO_URL": "https://github.com/test/repo.git"}):
        node = SidecarNode(suggested_skills_dir=str(test_dir))
        node.repo = mock_repo

        # Mock pull to raise GitCommandError
        mock_repo.git.pull.side_effect = GitCommandError("pull", "conflict")

        # Mock LLM to raise Exception
        mock_llm.ainvoke.side_effect = Exception("LLM failed")

        await node._sync_git("Test commit")

        # Verify abort was called
        mock_repo.git.rebase.assert_called_with("--abort")

        # Verify push was NOT called
        mock_repo.git.push.assert_not_called()

    # Cleanup
    if test_dir.exists():
        shutil.rmtree(test_dir)
