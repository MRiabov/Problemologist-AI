import shutil
from pathlib import Path
from unittest.mock import AsyncMock, MagicMock, patch

import pytest
from git import GitCommandError

from controller.agent.nodes.sidecar import SidecarNode
from controller.agent.state import AgentState
from shared.type_checking import type_check


@pytest.fixture
def mock_llm_git():
    with patch("controller.agent.nodes.sidecar.ChatOpenAI") as mock:
        instance = mock.return_value
        instance.ainvoke = AsyncMock()
        # First call is for skill extraction (irrelevant here, but needs to not fail)
        # Second call is for conflict resolution
        instance.ainvoke.side_effect = [
            MagicMock(content="No new skills identified."),
            MagicMock(content="Resolved Content"),
        ]
        yield instance


@pytest.fixture
def mock_repo():
    with patch("controller.agent.nodes.sidecar.Repo") as mock:
        repo_instance = mock.return_value
        mock.clone_from.return_value = repo_instance

        # Mock git object
        repo_instance.git = MagicMock()

        # Simulate pull --rebase failure
        def pull_side_effect(*args, **kwargs):
            if "--rebase" in args:
                raise GitCommandError("pull", "Conflict")
            return None

        repo_instance.git.pull.side_effect = pull_side_effect

        # Mock status to show conflict
        # UU indicates unmerged, both modified
        repo_instance.git.status.return_value = "UU conflict.md"

        yield repo_instance


@type_check
@pytest.mark.asyncio
async def test_sidecar_git_conflict_resolution(mock_llm_git, mock_repo):
    # Setup
    test_dir = Path("test_suggested_skills_git")
    if test_dir.exists():
        shutil.rmtree(test_dir)
    test_dir.mkdir()

    # Create a dummy conflicted file
    conflicted_file = test_dir / "conflict.md"
    conflicted_file.write_text("""<<<<<<< HEAD
Our Change
=======
Their Change
>>>>>>> branch
""")

    # Initialize node
    # We need to set GIT_REPO_URL env var to trigger repo init
    with patch.dict("os.environ", {"GIT_REPO_URL": "https://github.com/test/repo.git"}):
        node = SidecarNode(suggested_skills_dir=str(test_dir))

        # Manually trigger _sync_git to test the logic directly or trigger via __call__
        # __call__ calls _sync_git if a skill is found.
        # Let's mock a skill finding to trigger sync.

        # Reset llm mock to return a skill first, then the resolution
        mock_llm_git.ainvoke.side_effect = [
            MagicMock(content="SKILL: TestSkill\nCONTENT: test content"),
            MagicMock(content="Resolved Content"),
        ]

        state = AgentState(task="Test task", journal="Journal")

        # Execute
        await node(state)

        # Verify
        # 1. Pull --rebase was called
        mock_repo.git.pull.assert_called_with("--rebase")

        # 2. Status was checked
        mock_repo.git.status.assert_called_with("--porcelain")

        # 3. LLM was called for resolution (second call)
        assert mock_llm_git.ainvoke.call_count == 2
        args, _ = mock_llm_git.ainvoke.call_args_list[1]

        # args[0] is the list of messages passed to ainvoke
        messages = args[0]
        assert len(messages) == 1
        prompt_sent = messages[0].content

        assert "<<<<<<< HEAD" in prompt_sent

        # 4. File was overwritten with resolved content
        assert conflicted_file.read_text() == "Resolved Content"

        # 5. Git add was called
        mock_repo.git.add.assert_called_with("conflict.md")

        # 6. Rebase continue was called
        mock_repo.git.rebase.assert_called_with("--continue")

        # 7. Push was called
        mock_repo.git.push.assert_called()

    # Cleanup
    if test_dir.exists():
        shutil.rmtree(test_dir)
