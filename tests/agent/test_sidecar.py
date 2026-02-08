import shutil
import os
from pathlib import Path
from unittest.mock import AsyncMock, MagicMock, patch

import pytest
from git import GitCommandError

from controller.agent.nodes.sidecar import SidecarNode
from controller.agent.state import AgentState
from shared.type_checking import type_check


@pytest.fixture
def mock_llm():
    with patch("controller.agent.nodes.sidecar.ChatOpenAI") as mock:
        instance = mock.return_value
        instance.ainvoke = AsyncMock()
        instance.ainvoke.return_value = MagicMock(
            content="""SKILL: Build123D Import Trick
CONTENT:
# Build123D Import Trick
## Problem
Syntax error when using Box.
## Solution
Import Box from build123d.
## Example
```python
from build123d import Box
```"""
        )
        yield instance


@type_check
@pytest.mark.asyncio
async def test_sidecar_node_suggest_skill(mock_llm):
    test_dir = Path("test_suggested_skills")
    if test_dir.exists():
        shutil.rmtree(test_dir)

    node = SidecarNode(suggested_skills_dir=str(test_dir))
    state = AgentState(
        task="Test task", journal="I struggled with Box until I imported it correctly."
    )

    result = await node(state)

    assert "Suggested skill build123d_import_trick" in result.journal
    skill_file = test_dir / "build123d_import_trick.md"
    assert skill_file.exists()

    with skill_file.open("r") as f:
        content = f.read()
        assert "Build123D Import Trick" in content
        assert "from build123d import Box" in content

    # Cleanup
    shutil.rmtree(test_dir)


@type_check
@pytest.mark.asyncio
async def test_sidecar_node_no_skill(mock_llm):
    mock_llm.ainvoke.return_value = MagicMock(content="No new skills identified.")

    test_dir = Path("test_suggested_skills")
    node = SidecarNode(suggested_skills_dir=str(test_dir))
    state = AgentState(task="Easy task", journal="Everything worked perfectly.")

    result = await node(state)

    assert "No new skills identified" in result.journal

    if test_dir.exists():
        shutil.rmtree(test_dir)


@type_check
@pytest.mark.asyncio
async def test_sidecar_git_conflict_resolution(mock_llm):
    test_dir = Path("test_suggested_skills_conflict")
    if test_dir.exists():
        shutil.rmtree(test_dir)

    # Setup conflict content
    conflict_content = """<<<<<<< HEAD
Our version
=======
Their version
>>>>>>> branch"""
    resolved_content = "Resolved version"

    # Mock LLM response for conflict resolution
    # Note: the first call is for skill suggestion, second for resolution
    mock_llm.ainvoke.side_effect = [
        MagicMock(content="""SKILL: Conflict Skill
CONTENT:
Some content
"""),
        MagicMock(content=resolved_content)
    ]

    with patch("controller.agent.nodes.sidecar.Repo") as MockRepo:
        mock_repo_instance = MockRepo.return_value
        MockRepo.clone_from.return_value = mock_repo_instance

        # Simulate pull --rebase failure
        mock_repo_instance.git.pull.side_effect = GitCommandError("pull --rebase", 128)

        # Simulate git status returning unmerged file
        # 'UU' indicates unmerged
        mock_repo_instance.git.status.return_value = "UU conflict_skill.md"

        # We need GIT_REPO_URL to be set so _ensure_repo attempts to use Repo
        with patch.dict(os.environ, {"GIT_REPO_URL": "https://github.com/mock/repo.git"}):
             node = SidecarNode(suggested_skills_dir=str(test_dir))

        # Create the file that will conflict so open() works (it's called before git operations in the test setup implicitly? No.)
        # The node logic writes the skill file first, then calls _sync_git.
        # But _sync_git logic reads the file to resolve conflict.
        # The file will be written by "suggested skill" logic first.
        # Then _sync_git pulls, fails, and we need to simulate that the pull messed up the file content with markers.

        # Wait! The logic in `__call__` writes the NEW skill file.
        # Then `_sync_git` calls `git add` and `git commit`.
        # Then `git pull --rebase`.
        # If `git pull` fails, the file on disk might have been updated by git to contain conflict markers.
        # So we need to ensure the file on disk has conflict markers BEFORE the resolution logic reads it.
        # Since we are mocking `git pull`, we need to manually update the file content to simulate what git would have done
        # OR we just rely on our mocks.

        # But `_sync_git` logic is:
        # 1. pull raises exception.
        # 2. catch exception.
        # 3. read file.

        # So between step 1 and 3, the file content must change to have conflict markers.
        # Since step 1 is a mock side_effect, we can use a side_effect function that updates the file!

        def pull_side_effect(*args, **kwargs):
            # Write conflict markers to the file
            skill_file = test_dir / "conflict_skill.md"
            with open(skill_file, "w") as f:
                f.write(conflict_content)
            raise GitCommandError("pull --rebase", 128)

        mock_repo_instance.git.pull.side_effect = pull_side_effect

        state = AgentState(
            task="Test conflict", journal="I found a conflict."
        )

        await node(state)

        # Verification
        # 1. pull --rebase was called
        mock_repo_instance.git.pull.assert_called_with("--rebase")

        # 2. status was checked
        mock_repo_instance.git.status.assert_called_with(porcelain=True)

        # 3. add was called
        mock_repo_instance.git.add.assert_called_with("conflict_skill.md")

        # 4. rebase --continue was called
        mock_repo_instance.git.rebase.assert_called_with("--continue")

        # 5. File content is updated
        skill_file = test_dir / "conflict_skill.md"
        with open(skill_file, "r") as f:
            assert f.read() == resolved_content

    # Cleanup
    if test_dir.exists():
        shutil.rmtree(test_dir)
