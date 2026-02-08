import shutil
from pathlib import Path
from unittest.mock import AsyncMock, MagicMock, patch

import pytest

from controller.agent.nodes.sidecar import SidecarNode
from controller.agent.state import AgentState
from shared.type_checking import type_check


@pytest.fixture
def mock_llm():
    with patch("controller.agent.nodes.sidecar.ChatOpenAI") as mock:
        instance = mock.return_value
        instance.ainvoke = AsyncMock()
        instance.invoke = MagicMock()
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
    from git import GitCommandError
    from unittest.mock import MagicMock, patch

    test_dir = Path("test_conflict_skills")
    if test_dir.exists():
        shutil.rmtree(test_dir)
    test_dir.mkdir(parents=True)

    # Create a dummy conflicted file with spaces
    conflict_file = test_dir / "conflict file.md"
    conflict_content = """<<<<<<< HEAD
Old Content
=======
New Content
>>>>>>> new-branch
"""
    conflict_file.write_text(conflict_content)

    # Mock Repo
    with patch("controller.agent.nodes.sidecar.Repo") as MockRepo:
        mock_repo_instance = MagicMock()
        MockRepo.return_value = mock_repo_instance

        # Simulate pull --rebase failure
        mock_repo_instance.git.pull.side_effect = GitCommandError("pull --rebase", "conflict")

        # Mock status to return the conflicted file (quoted)
        mock_repo_instance.git.status.return_value = 'UU "conflict file.md"'

        # Mock LLM resolution with markdown code block and preamble
        mock_llm.invoke.return_value = MagicMock(content="Here is the fixed code:\n```python\nResolved Content\n```")

        node = SidecarNode(suggested_skills_dir=str(test_dir))

        # Inject our mock repo
        node.repo = mock_repo_instance

        # Manually trigger _sync_git
        node._sync_git("Test commit")

        # Verify LLM was called with the conflict content
        args, _ = mock_llm.invoke.call_args
        assert "<<<<<<< HEAD" in str(args[0][0].content)
        assert ">>>>>>> new-branch" in str(args[0][0].content)

        # Verify file was updated with resolved content (stripped)
        assert conflict_file.read_text() == "Resolved Content"

        # Verify git add and rebase --continue were called
        mock_repo_instance.git.add.assert_called_with("conflict file.md")
        mock_repo_instance.git.rebase.assert_called_with("--continue")

    # Cleanup
    if test_dir.exists():
        shutil.rmtree(test_dir)
