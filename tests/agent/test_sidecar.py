from unittest.mock import AsyncMock, MagicMock, patch

import pytest

from controller.agent.nodes.sidecar import sidecar_node
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


@pytest.fixture
def mock_worker_client():
    with patch("controller.agent.nodes.sidecar.WorkerClient") as mock:
        instance = mock.return_value
        instance.write_file = AsyncMock()
        instance.write_file.return_value = True
        yield instance


@type_check
@pytest.mark.asyncio
async def test_sidecar_node_suggest_skill(mock_llm, mock_worker_client):
    state = AgentState(
        task="Test task", journal="I struggled with Box until I imported it correctly."
    )

    result = await sidecar_node(state)

    assert "Suggested skill build123d_import_trick" in result.journal

    # Verify write_file call to remote workspace
    mock_worker_client.write_file.assert_called_with(
        "suggested_skills/build123d_import_trick.md",
        """# Build123D Import Trick
## Problem
Syntax error when using Box.
## Solution
Import Box from build123d.
## Example
```python
from build123d import Box
```""",
    )


@type_check
@pytest.mark.asyncio
async def test_sidecar_node_no_skill(mock_llm, mock_worker_client):
    mock_llm.ainvoke.return_value = MagicMock(content="No new skills identified.")

    state = AgentState(task="Easy task", journal="Everything worked perfectly.")

    result = await sidecar_node(state)

    assert "No new skills identified" in result.journal

    mock_worker_client.write_file.assert_not_called()
