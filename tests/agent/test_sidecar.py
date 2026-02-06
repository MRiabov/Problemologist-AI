import os
import shutil
import pytest
from unittest.mock import MagicMock, patch, AsyncMock
from src.agent.nodes.sidecar import SidecarNode
from src.agent.state import AgentState

@pytest.fixture
def mock_llm():
    with patch("src.agent.nodes.sidecar.ChatOpenAI") as mock:
        instance = mock.return_value
        instance.ainvoke = AsyncMock()
        instance.ainvoke.return_value = MagicMock(content="""SKILL: Build123D Import Trick
CONTENT:
# Build123D Import Trick
## Problem
Syntax error when using Box.
## Solution
Import Box from build123d.
## Example
```python
from build123d import Box
```""")
        yield instance

@pytest.mark.asyncio
async def test_sidecar_node_suggest_skill(mock_llm):
    test_dir = "test_suggested_skills"
    if os.path.exists(test_dir):
        shutil.rmtree(test_dir)
        
    node = SidecarNode(suggested_skills_dir=test_dir)
    state = AgentState(
        task="Test task",
        journal="I struggled with Box until I imported it correctly."
    )
    
    result = await node(state)
    
    assert "Suggested skill build123d_import_trick" in result["journal"]
    assert os.path.exists(os.path.join(test_dir, "build123d_import_trick.md"))
    
    with open(os.path.join(test_dir, "build123d_import_trick.md"), "r") as f:
        content = f.read()
        assert "Build123D Import Trick" in content
        assert "from build123d import Box" in content
        
    # Cleanup
    shutil.rmtree(test_dir)

@pytest.mark.asyncio
async def test_sidecar_node_no_skill(mock_llm):
    mock_llm.ainvoke.return_value = MagicMock(content="No new skills identified.")
    
    node = SidecarNode(suggested_skills_dir="test_suggested_skills")
    state = AgentState(task="Easy task", journal="Everything worked perfectly.")
    
    result = await node(state)
    
    assert "No new skills identified" in result["journal"]
    
    if os.path.exists("test_suggested_skills"):
        shutil.rmtree("test_suggested_skills")
