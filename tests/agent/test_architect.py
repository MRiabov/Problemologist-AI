import os
import pytest
from unittest.mock import MagicMock, patch
from src.agent.nodes.architect import architect_node
from src.agent.state import AgentState

@pytest.fixture
def mock_llm():
    with patch("src.agent.nodes.architect.ChatOpenAI") as mock:
        instance = mock.return_value
        instance.invoke.return_value = MagicMock(content="""# PLAN
Test Plan
# TODO
- [ ] Test Todo""")
        yield instance

def test_architect_node_logic(mock_llm):
    # Cleanup files if they exist
    for f in ["plan.md", "todo.md"]:
        if os.path.exists(f):
            os.remove(f)
            
    state = AgentState(task="Build a robot")
    
    result = architect_node(state)
    
    # Check return value
    assert "plan" in result
    assert "todo" in result
    assert "Test Plan" in result["plan"]
    assert "Test Todo" in result["todo"]
    
    # Check file creation
    assert os.path.exists("plan.md")
    assert os.path.exists("todo.md")
    
    with open("plan.md", "r") as f:
        assert f.read() == "Test Plan"
        
    with open("todo.md", "r") as f:
        assert f.read() == "- [ ] Test Todo"
    
    # Cleanup
    os.remove("plan.md")
    os.remove("todo.md")

def test_architect_node_fallback(mock_llm):
    # Mock fallback response
    mock_llm.invoke.return_value = MagicMock(content="Just some text without sections")
    
    state = AgentState(task="Build a robot")
    result = architect_node(state)
    
    assert result["plan"] == "Just some text without sections"
    assert result["todo"] == "- [ ] Implement the plan"
    
    os.remove("plan.md")
    os.remove("todo.md")
