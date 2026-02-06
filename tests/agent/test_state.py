from src.agent.state import AgentState
from langchain_core.messages import HumanMessage

def test_agent_state_keys():
    """Verify that AgentState has the expected keys."""
    # Since it's a TypedDict, we test by creating an instance
    state: AgentState = {
        "messages": [HumanMessage(content="Hello")],
        "task": "Do something",
        "plan": "A plan",
        "todo": "A todo list",
        "current_step": "Step 1",
        "journal": "Log entry",
        "iteration": 1
    }
    
    assert state["task"] == "Do something"
    assert state["iteration"] == 1
    assert isinstance(state["messages"], list)
    assert len(state["messages"]) == 1
    assert state["messages"][0].content == "Hello"

def test_agent_state_types():
    """Basic type checking for AgentState keys."""
    # This is more for documentation/clarity as TypedDict doesn't enforce at runtime
    state: AgentState = {
        "messages": [],
        "task": "",
        "plan": "",
        "todo": "",
        "current_step": "",
        "journal": "",
        "iteration": 0
    }
    assert isinstance(state["iteration"], int)
    assert isinstance(state["task"], str)
