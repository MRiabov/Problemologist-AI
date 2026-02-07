from controller.agent.state import AgentState
from langchain_core.messages import HumanMessage

def test_agent_state_keys():
    """Verify that AgentState has the expected fields."""
    state = AgentState(
        messages=[HumanMessage(content="Hello")],
        task="Do something",
        plan="A plan",
        todo="A todo list",
        current_step="Step 1",
        journal="Log entry",
        iteration=1
    )
    
    assert state.task == "Do something"
    assert state.iteration == 1
    assert isinstance(state.messages, list)
    assert len(state.messages) == 1
    assert state.messages[0].content == "Hello"

def test_agent_state_defaults():
    """Verify default values for AgentState."""
    state = AgentState()
    assert state.messages == []
    assert state.task == ""
    assert state.plan == ""
    assert state.todo == ""
    assert state.current_step == ""
    assert state.journal == ""
    assert state.iteration == 0
