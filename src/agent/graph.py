from langgraph.graph import StateGraph, START
from .state import AgentState

# Dummy node for initial skeleton
def dummy_node(state: AgentState) -> AgentState:
    return state

# Initialize the StateGraph with our AgentState
builder = StateGraph(AgentState)

# Add dummy node
builder.add_node("dummy", dummy_node)

# Placeholder for nodes:
# builder.add_node("architect", architect_node)
# builder.add_node("engineer", engineer_node)
# builder.add_node("critic", critic_node)

# Set the entry point and compile
builder.add_edge(START, "dummy")

graph = builder.compile()
