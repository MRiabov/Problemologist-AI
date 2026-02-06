from langgraph.graph import StateGraph, START, END
from .state import AgentState
from .nodes.architect import architect_node

# Dummy node for future nodes
def dummy_node(state: AgentState) -> AgentState:
    return state

# Initialize the StateGraph with our AgentState
builder = StateGraph(AgentState)

# Add nodes
builder.add_node("architect", architect_node)
builder.add_node("dummy", dummy_node)

# Set the entry point and edges
builder.add_edge(START, "architect")
builder.add_edge("architect", "dummy")
builder.add_edge("dummy", END)

graph = builder.compile()
