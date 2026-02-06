from langgraph.graph import StateGraph, START, END
from .state import AgentState
from .nodes.architect import architect_node
from .nodes.engineer import engineer_node

# Placeholder for Critic
def critic_node(state: AgentState) -> AgentState:
    return state

# Initialize the StateGraph with our AgentState
builder = StateGraph(AgentState)

# Add nodes
builder.add_node("architect", architect_node)
builder.add_node("engineer", engineer_node)
builder.add_node("critic", critic_node)

# Set the entry point and edges
builder.add_edge(START, "architect")
builder.add_edge("architect", "engineer")
builder.add_edge("engineer", "critic")
builder.add_edge("critic", END)

graph = builder.compile()
