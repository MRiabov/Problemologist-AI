from langgraph.graph import StateGraph, END
from langgraph.prebuilt import ToolNode
from src.agent.graph.state import AgentState
from src.agent.graph.nodes.planner import planner_node
from src.agent.graph.nodes.actor import actor_node, TOOLS
from src.agent.graph.nodes.critic import critic_node
from src.agent.utils.config import Config


def should_continue(state: AgentState):
    """
    Conditional edge to determine whether to continue tool execution, 
    move to criticism, or end.
    """
    messages = state["messages"]
    last_message = messages[-1]

    # If there are tool calls, decide based on which tools are called
    if hasattr(last_message, "tool_calls") and last_message.tool_calls:
        for tool_call in last_message.tool_calls:
            if tool_call["name"] in ["preview_design", "submit_design"]:
                return "critic"
        return "tools"

    # Safety check for max steps
    if state.get("step_count", 0) >= Config.MAX_STEPS:
        return "end"

    # If no tool calls and we haven't hit max steps, 
    # we might be at the end of a thought process
    return "end"


def build_graph():
    """
    Constructs the LangGraph for the VLM CAD Agent.
    """
    workflow = StateGraph(AgentState)

    # Add Nodes
    workflow.add_node("planner", planner_node)
    workflow.add_node("actor", actor_node)
    workflow.add_node("critic", critic_node)
    workflow.add_node("tools", ToolNode(TOOLS))

    # Set Entry Point
    workflow.set_entry_point("planner")

    # Define Edges
    workflow.add_edge("planner", "actor")
    
    # Conditional Edges from Actor
    workflow.add_conditional_edges(
        "actor",
        should_continue,
        {
            "tools": "tools",
            "critic": "critic",
            "end": END
        }
    )

    # Tools always go back to actor
    workflow.add_edge("tools", "actor")

    # Critic can go back to actor (to fix issues) or end
    # For simplicity in this WP, we'll route back to actor 
    # if it's not the end of the simulation.
    workflow.add_edge("critic", "actor")

    return workflow.compile()