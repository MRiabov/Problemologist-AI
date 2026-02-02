from typing import Literal

from langgraph.graph import END, START, StateGraph
from langgraph.prebuilt import ToolNode

from src.agent.graph.nodes.actor import actor_node
from src.agent.graph.nodes.critic import critic_node
from src.agent.graph.nodes.planner import planner_node
from src.agent.graph.nodes.skill_populator import skill_populator_node
from src.agent.graph.state import AgentState
from src.agent.tools.env import (
    edit_script,
    preview_design,
    search_docs,
    submit_design,
    update_skill,
    write_script,
)
from src.agent.tools.memory import read_journal, write_journal
from src.agent.utils.config import Config


def build_graph():
    """
    Constructs the LangGraph for the VLM CAD Agent.
    """
    builder = StateGraph(AgentState)

    # 1. Add Nodes
    builder.add_node("planner", planner_node)
    builder.add_node("actor", actor_node)
    builder.add_node("critic", critic_node)
    builder.add_node("skill_populator", skill_populator_node)

    # ToolNode implementation
    tools = [
        write_script,
        edit_script,
        preview_design,
        submit_design,
        search_docs,
        update_skill,
        read_journal,
        write_journal,
    ]
    builder.add_node("tools", ToolNode(tools))

    # 2. Define Edges

    # Start -> Planner
    builder.add_edge(START, "planner")

    # Planner -> Actor
    builder.add_edge("planner", "actor")

    # Actor -> conditional (Tools or End)
    def should_continue(state: AgentState) -> Literal["tools", "__end__"]:
        messages = state["messages"]
        last_message = messages[-1]

        # If the LLM didn't make a tool call, we stop.
        if not hasattr(last_message, "tool_calls") or not last_message.tool_calls:
            return END

        # Otherwise, we execute the tools.
        return "tools"

    builder.add_conditional_edges("actor", should_continue)

    # Tools -> conditional (Critic or Actor)
    def route_tools(state: AgentState) -> Literal["critic", "actor"]:
        messages = state["messages"]
        # The last message is the *output* of the tool (ToolMessage)
        # But we need to know *which* tool was called.
        # The message before that (AIMessage) has the tool_calls.

        # In a standard graph update, 'messages' list might be appended.
        # So [-1] is ToolMessage.
        # We can check the tool_name in the ToolMessage if available, or look at the preceding AIMessage.

        # A robust way is to look at the last AIMessage's tool calls.
        # But here we are *after* the tools node, so the last message is a ToolMessage (or list of them).

        # Let's iterate backwards to find the last AIMessage and check its tool calls
        for msg in reversed(messages):
            if hasattr(msg, "tool_calls") and msg.tool_calls:
                tool_names = [tc["name"] for tc in msg.tool_calls]
                if "preview_design" in tool_names or "submit_design" in tool_names:
                    return "critic"
                break

        return "actor"

    builder.add_conditional_edges("tools", route_tools)

    # Critic -> conditional (loop back or end)
    def route_critic(state: AgentState) -> Literal["planner", "actor", "skill_populator", "__end__"]:
        # Logic to decide if we are done or need to loop.
        # For this prototype, we'll loop back to Planner to allow for re-planning or continuation.
        # Ideally, the Critic's output (AIMessage) would contain a structured decision or specific text.

        # Check step count to prevent infinite loops
        if state.get("step_count", 0) > Config.MAX_STEPS:
            return "skill_populator"

        return "planner"

    builder.add_conditional_edges("critic", route_critic)
    builder.add_edge("skill_populator", END)

    return builder
