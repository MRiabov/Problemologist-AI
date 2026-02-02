from typing import Literal, Optional

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
    preview_part,
    search_docs,
    search_parts,
    submit_design,
    update_skill,
    read_skill,
    list_skills,
    list_skill_files,
    init_skill,
    package_skill,
    write_script,
    check_manufacturability,
)
from src.agent.tools.memory import read_journal, write_journal
from src.agent.utils.config import Config


from functools import partial


def build_graph(
    extra_tools: Optional[list] = None, validation_tool_name: str = "submit_design"
):
    """
    Constructs the LangGraph for the VLM CAD Agent.

    Args:
        extra_tools: Optional list of additional tools to include.
        validation_tool_name: The name of the tool that triggers a critic review (default: submit_design).
    """

    builder = StateGraph(AgentState)

    # ToolNode implementation
    tools = [
        write_script,
        edit_script,
        preview_design,
        submit_design,
        search_docs,
        update_skill,
        read_skill,
        list_skills,
        list_skill_files,
        init_skill,
        package_skill,
        check_manufacturability,
        read_journal,
        write_journal,
        search_parts,
        preview_part,
    ]
    if extra_tools:
        tools.extend(extra_tools)

    # 1. Add Nodes
    builder.add_node("planner", planner_node)

    # Pass all tools to actor so it can bind them
    actor_with_tools = partial(actor_node, tools=tools)
    builder.add_node("actor", actor_with_tools)

    builder.add_node("critic", critic_node)
    builder.add_node("skill_populator", skill_populator_node)

    # Wrap ToolNode to capture state updates from tool outputs
    standard_tool_node = ToolNode(tools)

    async def tools_node(state: AgentState):
        result = await standard_tool_node.ainvoke(state)
        # The result is usually {'messages': [ToolMessage, ...]}
        
        updates = result.copy()
        
        # Check if we just ran validation
        last_ai_msg = None
        for msg in reversed(state["messages"]):
             if hasattr(msg, "tool_calls") and msg.tool_calls:
                 last_ai_msg = msg
                 break
        
        if last_ai_msg:
            for tc in last_ai_msg.tool_calls:
                if tc["name"] == validation_tool_name:
                    # Capture the attempt in full_history
                    history = state.get("full_history", [])
                    # The tool output is in the first message of result
                    tool_output = result["messages"][0].content if result["messages"] else "N/A"
                    
                    history.append({
                        "attempt": len(history) + 1,
                        "code": tc["args"].get("code", ""),
                        "reasoning": state.get("coder_reasoning", ""),
                        "errors": None if "Validation Passed!" in tool_output else tool_output
                    })
                    updates["full_history"] = history
        
        return updates

    builder.add_node("tools", tools_node)

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
                if "preview_design" in tool_names or validation_tool_name in tool_names:
                    return "critic"
                break

        return "actor"

    builder.add_conditional_edges("tools", route_tools)

    # Critic -> conditional (loop back or end)
    def route_critic(
        state: AgentState,
    ) -> Literal["planner", "actor", "skill_populator", "__end__"]:
        # Check for success indicators in the last message
        last_message = state["messages"][-1]
        if hasattr(last_message, "content"):
            content = str(last_message.content)
            if any(x in content for x in ["Validation Passed!", "Task complete", "TASK_COMPLETE"]):
                return END

        # Check step count to prevent infinite loops
        if state.get("step_count", 0) > Config.MAX_STEPS:
            return "skill_populator"

        return "planner"

    builder.add_conditional_edges("critic", route_critic)
    builder.add_edge("skill_populator", END)

    return builder
