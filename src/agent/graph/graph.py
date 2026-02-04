import re
import yaml
from contextlib import suppress
from typing import Literal, Optional

from langgraph.graph import END, START, StateGraph
from langgraph.prebuilt import ToolNode

from src.agent.graph.nodes.coder import coder_node
from src.agent.graph.nodes.reviewer import reviewer_node
from src.agent.graph.nodes.planner import planner_node
from src.agent.graph.nodes.learner import learner_node
from src.agent.graph.state import AgentState
from src.agent.tools.env_adapter import get_runtime
from src.agent.tools.registry import AGENT_TOOLS
from src.agent.utils.config import Config


from functools import partial


def build_graph(
    extra_tools: Optional[list] = None, validation_tool_name: str = "verify_solution"
):
    """
    Constructs the LangGraph for the VLM CAD Agent.

    Args:
        extra_tools: Optional list of additional tools to include.
        validation_tool_name: The name of the tool that triggers a reviewer review (default: verify_solution).
    """

    builder = StateGraph(AgentState)

    # Use the canonical tool registry
    tools = AGENT_TOOLS.copy()
    if extra_tools:
        tools.extend(extra_tools)

    # 1. Add Nodes
    builder.add_node("planner", planner_node)

    # Pass all tools to coder so it can bind them
    coder_with_tools = partial(coder_node, tools=tools)
    builder.add_node("coder", coder_with_tools)

    builder.add_node("reviewer", reviewer_node)
    builder.add_node("learner", learner_node)

    # Wrap ToolNode to capture state updates from tool outputs
    standard_tool_node = ToolNode(tools)

    async def tools_node(state: AgentState):
        # Inject runtime into tool calls
        runtime_id = state.get("runtime_id", "default")
        runtime = get_runtime(runtime_id)

        last_msg = state["messages"][-1]
        if hasattr(last_msg, "tool_calls") and last_msg.tool_calls:
            for tc in last_msg.tool_calls:
                tc["args"]["tool_runtime"] = runtime

        result = await standard_tool_node.ainvoke(state)
        # The result is usually {'messages': [ToolMessage, ...]}

        updates = result.copy()
        task_complete = False

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
                    tool_output = (
                        result["messages"][0].content if result["messages"] else "N/A"
                    )

                    # Success detection moved to Reviewer node (structured YAML)
                    task_complete = False

                    history.append(
                        {
                            "attempt": len(history) + 1,
                            "code": tc["args"].get("code", ""),
                            "reasoning": state.get("coder_reasoning", ""),
                            "errors": None if task_complete else tool_output,
                        }
                    )
                    updates["full_history"] = history

        updates["task_complete"] = task_complete
        return updates

    builder.add_node("tools", tools_node)

    # 2. Define Edges

    # Start -> Planner
    builder.add_edge(START, "planner")

    # Planner -> Coder
    builder.add_edge("planner", "coder")

    # Coder -> conditional (Tools or End)
    def should_continue(state: AgentState) -> Literal["tools", "__end__"]:
        messages = state["messages"]
        last_message = messages[-1]

        # If the LLM didn't make a tool call, we stop.
        if not hasattr(last_message, "tool_calls") or not last_message.tool_calls:
            return END

        # Otherwise, we execute the tools.
        return "tools"

    builder.add_conditional_edges("coder", should_continue)

    # Tools -> conditional (Reviewer or Coder)
    def route_tools(state: AgentState) -> Literal["reviewer", "coder"]:
        messages = state["messages"]
        # The last message is the *output* of the tool (ToolMessage)

        # Build a map of tools for quick lookup
        tool_map = {t.name: t for t in AGENT_TOOLS}

        # Iterate backwards to find the last AIMessage and check its tool calls
        for msg in reversed(messages):
            if hasattr(msg, "tool_calls") and msg.tool_calls:
                for tc in msg.tool_calls:
                    tool_name = tc["name"]
                    tool = tool_map.get(tool_name)

                    # Check for explicit review trigger
                    if tool and getattr(tool, "triggers_review", False):
                        return "reviewer"

                    # Fallback/Safety: Check if validation tool specifically named in arg
                    if tool_name == validation_tool_name:
                        return "reviewer"

                break

        return "coder"

    builder.add_conditional_edges("tools", route_tools)

    # Reviewer -> conditional (loop back or end)
    def route_reviewer(
        state: AgentState,
    ) -> Literal["planner", "coder", "learner", "__end__"]:
        last_message = state["messages"][-1]
        if not hasattr(last_message, "content") or not last_message.content:
            return "planner"

        content = str(last_message.content)

        # Parse YAML frontmatter
        match = re.search(r"^---\s*\n(.*?)\n---\s*\n", content, re.DOTALL)
        metadata = {}
        if match:
            with suppress(Exception):
                metadata = yaml.safe_load(match.group(1))

        # Check task_complete from metadata
        if metadata.get("task_complete"):
            return END

        # Check status from metadata
        status = metadata.get("status")
        if status == "pass":
            return END
        if status == "replan":
            return "planner"

        # Fallback: Check for legacy success indicators
        if any(
            x in content
            for x in ["Validation Passed!", "Task complete", "TASK_COMPLETE"]
        ):
            return END

        # Check step count to prevent infinite loops
        if state.get("step_count", 0) > Config.MAX_STEPS:
            return "learner"

        return "planner"

    builder.add_conditional_edges("reviewer", route_reviewer)
    builder.add_edge("learner", END)

    return builder
