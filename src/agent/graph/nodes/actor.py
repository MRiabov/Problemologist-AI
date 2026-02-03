from langchain_core.messages import SystemMessage
from typing import Optional
from src.agent.utils.llm import get_model

from src.agent.graph.state import AgentState
from src.agent.tools.env import (
    write_file,
    edit_file,
    view_file,
    run_command,
    preview_design,
    submit_design,
    search_docs,
    check_manufacturability,
    search_parts,
    preview_part,
)
from src.agent.tools.env_adapter import set_current_role
from src.agent.tools.memory import read_journal
from src.agent.utils.config import Config
from src.agent.utils.env_log import log_to_env
from src.agent.utils.prompts import get_prompt


async def actor_node(state: AgentState, tools: Optional[list] = None):
    """
    Executes the next step in the plan using tools.
    """
    set_current_role("Actor")
    log_to_env("Executing current step...", agent_role="Actor")
    model = get_model(Config.LLM_MODEL)

    # Bind tools to the model
    if tools is None:
        tools = [
            write_file,
            edit_file,
            view_file,
            run_command,
            preview_design,
            submit_design,
            search_docs,
            check_manufacturability,
            read_journal,
            search_parts,
            preview_part,
        ]
    model_with_tools = model.bind_tools(tools)

    system_prompt_key = "cad_agent.actor.system"
    system_prompt = get_prompt(system_prompt_key)

    # Check for overrides
    if (
        state.get("runtime_config")
        and "system_prompt_overrides" in state["runtime_config"]
    ):
        overrides = state["runtime_config"]["system_prompt_overrides"]
        if "actor" in overrides:
            system_prompt = get_prompt(overrides["actor"])

    if state.get("step_count", 0) > 5:
        system_prompt += (
            "\n\nCRITICAL: You have taken more than 5 steps without successful "
            "submission. Please read "
            "`docs/skills/build123d_cad_drafting_skill/SKILL.md` for guidance."
        )

    messages = [SystemMessage(content=system_prompt)] + state["messages"]

    # We might want to inject the plan as context if it's not the most recent message
    if state.get("plan"):
        messages.append(SystemMessage(content=f"Current Plan:\n{state['plan']}"))

    response = await model_with_tools.ainvoke(messages)

    updates = {
        "messages": [response],
        # Increment step count to avoid infinite loops
        "step_count": state.get("step_count", 0) + 1,
    }

    # Extract reasoning if possible
    if hasattr(response, "content") and response.content:
        updates["coder_reasoning"] = response.content
        log_to_env(response.content, type="thought", agent_role="Actor")

    if hasattr(response, "tool_calls") and response.tool_calls:
        tool_names = [tc["name"] for tc in response.tool_calls]
        log_to_env(
            f"Handing off to tools: {', '.join(tool_names)}",
            type="handoff",
            agent_role="Actor",
        )

    return updates
