from langchain_core.messages import SystemMessage

from src.agent.graph.state import AgentState
from src.agent.tools.env import (
    edit_script,
    preview_design,
    read_skill,
    search_docs,
    submit_design,
    update_skill,
    write_script,
)
from src.agent.tools.memory import read_journal, write_journal
from src.agent.utils.config import Config
from src.agent.utils.llm import get_model
from src.agent.utils.prompts import get_prompt


def actor_node(state: AgentState):
    """
    Executes the next step in the plan using tools.
    """
    model = get_model(Config.LLM_MODEL)

    # Bind tools to the model
    tools = [
        write_script,
        edit_script,
        preview_design,
        submit_design,
        search_docs,
        update_skill,
        read_skill,
        read_journal,
        write_journal,
    ]
    model_with_tools = model.bind_tools(tools)

    system_prompt = get_prompt("cad_agent.actor.system")

    # Mandatory skill check instruction
    system_prompt += (
        "\n\nMANDATORY: Before writing any `build123d` code, you MUST use the `read_skill` tool "
        "to read the `build123d_cad_drafting_skill` (SKILL.md). It contains expert knowledge, "
        "curated patterns, and critical pitfalls that are not in the standard documentation."
    )

    if state.get("step_count", 0) > 5:
        system_prompt += (
            "\n\nCRITICAL: You have taken more than 5 steps without successful submission. "
            "Please read `@file:.agent/skills/build123d_cad_drafting_skill/SKILL.md` for guidance. "
            "You should also use the `update_skill` tool to record any new insights, "
            "recurring patterns, or fixes you've discovered to help future attempts."
        )

    messages = [SystemMessage(content=system_prompt)] + state["messages"]

    # We might want to inject the plan as context if it's not the most recent message
    if state.get("plan"):
        messages.append(SystemMessage(content=f"Current Plan:\n{state['plan']}"))

    response = model_with_tools.invoke(messages)

    return {
        "messages": [response],
        # Increment step count to avoid infinite loops
        "step_count": state.get("step_count", 0) + 1,
    }
