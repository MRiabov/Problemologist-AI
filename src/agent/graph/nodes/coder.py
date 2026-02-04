from langchain_core.messages import SystemMessage, trim_messages
from typing import Optional
from src.agent.utils.llm import get_model

from src.agent.graph.state import AgentState
from src.agent.tools.registry import AGENT_TOOLS
from src.agent.utils.config import Config
from src.agent.utils.logging import get_logger
from src.agent.utils.prompts import get_prompt

logger = get_logger(__name__)


async def coder_node(state: AgentState, tools: Optional[list] = None):
    """
    Executes the next step in the plan using tools.
    """
    log = logger.bind(agent_role="Coder")
    log.info("Executing current step")
    model = get_model(Config.LLM_MODEL)

    # Use provided tools or fall back to registry
    if tools is None:
        tools = AGENT_TOOLS
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

    # Trim messages to avoid context bloat (keep last 4000 tokens)
    trimmed_messages = trim_messages(
        messages,
        strategy="last",
        max_tokens=4000,
        include_system=True,
        allow_partial=False,
    )

    response = await model_with_tools.ainvoke(trimmed_messages)

    updates = {
        "messages": [response],
        # Increment step count to avoid infinite loops
        "step_count": state.get("step_count", 0) + 1,
    }

    # Extract reasoning if possible
    if hasattr(response, "content") and response.content:
        updates["coder_reasoning"] = response.content
        log.info(response.content, type="thought")

    if hasattr(response, "tool_calls") and response.tool_calls:
        tool_names = [tc["name"] for tc in response.tool_calls]
        log.info(f"Handing off to tools: {', '.join(tool_names)}", type="handoff")

    return updates
