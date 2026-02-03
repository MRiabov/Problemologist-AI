from langchain_core.messages import SystemMessage
from langchain_core.runnables import RunnableConfig
from typing import Optional
from src.agent.utils.llm import get_model

from src.agent.graph.state import AgentState
from src.agent.utils.config import Config
from src.agent.utils.env_log import log_to_env
from src.agent.utils.prompts import get_prompt


async def actor_node(
    state: AgentState, config: RunnableConfig, tools: Optional[list] = None
):
    """
    Executes the next step in the plan using tools.
    """
    runtime = config.get("configurable", {}).get("runtime")
    log_to_env("Executing current step...", agent_role="Actor", runtime=runtime)
    model = get_model(Config.LLM_MODEL)

    # Bind tools to the model
    if tools is None:
        raise ValueError("Tools must be provided to actor_node")

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

    # Mandatory skill check instruction
    system_prompt += get_prompt("cad_agent.actor.mandatory_instruction")

    if state.get("step_count", 0) > 5:
        system_prompt += get_prompt("cad_agent.actor.critical_warning")

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
        log_to_env(
            response.content, type="thought", agent_role="Actor", runtime=runtime
        )

    if hasattr(response, "tool_calls") and response.tool_calls:
        tool_names = [tc["name"] for tc in response.tool_calls]
        log_to_env(
            f"Handing off to tools: {', '.join(tool_names)}",
            type="handoff",
            agent_role="Actor",
            runtime=runtime,
        )

    return updates
