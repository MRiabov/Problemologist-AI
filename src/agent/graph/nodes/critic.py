from langchain_core.messages import SystemMessage

from src.agent.graph.state import AgentState
from src.agent.utils.config import Config
from src.agent.utils.logging import get_logger
from src.agent.utils.llm import get_model
from src.agent.utils.prompts import get_prompt

logger = get_logger(__name__)


async def critic_node(state: AgentState):
    """
    Analyzes the output of the tools (preview/submit) and decides next steps.
    """
    log = logger.bind(agent_role="Critic")
    log.info("Analyzing results and providing feedback")
    model = get_model(Config.LLM_MODEL)

    # We look at the last message, which should be a ToolMessage from the execution
    # or the sequence of messages leading up to it.

    system_prompt_key = "cad_agent.critic.system"
    system_prompt = get_prompt(system_prompt_key)
    system_prompt += "\n\nIMPORTANT: If the design failed validation or exceeded the budget, you MUST include '[REPLAN]' in your feedback to trigger a strategy update."

    if (
        state.get("runtime_config")
        and "system_prompt_overrides" in state["runtime_config"]
    ):
        overrides = state["runtime_config"]["system_prompt_overrides"]
        if "critic" in overrides:
            system_prompt = get_prompt(overrides["critic"])

    messages = [SystemMessage(content=system_prompt)] + state["messages"]

    # We ask the LLM to review the situation
    response = await model.ainvoke(messages)

    if hasattr(response, "content") and response.content:
        log.info(response.content, type="thought")
        log.info("Returning feedback to team.", type="handoff")

    return {"messages": [response]}
