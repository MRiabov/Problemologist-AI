from langchain_core.messages import SystemMessage, trim_messages

from src.agent.graph.state import AgentState
from src.agent.utils.config import Config
from src.agent.utils.logging import get_logger
from src.agent.utils.llm import get_model
from src.agent.utils.prompts import get_prompt

logger = get_logger(__name__)


async def reviewer_node(state: AgentState):
    """
    Analyzes the output of the tools (preview/submit) and decides next steps.
    """
    log = logger.bind(agent_role="Reviewer")
    log.info("Analyzing results and providing feedback")
    model = get_model(Config.LLM_MODEL)

    # We look at the last message, which should be a ToolMessage from the execution
    # or the sequence of messages leading up to it.

    system_prompt_key = "cad_agent.critic.system"
    system_prompt = get_prompt(system_prompt_key)
    system_prompt += (
        "\n\nIMPORTANT: You MUST provide your review in Markdown format with a YAML frontmatter block at the very beginning. "
        "The YAML block must contain two fields:\n"
        "1. `status`: one of 'pass', 'fail', or 'replan'. Use 'replan' if a major strategy shift is needed.\n"
        "2. `task_complete`: a boolean indicating if the original goal has been fully met.\n\n"
        "Format example:\n"
        "---\n"
        "status: pass\n"
        "task_complete: true\n"
        "---\n\n"
        "Review content follows here..."
    )

    messages = [SystemMessage(content=system_prompt)] + state["messages"]

    # Trim messages to avoid context bloat
    trimmed_messages = trim_messages(
        messages,
        strategy="last",
        max_tokens=4000,
        token_counter="approximate",
        include_system=True,
    )

    # We ask the LLM to review the situation
    response = await model.ainvoke(trimmed_messages)

    if hasattr(response, "content") and response.content:
        log.info(response.content, type="thought")
        log.info("Returning feedback to team.", type="handoff")

    return {"messages": [response]}
