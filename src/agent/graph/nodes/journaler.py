from langchain_core.messages import SystemMessage

from src.agent.graph.state import AgentState
from src.agent.tools.env_adapter import set_current_role
from src.agent.tools.memory import write_journal
from src.agent.utils.config import Config
from src.agent.utils.env_log import log_to_env
from src.agent.utils.llm import get_model


async def journaler_node(state: AgentState):
    """
    Analyzes the successful execution and logs insights to the journal.
    """
    set_current_role("Journaler")
    log_to_env(
        "Task completed successfully. Analyzing for journal insights...",
        agent_role="Journaler",
    )

    model = get_model(Config.LLM_MODEL).bind_tools([write_journal])

    prompt = (
        "The CAD agent has successfully completed the task.\n"
        "Your job is to analyze the execution history and identify key insights, successful patterns, or reusable techniques.\n"
        "Document these in the journal using the `write_journal` tool.\n"
        "Focus on: \n"
        "1. Unique geometric challenges and how they were solved.\n"
        "2. Effective use of specific build123d operations.\n"
        "3. Any 'aha!' moments or corrections that led to success.\n"
        "If the task was trivial, you can write a brief entry noting the successful completion."
    )

    # Use a limited context to avoid context length issues, or rely on the last summary.
    # We'll pass the system prompt + messages.
    messages = [SystemMessage(content=prompt)] + state["messages"]

    response = await model.ainvoke(messages)

    if hasattr(response, "tool_calls") and response.tool_calls:
        for tool_call in response.tool_calls:
            if tool_call["name"] == "write_journal":
                # Execute the tool directly
                try:
                    write_journal.invoke(tool_call["args"])
                    log_to_env("Journal entry written.", agent_role="Journaler")
                except Exception as e:
                    log_to_env(f"Failed to write journal: {e}", agent_role="Journaler", type="error")

    return {}
