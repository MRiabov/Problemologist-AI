from langchain_core.messages import SystemMessage, HumanMessage
from src.agent.graph.state import AgentState
from src.agent.utils.llm import get_model
from src.agent.utils.config import Config
from src.agent.utils.prompts import get_prompt


def critic_node(state: AgentState):
    """
    Analyzes the output of the tools (preview/submit) and decides next steps.
    """
    model = get_model(Config.LLM_MODEL)

    # We look at the last message, which should be a ToolMessage from the execution
    # or the sequence of messages leading up to it.

    messages = [SystemMessage(content=get_prompt("cad_agent.critic.system"))] + state[
        "messages"
    ]

    # We ask the LLM to review the situation
    response = model.invoke(messages)

    return {"messages": [response]}
