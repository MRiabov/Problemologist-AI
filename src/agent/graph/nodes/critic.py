from langchain_core.messages import SystemMessage, ToolMessage
from src.agent.graph.state import AgentState
from src.agent.utils.llm import get_model
from src.agent.utils.config import Config
from src.agent.tools.memory import write_journal

CRITIC_SYSTEM_PROMPT = """You are the Critic for the VLM CAD Agent. Your goal is to evaluate the results of a design preview or submission.

Current Plan:
{plan}

Analyze the latest tool output (preview or submission results) and decide if the design is successful.

1. If SUCCESSFUL: 
   - Use `write_journal` to record the final design and achievements.
   - Summarize the final result.
2. If FAILURE/NEEDS IMPROVEMENT:
   - Identify specifically what went wrong.
   - Provide an updated plan to fix the issues.
   - The updated plan should be a numbered list.

Always provide your critical analysis before deciding on success or failure."""


def critic_node(state: AgentState):
    """
    The Critic node that validates results and decides on the next steps.
    """
    model = get_model(Config.LLM_MODEL, Config.TEMPERATURE)
    # Critic might need to write to the journal
    model_with_tools = model.bind_tools([write_journal])
    
    # Find the latest tool output related to preview or submission
    latest_output = "No tool output found."
    for msg in reversed(state["messages"]):
        if isinstance(msg, ToolMessage):
            latest_output = msg.content
            break
            
    system_msg = SystemMessage(content=CRITIC_SYSTEM_PROMPT.format(
        plan=state["plan"],
        tool_output=latest_output
    ))
    
    response = model_with_tools.invoke([system_msg] + state["messages"])
    
    # Logic to detect if we should update the plan
    # For now, we'll just append the response to messages. 
    # In a more advanced version, we could parse the response to update state["plan"]
    
    return {
        "messages": [response]
    }
