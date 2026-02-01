from langchain_core.messages import SystemMessage, HumanMessage
from src.agent.graph.state import AgentState
from src.agent.utils.llm import get_model
from src.agent.utils.config import Config

CRITIC_PROMPT = """You are the Design Reviewer (Critic).
Your job is to evaluate the results of the recent design action (Preview or Submission).

1. If this is a PREVIEW:
   - Check if the visual feedback (if described) looks correct.
   - If there are errors, suggest fixes.
   - If it looks good, encourage the Actor to proceed or submit.

2. If this is a SUBMISSION:
   - Check the validation score/report.
   - If successful, celebrate and finalize the task.
   - If failed, analyze the failure reason and explicitly update the plan to fix it.

Output your feedback clearly.
"""

def critic_node(state: AgentState):
    """
    Analyzes the output of the tools (preview/submit) and decides next steps.
    """
    model = get_model(Config.LLM_MODEL)
    
    # We look at the last message, which should be a ToolMessage from the execution
    # or the sequence of messages leading up to it.
    
    messages = [SystemMessage(content=CRITIC_PROMPT)] + state["messages"]
    
    # We ask the LLM to review the situation
    response = model.invoke(messages)
    
    return {
        "messages": [response]
    }