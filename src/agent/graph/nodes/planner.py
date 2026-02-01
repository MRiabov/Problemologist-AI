from langchain_core.messages import SystemMessage, HumanMessage, AIMessage
from src.agent.graph.state import AgentState
from src.agent.utils.llm import get_model
from src.agent.utils.config import Config

PLANNING_PROMPT = """You are the Lead Mechanical Engineer (Planner).
Your goal is to decompose the user's design request into a structured step-by-step technical plan.
You must use build123d for CAD and consider physics constraints (MuJoCo).

Reference the 'journal.md' if it exists to understand previous attempts and failures.

Output only the plan in a clear, numbered list format.
"""

def planner_node(state: AgentState):
    """
    Decides the high-level strategy and updates the plan.
    """
    # If a plan already exists, we might want to revise it or just keep it.
    # For this simple version, if we have a plan, we don't re-plan unless explicitly triggered (not covered here).
    if state.get("plan"):
       return {"messages": [AIMessage(content="Plan already exists. Proceeding to execution.")]}

    model = get_model(Config.LLM_MODEL)
    
    # Get the original request from the first message
    original_request = ""
    for msg in state["messages"]:
        if isinstance(msg, HumanMessage) or (hasattr(msg, "type") and msg.type == "human"):
            original_request = msg.content
            break
            
    messages = [
        SystemMessage(content=PLANNING_PROMPT),
        HumanMessage(content=f"Original Request: {original_request}\n\nPlease generate a technical plan.")
    ]
    
    response = model.invoke(messages)
    
    return {
        "plan": response.content,
        "messages": [response]
    }