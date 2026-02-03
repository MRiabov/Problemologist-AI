from langchain_core.messages import HumanMessage, SystemMessage
from src.agent.graph.state import AgentState
from src.agent.utils.config import Config
from src.agent.utils.llm import get_model
from src.agent.utils.prompts import get_prompt

async def planner_critic_node(state: AgentState):
    """
    Evaluates the Planner's plan for realism and budget adherence.
    """
    model = get_model(Config.LLM_MODEL)
    system_prompt = get_prompt("cad_agent.planner_critic.system")
    
    plan = state.get("plan", "")
    
    # Get the original request from the first message
    original_request = ""
    for msg in state["messages"]:
        if isinstance(msg, HumanMessage) or (
            hasattr(msg, "type") and msg.type == "human"
        ):
            original_request = msg.content
            break

    content = f"Original Request: {original_request}\n\nProposed Plan:\n{plan}"

    messages = [
        SystemMessage(content=system_prompt),
        HumanMessage(content=content)
    ]

    response = await model.ainvoke(messages)

    # Update scratchpad for loop detection
    scratchpad = state.get("scratchpad", {}).copy()
    attempts = scratchpad.get("planning_attempts", 0)
    scratchpad["planning_attempts"] = attempts + 1

    return {"messages": [response], "scratchpad": scratchpad}
