# from langchain_core.messages import SystemMessage
# from src.agent.graph.state import AgentState
# from src.agent.utils.config import Config
# from src.agent.utils.llm import get_model
# from src.agent.utils.prompts import get_prompt

# def planner_critic_node(state: AgentState):
#     """
#     (INACTIVE) Evaluates the Planner's plan for realism and budget adherence.
#     """
#     model = get_model(Config.LLM_MODEL)
#     system_prompt = get_prompt("cad_agent.planner_critic.system")
    
#     # We evaluate the 'plan' against the task description
#     # (Implementation details would go here)
    
#     # For now, it just passes through
#     return {"messages": []}
