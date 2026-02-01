from langchain_core.messages import SystemMessage
from src.agent.graph.state import AgentState
from src.agent.utils.llm import get_model
from src.agent.utils.config import Config
from src.agent.utils.prompts import get_prompt
from src.agent.tools.env import write_script, edit_script, preview_design, submit_design, search_docs
from src.agent.tools.memory import read_journal, write_journal

def actor_node(state: AgentState):
    """
    Executes the next step in the plan using tools.
    """
    model = get_model(Config.LLM_MODEL)
    
    # Bind tools to the model
    tools = [write_script, edit_script, preview_design, submit_design, search_docs, read_journal, write_journal]
    model_with_tools = model.bind_tools(tools)
    
    messages = [SystemMessage(content=get_prompt("cad_agent.actor.system"))] + state["messages"]
    
    # We might want to inject the plan as context if it's not the most recent message
    if state.get("plan"):
         messages.append(SystemMessage(content=f"Current Plan:\n{state['plan']}"))

    response = model_with_tools.invoke(messages)
    
    return {
        "messages": [response],
        # Increment step count to avoid infinite loops
        "step_count": state.get("step_count", 0) + 1
    }