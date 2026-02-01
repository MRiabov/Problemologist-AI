from langchain_core.messages import SystemMessage
from src.agent.graph.state import AgentState
from src.agent.utils.llm import get_model
from src.agent.utils.config import Config
from src.agent.tools.env import write_script, edit_script, preview_design, submit_design, search_docs
from src.agent.tools.memory import read_journal, write_journal

TOOLS = [
    write_script, 
    edit_script, 
    preview_design, 
    submit_design, 
    search_docs, 
    read_journal, 
    write_journal
]

ACTOR_SYSTEM_PROMPT = """You are the Actor for the VLM CAD Agent. Your goal is to execute the current plan by calling the available tools.

Current Plan:
{plan}

Use the available tools to implement the geometry, material properties, and simulation setup. 
If you need more information, use `search_docs` or `read_journal`.
When you have implemented a part of the design, use `preview_design` to check it.
When you are ready to finalize, use `submit_design`.

Always provide a brief thought process before calling a tool."""


def actor_node(state: AgentState):
    """
    The Actor node that decides which tool to call based on the plan and history.
    """
    model = get_model(Config.LLM_MODEL, Config.TEMPERATURE)
    model_with_tools = model.bind_tools(TOOLS)
    
    # Prepare the system message with the current plan
    system_msg = SystemMessage(content=ACTOR_SYSTEM_PROMPT.format(plan=state["plan"]))
    
    # Invoke the model with the full message history
    # LangGraph's add_messages will handle merging this into the state
    response = model_with_tools.invoke([system_msg] + state["messages"])
    
    return {
        "messages": [response],
        "step_count": state.get("step_count", 0) + 1
    }
