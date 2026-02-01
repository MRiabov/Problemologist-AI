from langchain_core.messages import SystemMessage
from src.agent.graph.state import AgentState
from src.agent.utils.llm import get_model
from src.agent.utils.config import Config
from src.agent.tools.env import write_script, edit_script, preview_design, submit_design, search_docs
from src.agent.tools.memory import read_journal, write_journal

ACTOR_PROMPT = """You are the CAD Engineer (Actor).
Your goal is to execute the technical plan provided by the Planner.
You have access to a workspace where you can write and edit Python scripts using build123d.

Follow the plan step-by-step.
- Use 'write_script' to create new files.
- Use 'edit_script' to modify existing ones.
- Use 'search_docs' if you are unsure about syntax.
- Use 'preview_design' to check your geometry visually.
- Use 'submit_design' ONLY when you are confident the design is complete and meets requirements.

You can also read/write to the journal for memory.
"""

def actor_node(state: AgentState):
    """
    Executes the next step in the plan using tools.
    """
    model = get_model(Config.LLM_MODEL)
    
    # Bind tools to the model
    tools = [write_script, edit_script, preview_design, submit_design, search_docs, read_journal, write_journal]
    model_with_tools = model.bind_tools(tools)
    
    messages = [SystemMessage(content=ACTOR_PROMPT)] + state["messages"]
    
    # We might want to inject the plan as context if it's not the most recent message
    if state.get("plan"):
         messages.append(SystemMessage(content=f"Current Plan:\n{state['plan']}"))

    response = model_with_tools.invoke(messages)
    
    return {
        "messages": [response],
        # Increment step count to avoid infinite loops
        "step_count": state.get("step_count", 0) + 1
    }