from langchain_core.messages import HumanMessage, AIMessage
from src.agent.graph.state import AgentState
from src.agent.utils.llm import get_model
from src.agent.utils.config import Config
from src.agent.tools.memory import read_journal


PLANNING_PROMPT = """You are the Planner for the VLM CAD Agent. Your goal is to break down a mechanical engineering design problem into a sequence of actionable steps.

Context from previous sessions:
{journal}

Current User Request:
{user_input}

Generate a high-level implementation plan. Focus on:
1. Geometry creation (CSG or B-Rep).
2. Material and physical property assignment.
3. Simulation setup (MuJoCo).
4. Validation and iteration.

Return ONLY the plan as a numbered list of steps."""


def planner_node(state: AgentState):
    """
    The Planner node that generates or updates the high-level implementation plan.
    """
    # Only generate a plan if it's currently empty
    if state.get("plan"):
        return state

    # Retrieve context from the journal
    journal_context = read_journal.invoke({})

    # Get the original user input (first human message)
    user_input = ""
    for msg in state["messages"]:
        if isinstance(msg, HumanMessage):
            user_input = msg.content
            break

    if not user_input:
        # Fallback if no human message found (should not happen in normal flow)
        user_input = "No user input provided."

    # Format the prompt
    prompt = PLANNING_PROMPT.format(
        journal=journal_context,
        user_input=user_input
    )

    # Call the LLM
    model = get_model(Config.LLM_MODEL, Config.TEMPERATURE)
    response = model.invoke([HumanMessage(content=prompt)])

    plan = response.content

    # Update state
    return {
        "plan": plan,
        "messages": [AIMessage(content=f"Generated plan:\n{plan}")]
    }
