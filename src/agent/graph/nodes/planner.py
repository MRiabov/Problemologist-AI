from langchain_core.messages import AIMessage, HumanMessage, SystemMessage

from src.agent.graph.state import AgentState
from src.agent.utils.config import Config
from src.agent.utils.llm import get_model
from src.agent.utils.prompts import get_prompt


def planner_node(state: AgentState):
    """
    Decides the high-level strategy and updates the plan.
    """
    # If a plan already exists, we might want to revise it or just keep it.
    # For this simple version, if we have a plan, we don't re-plan unless explicitly triggered (not covered here).
    if state.get("plan"):
        return {
            "messages": [
                AIMessage(content="Plan already exists. Proceeding to execution.")
            ]
        }

    model = get_model(Config.LLM_MODEL)

    # Get the original request from the first message
    original_request = ""
    for msg in state["messages"]:
        if isinstance(msg, HumanMessage) or (
            hasattr(msg, "type") and msg.type == "human"
        ):
            original_request = msg.content
            break

    system_prompt_key = "cad_agent.planner.system"
    system_prompt = get_prompt(system_prompt_key)

    # Check for overrides
    if (
        state.get("runtime_config")
        and "system_prompt_overrides" in state["runtime_config"]
    ):
        overrides = state["runtime_config"]["system_prompt_overrides"]
        if "planner" in overrides:
            system_prompt = get_prompt(overrides["planner"])
            # If the override is raw text (not dot-path), get_prompt returns default if not found,
            # but we assume overrides might be direct keys to prompts.yaml or even raw text?
            # get_prompt splits by dot. If we pass "benchmark_generator.planner", it works.
            # If we pass raw text, get_prompt returns default "" unless it happens to match a key.
            # Let's assume overrides are KEYS into prompts.yaml.

    messages = [
        SystemMessage(content=system_prompt),
        HumanMessage(
            content=f"Original Request: {original_request}\n\nPlease generate a technical plan."
        ),
    ]

    response = model.invoke(messages)

    return {"plan": response.content, "messages": [response]}
