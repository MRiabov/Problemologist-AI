from langchain_core.messages import AIMessage, HumanMessage, SystemMessage

from src.agent.graph.state import AgentState
from src.agent.tools.env_adapter import set_current_role
from src.agent.utils.config import Config
from src.agent.utils.logging import get_logger
from src.agent.utils.prompts import get_prompt
from src.agent.utils.llm import get_model

logger = get_logger(__name__)

async def planner_node(state: AgentState):
    """
    Decides the high-level strategy and updates the plan.
    """
    log = logger.bind(agent_role="Planner")
    log.info("Planning high-level strategy")

    # Check if we need to re-plan based on Critic feedback
    needs_replan = False
    last_critic_msg = ""
    # Look for explicit re-plan signals or budget/cost issues in the conversation
    for msg in reversed(state["messages"]):
        if hasattr(msg, "content") and any(
            x.lower() in msg.content.lower() for x in ["[REPLAN]", "REPLAN_REQUIRED", "HARD_LIMIT", "budget exceeded"]
        ):
            needs_replan = True
            last_critic_msg = msg.content
            break
        # Also check for general budget/cost mentions in Critic messages
        if getattr(msg, "name", "") == "Critic" or (hasattr(msg, "type") and msg.type == "ai" and "Critic" in str(msg)):
             if any(x in msg.content for x in ["budget", "cost"]):
                 needs_replan = True
                 last_critic_msg = msg.content
                 break

    # If a plan already exists and no re-plan is triggered, proceed.
    if state.get("plan") and not needs_replan:
        log.info("Existing plan is valid. Handing off to Actor.", type="handoff")
        return {
            "messages": [
                AIMessage(
                    content="Plan already exists and is valid. Proceeding to execution."
                )
            ]
        }

    if needs_replan:
        log.info("Critic requested re-plan", critic_msg=last_critic_msg[:100])

    model = get_model(Config.LLM_MODEL)

    # Aggregate all human messages for full context
    human_messages = []
    for msg in state["messages"]:
        if isinstance(msg, HumanMessage) or (
            hasattr(msg, "type") and msg.type == "human"
        ):
            human_messages.append(msg.content)
    
    full_request = "\n---\n".join(human_messages)

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

    planning_context = f"Original/Updated Request:\n{full_request}"
    if last_critic_msg:
        planning_context += f"\n\nCritic Feedback triggering Re-plan:\n{last_critic_msg}"

    messages = [
        SystemMessage(content=system_prompt),
        HumanMessage(
            content=f"{planning_context}\n\nPlease generate a technical plan."
        ),
    ]

    response = await model.ainvoke(messages)

    log.info("Strategy developed", strategy=response.content, type="thought")
    log.info("Handing off to Actor for execution.", type="handoff")

    return {"plan": response.content, "messages": [response]}
