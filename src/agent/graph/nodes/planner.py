from langchain_core.messages import AIMessage, HumanMessage, SystemMessage

from src.agent.graph.state import AgentState
from src.agent.tools.env_adapter import set_current_role
from src.agent.utils.config import Config
from src.agent.utils.env_log import log_to_env
from src.agent.utils.prompts import get_prompt
from src.agent.utils.llm import get_model
from src.agent.tools.env_adapter import start_session_async


async def planner_node(state: AgentState):
    """
    Decides the high-level strategy and updates the plan.
    """
    set_current_role("Planner")
    log_to_env("Planning high-level strategy...", agent_role="Planner")

    # Start persistent sandbox session if not already active
    # This ensures follow-up run_command calls will work
    await start_session_async("vlm-cad-session")

    # Check if we need to re-plan based on Critic feedback
    needs_replan = False
    last_critic_msg = ""
    for msg in reversed(state["messages"]):
        if hasattr(msg, "content") and any(
            x in msg.content for x in ["Replan", "budget", "cost", "HARD_LIMIT"]
        ):
            needs_replan = True
            last_critic_msg = msg.content
            break

    # If a plan already exists and no re-plan is triggered, proceed.
    if state.get("plan") and not needs_replan:
        log_to_env(
            "Existing plan is valid. Handing off to Actor.",
            type="handoff",
            agent_role="Planner",
        )
        return {
            "messages": [
                AIMessage(
                    content="Plan already exists and is valid. Proceeding to execution."
                )
            ]
        }

    if needs_replan:
        log_to_env(
            f"Critic requested re-plan: {last_critic_msg[:100]}...",
            agent_role="Planner",
        )

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

    # Mandatory skill check instruction
    system_prompt += (
        "\n\nMANDATORY: Before planning any `build123d` implementation, "
        "you MUST use `view_file` to read the documentation at: "
        "`docs/skills/build123d_cad_drafting_skill/SKILL.md` "
        "and `docs/skills/manufacturing-knowledge/SKILL.md`. These contain expert knowledge, "
        "curated patterns, and critical pitfalls."
    )

    # Check for overrides
    if (
        state.get("runtime_config")
        and "system_prompt_overrides" in state["runtime_config"]
    ):
        overrides = state["runtime_config"]["system_prompt_overrides"]
        if "planner" in overrides:
            system_prompt = get_prompt(overrides["planner"])
            # If the override is raw text (not dot-path), get_prompt returns default if not found,
            # but we assume overrides might be direct keys to config/prompts.yaml or even raw text?
            # get_prompt splits by dot. If we pass "benchmark_generator.planner", it works.
            # If we pass raw text, get_prompt returns default "" unless it happens to match a key.
            # Let's assume overrides are KEYS into config/prompts.yaml.

    messages = [
        SystemMessage(content=system_prompt),
        HumanMessage(
            content=f"Original Request: {original_request}\n\nPlease generate a technical plan."
        ),
    ]

    response = await model.ainvoke(messages)

    log_to_env(
        f"Strategy developed:\n{response.content}", type="thought", agent_role="Planner"
    )
    log_to_env(
        "Handing off to Actor for execution.", type="handoff", agent_role="Planner"
    )

    return {"plan": response.content, "messages": [response]}
