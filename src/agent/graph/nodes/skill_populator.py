import asyncio
from langchain_core.messages import HumanMessage, SystemMessage
from langchain_core.runnables import RunnableConfig
from langchain_core.tools import StructuredTool

from src.agent.graph.state import AgentState
from src.agent.utils.config import Config
from src.agent.utils.env_log import log_to_env
from src.agent.utils.llm import get_model


async def skill_populator_node(state: AgentState, config: RunnableConfig):
    """
    Populates the skill with insights after a definitive failure or timeout.
    """
    runtime = config.get("configurable", {}).get("runtime")
    log_to_env(
        "Analyzing failure and populating skills with lessons learned...",
        agent_role="SkillPopulator",
        runtime=runtime,
    )

    # Check if we actually failed (e.g., step_count > MAX_STEPS or explicit failure)
    # For now, we assume this node is only reached on failure path.

    async def update_skill_func(
        skill_name: str,
        content: str,
        filename: str = "SKILL.md",
        resource_type: str | None = None,
    ) -> str:
        """
        Updates or adds information to a specialized skill folder.
        """
        return await asyncio.to_thread(
            runtime.update_skill, skill_name, content, filename, resource_type
        )

    update_skill = StructuredTool.from_function(update_skill_func, name="update_skill")
    model = get_model(Config.LLM_MODEL).bind_tools([update_skill])

    # Extract conversation summary or last few messages to understand failure
    messages = state["messages"]
    # Filter for relevant messages (e.g., tool outputs with errors)
    summary = []
    for msg in messages[-10:]:  # Look at last 10 messages
        if hasattr(msg, "content") and msg.content:
            summary.append(f"{type(msg).__name__}: {msg.content[:200]}...")

    summary_str = "\n".join(summary)

    prompt = (
        "The CAD agent has failed to complete the task within the allotted steps.\n"
        f"Execution Summary (last steps):\n{summary_str}\n\n"
        "Your task is to analyze why the agent struggled and update the 'build123d_cad_drafting_skill' "
        "with new insights, common pitfalls to avoid, or better patterns. "
        "Use the `update_skill` tool to add a new reference file (e.g., 'lessons_learned_cad.md') "
        "to the skill, describing the issue and the solution or workaround."
    )

    messages = [
        SystemMessage(content=prompt),
        HumanMessage(content="Populate the skill with lessons learned."),
    ]
    await model.ainvoke(messages)

    return {}
