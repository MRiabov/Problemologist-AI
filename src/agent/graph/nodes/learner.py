from langchain_core.messages import HumanMessage, SystemMessage

from src.agent.graph.state import AgentState
from src.agent.tools.env_adapter import write_file
from src.agent.utils.config import Config
from src.agent.utils.env_log import log_to_runtime
from src.agent.utils.llm import get_model


async def learner_node(state: AgentState):
    """
    Populates the skill with insights after a definitive failure or timeout.
    """
    log_to_runtime(
        "Analyzing failure and populating skills with lessons learned...",
        agent_role="Learner",
    )

    # Check if we actually failed (e.g., step_count > MAX_STEPS or explicit failure)
    # For now, we assume this node is only reached on failure path.

    model = get_model(Config.LLM_MODEL).bind_tools([write_file])

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
        "Use the `write_file` tool to add a new reference file (e.g., 'docs/skills/build123d_cad_drafting_skill/lessons_learned_cad.md') "
        "to the skill, describing the issue and the solution or workaround."
    )

    messages = [
        SystemMessage(content=prompt),
        HumanMessage(content="Populate the skill with lessons learned."),
    ]
    await model.ainvoke(messages)

    return {}
