from controller.agent.benchmark.graph import define_graph
from controller.agent.graph import graph as engineering_graph
from controller.config.settings import settings
from shared.cots.agent import create_cots_search_agent
from shared.logging import get_logger

logger = get_logger(__name__)


def create_agent_graph(
    agent_name: str = "engineer_coder",
    trace_id: str | None = None,
    session_id: str | None = None,
):
    """
    Factory to create/return the appropriate LangGraph based on agent_name.
    Migrated from deepagents to LangGraph.
    """
    # WP10: Remove LangChain-based Langfuse callbacks

    if agent_name.startswith("engineer") or agent_name == "electronics_engineer":
        # Unified engineering graph (Architect -> Engineer -> Critic)
        return engineering_graph, None

    if agent_name.startswith("benchmark"):
        # Unified benchmark generation graph (Planner -> Coder -> Reviewer)
        return define_graph(), None

    if agent_name == "cots_search":
        # Specialized COTS search agent
        return create_cots_search_agent(settings.llm_model), None

    logger.warning("unknown_agent_name_falling_back_to_engineer", agent_name=agent_name)
    return engineering_graph, None
