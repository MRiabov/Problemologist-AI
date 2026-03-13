from controller.agent.benchmark.graph import define_graph
from controller.agent.graph import (
    cots_search_graph,
    electronics_planner_graph,
    engineer_planner_graph,
)
from controller.agent.graph import (
    graph as engineering_graph,
)
from shared.enums import AgentName
from shared.logging import get_logger

logger = get_logger(__name__)


def create_agent_graph(
    agent_name: AgentName = AgentName.ENGINEER_CODER,
    trace_id: str | None = None,
    session_id: str | None = None,
    start_node: AgentName | None = None,
):
    """
    Factory to create/return the appropriate LangGraph based on agent_name.
    Migrated from deepagents to LangGraph.
    """
    # WP10: Remove LangChain-based Langfuse callbacks

    if start_node is not None and agent_name in {
        AgentName.ENGINEER_PLANNER,
        AgentName.ENGINEER_CODER,
        AgentName.ENGINEER_PLAN_REVIEWER,
        AgentName.ELECTRONICS_PLANNER,
        AgentName.ELECTRONICS_REVIEWER,
        AgentName.ENGINEER_EXECUTION_REVIEWER,
    }:
        return engineering_graph, None

    if agent_name == AgentName.ENGINEER_PLANNER:
        return engineer_planner_graph, None

    if agent_name == AgentName.ELECTRONICS_PLANNER:
        return electronics_planner_graph, None

    if agent_name in [
        AgentName.ENGINEER_CODER,
        AgentName.ENGINEER_PLAN_REVIEWER,
        AgentName.ENGINEER_EXECUTION_REVIEWER,
        AgentName.ELECTRONICS_REVIEWER,
    ]:
        # Unified engineering graph (Architect -> Engineer -> Critic)
        return engineering_graph, None

    if agent_name in [
        AgentName.BENCHMARK_PLANNER,
        AgentName.BENCHMARK_PLAN_REVIEWER,
        AgentName.BENCHMARK_CODER,
        AgentName.BENCHMARK_REVIEWER,
    ]:
        # Unified benchmark generation graph (Planner -> Coder -> Reviewer)
        return define_graph(), None

    if agent_name == AgentName.COTS_SEARCH:
        return cots_search_graph, None

    logger.error(
        "unknown_agent_name_falling_back_to_engineer",
        agent_name=agent_name,
        session_id=session_id,
    )
    return engineering_graph, None
