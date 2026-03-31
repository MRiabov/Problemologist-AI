from __future__ import annotations

from shared.enums import AgentName

BENCHMARK_SCRIPT_PATH = "benchmark_script.py"
SOLUTION_SCRIPT_PATH = "solution_script.py"
LEGACY_SCRIPT_PATH = "script.py"


def _normalize_agent_name(agent_name: AgentName | str | None) -> AgentName | None:
    if agent_name is None:
        return None
    if isinstance(agent_name, AgentName):
        return agent_name
    try:
        return AgentName(str(agent_name))
    except ValueError:
        return None


def authored_script_path_for_agent(agent_name: AgentName | str | None) -> str:
    normalized = _normalize_agent_name(agent_name)
    if normalized in {
        AgentName.BENCHMARK_CODER,
        AgentName.BENCHMARK_REVIEWER,
    }:
        return BENCHMARK_SCRIPT_PATH
    if normalized in {
        AgentName.ENGINEER_CODER,
        AgentName.ENGINEER_EXECUTION_REVIEWER,
        AgentName.ELECTRONICS_REVIEWER,
        AgentName.ENGINEER_PLANNER,
        AgentName.ENGINEER_PLAN_REVIEWER,
        AgentName.ELECTRONICS_PLANNER,
    }:
        return SOLUTION_SCRIPT_PATH
    return LEGACY_SCRIPT_PATH


def authored_script_path_for_reviewer_stage(reviewer_stage: str | None) -> str:
    stage = (reviewer_stage or "").strip()
    if stage == "benchmark_reviewer":
        return BENCHMARK_SCRIPT_PATH
    if stage in {"engineering_execution_reviewer", "electronics_reviewer"}:
        return SOLUTION_SCRIPT_PATH
    return LEGACY_SCRIPT_PATH
