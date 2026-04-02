from __future__ import annotations

from pathlib import Path

from shared.enums import AgentName

BENCHMARK_SCRIPT_PATH = "benchmark_script.py"
BENCHMARK_PLAN_EVIDENCE_SCRIPT_PATH = "benchmark_plan_evidence_script.py"
BENCHMARK_PLAN_TECHNICAL_DRAWING_SCRIPT_PATH = (
    "benchmark_plan_technical_drawing_script.py"
)
SOLUTION_SCRIPT_PATH = "solution_script.py"
SOLUTION_PLAN_EVIDENCE_SCRIPT_PATH = "solution_plan_evidence_script.py"
SOLUTION_PLAN_TECHNICAL_DRAWING_SCRIPT_PATH = (
    "solution_plan_technical_drawing_script.py"
)
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


def _as_path(script_path: str) -> Path:
    return Path(script_path)


def authored_script_path_for_agent(agent_name: AgentName | str | None) -> Path:
    normalized = _normalize_agent_name(agent_name)
    if normalized in {
        AgentName.BENCHMARK_PLANNER,
        AgentName.BENCHMARK_PLAN_REVIEWER,
        AgentName.BENCHMARK_CODER,
        AgentName.BENCHMARK_REVIEWER,
    }:
        return _as_path(BENCHMARK_SCRIPT_PATH)
    if normalized in {
        AgentName.ENGINEER_CODER,
        AgentName.ENGINEER_EXECUTION_REVIEWER,
        AgentName.ELECTRONICS_REVIEWER,
        AgentName.ENGINEER_PLANNER,
        AgentName.ENGINEER_PLAN_REVIEWER,
        AgentName.ELECTRONICS_PLANNER,
    }:
        return _as_path(SOLUTION_SCRIPT_PATH)
    return _as_path(LEGACY_SCRIPT_PATH)


def technical_drawing_script_path_for_agent(
    agent_name: AgentName | str | None,
) -> Path:
    normalized = _normalize_agent_name(agent_name)
    if normalized in {
        AgentName.BENCHMARK_PLANNER,
        AgentName.BENCHMARK_PLAN_REVIEWER,
        AgentName.BENCHMARK_CODER,
        AgentName.BENCHMARK_REVIEWER,
    }:
        return _as_path(BENCHMARK_PLAN_TECHNICAL_DRAWING_SCRIPT_PATH)
    if normalized in {
        AgentName.ENGINEER_PLANNER,
        AgentName.ENGINEER_PLAN_REVIEWER,
        AgentName.ENGINEER_CODER,
        AgentName.ENGINEER_EXECUTION_REVIEWER,
        AgentName.ELECTRONICS_PLANNER,
        AgentName.ELECTRONICS_REVIEWER,
    }:
        return _as_path(SOLUTION_PLAN_TECHNICAL_DRAWING_SCRIPT_PATH)
    return authored_script_path_for_agent(agent_name)


def authored_script_path_for_reviewer_stage(reviewer_stage: str | None) -> Path:
    stage = (reviewer_stage or "").strip()
    if stage == "benchmark_reviewer":
        return _as_path(BENCHMARK_SCRIPT_PATH)
    if stage in {"engineering_execution_reviewer", "electronics_reviewer"}:
        return _as_path(SOLUTION_SCRIPT_PATH)
    return _as_path(LEGACY_SCRIPT_PATH)
