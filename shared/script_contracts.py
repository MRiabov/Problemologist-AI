from __future__ import annotations

from pathlib import Path

from shared.enums import AgentName

BENCHMARK_SCRIPT_PATH = "benchmark_script.py"
BENCHMARK_PLAN_PATH = "benchmark_plan.md"
BENCHMARK_PLAN_EVIDENCE_SCRIPT_PATH = "benchmark_plan_evidence_script.py"
BENCHMARK_PLAN_TECHNICAL_DRAWING_SCRIPT_PATH = (
    "benchmark_plan_technical_drawing_script.py"
)
PAYLOAD_TRAJECTORY_DEFINITION_PATH = "payload_trajectory_definition.yaml"
# Compatibility alias for older imports during the rename rollout.
PRECISE_PATH_DEFINITION_PATH = PAYLOAD_TRAJECTORY_DEFINITION_PATH
SOLUTION_SCRIPT_PATH = "solution_script.py"
ENGINEERING_PLAN_PATH = "engineering_plan.md"
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


def plan_path_for_agent(agent_name: AgentName | str | None) -> Path:
    normalized = _normalize_agent_name(agent_name)
    if normalized in {
        AgentName.BENCHMARK_PLANNER,
        AgentName.BENCHMARK_PLAN_REVIEWER,
        AgentName.BENCHMARK_CODER,
        AgentName.BENCHMARK_REVIEWER,
    }:
        return _as_path(BENCHMARK_PLAN_PATH)
    if normalized in {
        AgentName.ENGINEER_CODER,
        AgentName.ENGINEER_EXECUTION_REVIEWER,
        AgentName.ELECTRONICS_REVIEWER,
        AgentName.ENGINEER_PLANNER,
        AgentName.ENGINEER_PLAN_REVIEWER,
        AgentName.ELECTRONICS_PLANNER,
    }:
        return _as_path(ENGINEERING_PLAN_PATH)
    return _as_path("plan.md")


def plan_artifact_candidates_for_agent(
    agent_name: AgentName | str | None,
) -> tuple[str, ...]:
    """Return accepted plan-file names for the requested role.

    Historical benchmark and engineer bundles may still carry the legacy
    ``plan.md`` filename, while newly materialized workspaces use the role-
    specific plan filename. Validation should accept either spelling for the
    same role, but not cross-role plan files.
    """
    canonical = plan_path_for_agent(agent_name).as_posix()
    if canonical == "plan.md":
        return (canonical,)
    return (canonical, "plan.md")


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


def drafting_script_paths_for_agent(
    agent_name: AgentName | str | None,
) -> tuple[Path, Path]:
    normalized = _normalize_agent_name(agent_name)
    if normalized in {
        AgentName.BENCHMARK_PLANNER,
        AgentName.BENCHMARK_PLAN_REVIEWER,
        AgentName.BENCHMARK_CODER,
        AgentName.BENCHMARK_REVIEWER,
    }:
        return (
            _as_path(BENCHMARK_PLAN_EVIDENCE_SCRIPT_PATH),
            _as_path(BENCHMARK_PLAN_TECHNICAL_DRAWING_SCRIPT_PATH),
        )
    if normalized in {
        AgentName.ENGINEER_PLANNER,
        AgentName.ENGINEER_PLAN_REVIEWER,
        AgentName.ENGINEER_CODER,
        AgentName.ENGINEER_EXECUTION_REVIEWER,
        AgentName.ELECTRONICS_PLANNER,
        AgentName.ELECTRONICS_REVIEWER,
    }:
        return (
            _as_path(SOLUTION_PLAN_EVIDENCE_SCRIPT_PATH),
            _as_path(SOLUTION_PLAN_TECHNICAL_DRAWING_SCRIPT_PATH),
        )
    return (_as_path(LEGACY_SCRIPT_PATH), _as_path(LEGACY_SCRIPT_PATH))


def planner_role_for_drafting_script_path(
    script_path: str | Path | None,
) -> AgentName | None:
    if script_path is None:
        return None

    script_name = Path(script_path).name
    if script_name in {
        BENCHMARK_PLAN_EVIDENCE_SCRIPT_PATH,
        BENCHMARK_PLAN_TECHNICAL_DRAWING_SCRIPT_PATH,
    }:
        return AgentName.BENCHMARK_PLANNER
    if script_name in {
        SOLUTION_PLAN_EVIDENCE_SCRIPT_PATH,
        SOLUTION_PLAN_TECHNICAL_DRAWING_SCRIPT_PATH,
    }:
        return AgentName.ENGINEER_PLANNER
    return None


def drafting_render_manifest_path_for_agent(
    agent_name: AgentName | str | None,
) -> Path:
    normalized = _normalize_agent_name(agent_name)
    if normalized in {
        AgentName.BENCHMARK_PLANNER,
        AgentName.BENCHMARK_PLAN_REVIEWER,
        AgentName.BENCHMARK_CODER,
        AgentName.BENCHMARK_REVIEWER,
    }:
        return Path("renders/benchmark_renders/render_manifest.json")
    if normalized in {
        AgentName.ENGINEER_PLANNER,
        AgentName.ENGINEER_PLAN_REVIEWER,
        AgentName.ENGINEER_CODER,
        AgentName.ENGINEER_EXECUTION_REVIEWER,
        AgentName.ELECTRONICS_PLANNER,
        AgentName.ELECTRONICS_REVIEWER,
    }:
        return Path("renders/engineer_plan_renders/render_manifest.json")
    return Path("renders/render_manifest.json")


def authored_script_path_for_reviewer_stage(reviewer_stage: str | None) -> Path:
    stage = (reviewer_stage or "").strip()
    if stage == "benchmark_reviewer":
        return _as_path(BENCHMARK_SCRIPT_PATH)
    if stage in {"engineering_execution_reviewer", "electronics_reviewer"}:
        return _as_path(SOLUTION_SCRIPT_PATH)
    return _as_path(LEGACY_SCRIPT_PATH)


def plan_path_for_reviewer_stage(reviewer_stage: str | None) -> Path:
    stage = (reviewer_stage or "").strip()
    if stage == "benchmark_reviewer":
        return _as_path(BENCHMARK_PLAN_PATH)
    if stage in {
        "engineering_plan_reviewer",
        "engineering_execution_reviewer",
        "electronics_reviewer",
    }:
        return _as_path(ENGINEERING_PLAN_PATH)
    return _as_path("plan.md")
