from __future__ import annotations

from collections.abc import Iterable

from shared.agents.config import DraftingMode, load_agents_config
from shared.enums import AgentName
from shared.script_contracts import (
    BENCHMARK_PLAN_EVIDENCE_SCRIPT_PATH,
    BENCHMARK_PLAN_TECHNICAL_DRAWING_SCRIPT_PATH,
    CURRENT_ROLE_MANIFEST_PATH,
    SOLUTION_PLAN_EVIDENCE_SCRIPT_PATH,
    SOLUTION_PLAN_TECHNICAL_DRAWING_SCRIPT_PATH,
)

_BENCHMARK_PLAN_BASE_FILES: tuple[str, ...] = (
    "benchmark_plan.md",
    "todo.md",
    "benchmark_definition.yaml",
    "benchmark_assembly_definition.yaml",
)

_ENGINEER_PLAN_BASE_FILES: tuple[str, ...] = (
    "engineering_plan.md",
    "todo.md",
    "benchmark_definition.yaml",
    "assembly_definition.yaml",
)

_BENCHMARK_DRAFTING_FILES: tuple[str, ...] = (
    BENCHMARK_PLAN_EVIDENCE_SCRIPT_PATH,
    BENCHMARK_PLAN_TECHNICAL_DRAWING_SCRIPT_PATH,
)

_ENGINEER_DRAFTING_FILES: tuple[str, ...] = (
    SOLUTION_PLAN_EVIDENCE_SCRIPT_PATH,
    SOLUTION_PLAN_TECHNICAL_DRAWING_SCRIPT_PATH,
)

_BENCHMARK_PLAN_ROLES = {
    AgentName.BENCHMARK_PLANNER,
    AgentName.BENCHMARK_PLAN_REVIEWER,
    AgentName.BENCHMARK_CODER,
    AgentName.BENCHMARK_REVIEWER,
}

_ENGINEER_PLAN_ROLES = {
    AgentName.ENGINEER_PLANNER,
    AgentName.ENGINEER_PLAN_REVIEWER,
    AgentName.ENGINEER_CODER,
    AgentName.ENGINEER_EXECUTION_REVIEWER,
    AgentName.ELECTRONICS_PLANNER,
    AgentName.ELECTRONICS_REVIEWER,
}

_ENGINEER_DRAFTING_TARGETS = {
    AgentName.ENGINEER_PLANNER,
    AgentName.ENGINEER_PLAN_REVIEWER,
    AgentName.ENGINEER_CODER,
    AgentName.ENGINEER_EXECUTION_REVIEWER,
}

_ENGINEER_BENCHMARK_CONTEXT_ROLES = {
    AgentName.ENGINEER_PLANNER,
    AgentName.ENGINEER_CODER,
}


def _technical_drawing_mode_active(mode: DraftingMode) -> bool:
    return mode in (DraftingMode.MINIMAL, DraftingMode.FULL)


def _agent_technical_drawing_modes() -> tuple[DraftingMode, DraftingMode]:
    try:
        config = load_agents_config()
    except Exception:
        return DraftingMode.OFF, DraftingMode.OFF

    return (
        config.get_technical_drawing_mode(AgentName.ENGINEER_PLANNER),
        config.get_technical_drawing_mode(AgentName.BENCHMARK_PLANNER),
    )


def _dedupe_paths(paths: Iterable[str]) -> tuple[str, ...]:
    return tuple(dict.fromkeys(paths))


def plan_artifacts_for_agent(agent_name: AgentName) -> tuple[str, ...]:
    engineer_mode, benchmark_mode = _agent_technical_drawing_modes()

    if agent_name in _BENCHMARK_PLAN_ROLES:
        files = list(_BENCHMARK_PLAN_BASE_FILES)
        if _technical_drawing_mode_active(benchmark_mode):
            files.extend(_BENCHMARK_DRAFTING_FILES)
        return _dedupe_paths(files)

    if agent_name in _ENGINEER_PLAN_ROLES:
        files = list(_ENGINEER_PLAN_BASE_FILES)
        if agent_name in _ENGINEER_DRAFTING_TARGETS and _technical_drawing_mode_active(
            engineer_mode
        ):
            files.extend(_ENGINEER_DRAFTING_FILES)
        if (
            agent_name in _ENGINEER_BENCHMARK_CONTEXT_ROLES
            and _technical_drawing_mode_active(benchmark_mode)
        ):
            files.extend(_BENCHMARK_DRAFTING_FILES)
        return _dedupe_paths(files)

    return ()


def workspace_artifacts_for_agent(agent_name: AgentName) -> tuple[str, ...]:
    """Return the base workspace contract including backend-owned metadata."""
    return (
        CURRENT_ROLE_MANIFEST_PATH.as_posix(),
        *plan_artifacts_for_agent(agent_name),
    )


__all__ = ["plan_artifacts_for_agent", "workspace_artifacts_for_agent"]
