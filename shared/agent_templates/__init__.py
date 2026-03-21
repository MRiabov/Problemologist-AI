from __future__ import annotations

from pathlib import Path

from shared.enums import AgentName

AGENT_TEMPLATES_ROOT = Path(__file__).resolve().parent

_NON_TEMPLATE_ROLES: frozenset[AgentName] = frozenset(
    {
        AgentName.COTS_SEARCH,
        AgentName.GIT_AGENT,
        AgentName.STEER,
    }
)

_ENGINEER_TEMPLATE_ROLES: frozenset[AgentName] = frozenset(
    {
        AgentName.ENGINEER_PLANNER,
        AgentName.ELECTRONICS_PLANNER,
        AgentName.ELECTRONICS_ENGINEER,
    }
)

_BENCHMARK_TEMPLATE_ROLES: frozenset[AgentName] = frozenset(
    {
        AgentName.BENCHMARK_PLANNER,
    }
)


def template_subdirs_for_agent(agent_name: AgentName) -> tuple[str, ...]:
    """Return shared template subdirectories to copy into the agent workspace."""
    if agent_name in _NON_TEMPLATE_ROLES:
        return ()

    subdirs = ["common"]
    if agent_name in _ENGINEER_TEMPLATE_ROLES:
        subdirs.append("engineer")
    elif agent_name in _BENCHMARK_TEMPLATE_ROLES:
        subdirs.append("benchmark_generator")

    return tuple(subdirs)


def resolve_template_path(template_file: str | Path) -> Path:
    """Resolve a template-relative path safely inside shared/agent_templates."""
    candidate = Path(template_file)
    resolved = (
        candidate if candidate.is_absolute() else AGENT_TEMPLATES_ROOT / candidate
    ).resolve()
    root = AGENT_TEMPLATES_ROOT.resolve()
    try:
        resolved.relative_to(root)
    except ValueError as exc:
        raise ValueError(
            f"Template path escapes shared/agent_templates: {template_file}"
        ) from exc

    if not resolved.is_file():
        raise FileNotFoundError(f"Template file not found: {template_file}")
    return resolved


def load_template_text(template_file: str | Path) -> str:
    """Read template text from shared/agent_templates."""
    return resolve_template_path(template_file).read_text(encoding="utf-8")


def load_agent_template_files(agent_name: AgentName) -> dict[str, str]:
    """Load the starter files that should be copied into an agent workspace."""
    merged: dict[str, str] = {}
    for template_subdir in template_subdirs_for_agent(agent_name):
        template_root = AGENT_TEMPLATES_ROOT / template_subdir
        if not template_root.exists():
            raise FileNotFoundError(f"Template directory not found: {template_root}")

        for src_path in sorted(p for p in template_root.rglob("*") if p.is_file()):
            if "__pycache__" in src_path.parts or src_path.suffix in {".pyc", ".pyo"}:
                continue
            rel_path = src_path.relative_to(template_root).as_posix()
            merged[rel_path] = src_path.read_text(encoding="utf-8")

    return merged
