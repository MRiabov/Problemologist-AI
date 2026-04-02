from __future__ import annotations

import hashlib
import os
import uuid
from pathlib import Path

from controller.agent.benchmark.tools import _canonicalize_benchmark_constraints
from shared.agents.config import DraftingMode, load_agents_config
from shared.enums import AgentName
from shared.git_utils import repo_revision
from shared.models.schemas import PlannerSubmissionResult
from shared.script_contracts import (
    drafting_render_manifest_path_for_agent,
    drafting_script_paths_for_agent,
)
from shared.workers.schema import PlanReviewManifest
from worker_heavy.utils.dfm import (
    load_planner_manufacturing_config_from_text,
)
from worker_heavy.utils.file_validation import validate_node_output


def _planner_agent(workspace: Path | None = None) -> AgentName | None:
    raw = os.getenv("AGENT_NAME", "").strip()
    if raw:
        try:
            return AgentName(raw)
        except ValueError:
            return None

    workspace = Path.cwd() if workspace is None else Path(workspace)

    prompt_path = workspace / "prompt.md"
    if prompt_path.exists():
        prompt_text = prompt_path.read_text(encoding="utf-8")
        for candidate, marker in (
            (AgentName.BENCHMARK_PLANNER, "Agent: benchmark_planner"),
            (AgentName.ENGINEER_PLANNER, "Agent: engineer_planner"),
            (AgentName.ELECTRONICS_PLANNER, "Agent: electronics_planner"),
        ):
            if marker in prompt_text:
                return candidate

    # The Codex launcher does not need a role-specific env var here.
    # Fall back to the workspace file layout only if the prompt marker is absent.
    benchmark_handoff = workspace / "benchmark_assembly_definition.yaml"
    engineering_handoff = workspace / "assembly_definition.yaml"

    has_benchmark_handoff = benchmark_handoff.exists()
    has_engineering_handoff = engineering_handoff.exists()

    if has_benchmark_handoff and not has_engineering_handoff:
        return AgentName.BENCHMARK_PLANNER
    if has_engineering_handoff and not has_benchmark_handoff:
        return AgentName.ENGINEER_PLANNER
    return None


def _session_id() -> str:
    return os.getenv("SESSION_ID", "local-submit-plan")


def _derived_episode_id(session_id: str) -> str:
    try:
        return str(uuid.UUID(session_id))
    except Exception:
        return str(uuid.uuid5(uuid.NAMESPACE_DNS, session_id))


def _workspace_environment_version(workspace: Path) -> str | None:
    candidate = (
        workspace / "benchmark_assembly_definition.yaml"
        if (workspace / "benchmark_assembly_definition.yaml").exists()
        else workspace / "assembly_definition.yaml"
    )
    if not candidate.exists():
        return None
    try:
        import yaml

        data = yaml.safe_load(candidate.read_text(encoding="utf-8")) or {}
    except Exception:
        return None
    version = data.get("version")
    if version is None:
        return None
    version_text = str(version).strip()
    return version_text or None


def _required_files(agent_name: AgentName) -> tuple[str, ...]:
    if agent_name == AgentName.BENCHMARK_PLANNER:
        return (
            "plan.md",
            "todo.md",
            "benchmark_definition.yaml",
            "benchmark_assembly_definition.yaml",
            "manufacturing_config.yaml",
        )
    return (
        "plan.md",
        "todo.md",
        "benchmark_definition.yaml",
        "assembly_definition.yaml",
        "manufacturing_config.yaml",
    )


def _manifest_info(agent_name: AgentName) -> tuple[Path, AgentName]:
    if agent_name == AgentName.BENCHMARK_PLANNER:
        return (
            Path(".manifests/benchmark_plan_review_manifest.json"),
            AgentName.BENCHMARK_PLAN_REVIEWER,
        )
    return (
        Path(".manifests/engineering_plan_review_manifest.json"),
        AgentName.ENGINEER_PLAN_REVIEWER,
    )


def _planner_role_for_agent(agent_name: AgentName) -> AgentName | None:
    if agent_name == AgentName.BENCHMARK_PLANNER:
        return AgentName.BENCHMARK_PLANNER
    if agent_name in {AgentName.ENGINEER_PLANNER, AgentName.ELECTRONICS_PLANNER}:
        return AgentName.ENGINEER_PLANNER
    return None


def _drafting_required_files(agent_name: AgentName) -> tuple[str, ...]:
    planner_role = _planner_role_for_agent(agent_name)
    if planner_role is None:
        return ()

    try:
        drafting_mode = load_agents_config().get_technical_drawing_mode(planner_role)
    except Exception:
        drafting_mode = DraftingMode.OFF

    if drafting_mode not in (DraftingMode.MINIMAL, DraftingMode.FULL):
        return ()

    evidence_path, technical_drawing_path = drafting_script_paths_for_agent(
        planner_role
    )
    return (
        str(evidence_path),
        str(technical_drawing_path),
        str(drafting_render_manifest_path_for_agent(planner_role)),
    )


def _read_workspace_files(workspace: Path, paths: tuple[str, ...]) -> dict[str, str]:
    artifacts: dict[str, str] = {}
    missing: list[str] = []

    for rel_path in paths:
        file_path = workspace / rel_path
        if not file_path.exists():
            missing.append(rel_path)
            continue
        content = file_path.read_text(encoding="utf-8")
        if not content.strip():
            missing.append(rel_path)
            continue
        artifacts[rel_path] = content

    if missing:
        raise ValueError("Missing required file(s): " + ", ".join(missing))
    return artifacts


def submit_plan(workspace: Path | None = None) -> PlannerSubmissionResult:
    workspace = Path.cwd() if workspace is None else Path(workspace)
    agent_name = _planner_agent(workspace)
    session_id = _session_id()

    if agent_name is None:
        return PlannerSubmissionResult(
            ok=False,
            status="rejected",
            errors=[
                "Unable to infer planner agent from workspace: expected "
                "prompt.md role marker or a single planner handoff file"
            ],
            node_type=AgentName.ENGINEER_PLANNER,
        )

    required_files = list(_required_files(agent_name))
    required_files.extend(_drafting_required_files(agent_name))
    try:
        artifacts = _read_workspace_files(workspace, tuple(required_files))
    except Exception as exc:
        return PlannerSubmissionResult(
            ok=False,
            status="rejected",
            errors=[str(exc)],
            node_type=agent_name,
        )

    if agent_name == AgentName.BENCHMARK_PLANNER:
        canonical_benchmark_definition, canonicalization_errors = (
            _canonicalize_benchmark_constraints(artifacts["benchmark_definition.yaml"])
        )
        if canonicalization_errors:
            return PlannerSubmissionResult(
                ok=False,
                status="rejected",
                errors=canonicalization_errors,
                node_type=agent_name,
            )

        artifacts["benchmark_definition.yaml"] = canonical_benchmark_definition
        workspace.joinpath("benchmark_definition.yaml").write_text(
            canonical_benchmark_definition,
            encoding="utf-8",
        )

    try:
        manufacturing_config_text = artifacts["manufacturing_config.yaml"]
        manufacturing_config = load_planner_manufacturing_config_from_text(
            manufacturing_config_text
        )
    except Exception as exc:
        return PlannerSubmissionResult(
            ok=False,
            status="rejected",
            errors=[f"failed to load manufacturing_config.yaml: {exc}"],
            node_type=agent_name,
        )

    artifacts["manufacturing_config.yaml"] = manufacturing_config_text

    is_valid, errors = validate_node_output(
        agent_name,
        artifacts,
        session_id=session_id,
        manufacturing_config=manufacturing_config,
    )
    if not is_valid:
        return PlannerSubmissionResult(
            ok=False,
            status="rejected",
            errors=errors,
            node_type=agent_name,
        )

    manifest_path, reviewer_stage = _manifest_info(agent_name)
    manifest_path = workspace / manifest_path
    manifest_path.parent.mkdir(parents=True, exist_ok=True)
    manifest = PlanReviewManifest(
        status="ready_for_review",
        reviewer_stage=reviewer_stage,
        session_id=session_id,
        planner_node_type=agent_name,
        episode_id=os.getenv("EPISODE_ID") or _derived_episode_id(session_id),
        worker_session_id=session_id,
        benchmark_revision=repo_revision(workspace),
        environment_version=_workspace_environment_version(workspace),
        artifact_hashes={
            rel_path: hashlib.sha256(content.encode("utf-8")).hexdigest()
            for rel_path, content in artifacts.items()
        },
    )
    manifest_path.write_text(manifest.model_dump_json(indent=2), encoding="utf-8")

    return PlannerSubmissionResult(
        ok=True,
        status="submitted",
        errors=[],
        node_type=agent_name,
    )


def main() -> int:
    try:
        result = submit_plan()
    except Exception as exc:
        result = PlannerSubmissionResult(
            ok=False,
            status="rejected",
            errors=[f"submit_plan execution failed: {exc}"],
            node_type=_planner_agent() or AgentName.ENGINEER_PLANNER,
        )
        print(result.model_dump_json(indent=2))
        return 1
    print(result.model_dump_json(indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
