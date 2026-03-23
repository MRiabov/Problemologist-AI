from __future__ import annotations

import hashlib
import os
from pathlib import Path

from controller.agent.benchmark.tools import _canonicalize_benchmark_constraints
from shared.enums import AgentName
from shared.models.schemas import PlannerSubmissionResult
from shared.workers.schema import PlanReviewManifest
from worker_heavy.utils.file_validation import validate_node_output
from worker_heavy.utils.dfm import load_planner_manufacturing_config


def _planner_agent(workspace: Path | None = None) -> AgentName | None:
    raw = os.getenv("AGENT_NAME", "").strip()
    if raw:
        try:
            return AgentName(raw)
        except ValueError:
            return None

    # The Codex launcher does not need a role-specific env var here.
    # Infer the planner variant from the workspace file layout instead.
    workspace = Path.cwd() if workspace is None else Path(workspace)
    benchmark_handoff = workspace / "benchmark_assembly_definition.yaml"
    engineering_handoff = workspace / "assembly_definition.yaml"

    has_benchmark_handoff = benchmark_handoff.exists()
    has_engineering_handoff = engineering_handoff.exists()

    if has_benchmark_handoff and not has_engineering_handoff:
        return AgentName.BENCHMARK_PLANNER
    if has_engineering_handoff and not has_benchmark_handoff:
        return AgentName.ENGINEER_PLANNER
    if has_benchmark_handoff and has_engineering_handoff:
        return None
    return None


def _session_id() -> str:
    return os.getenv("SESSION_ID", "local-submit-plan")


def _required_files(agent_name: AgentName) -> tuple[str, ...]:
    if agent_name == AgentName.BENCHMARK_PLANNER:
        return (
            "plan.md",
            "todo.md",
            "benchmark_definition.yaml",
            "benchmark_assembly_definition.yaml",
        )
    return (
        "plan.md",
        "todo.md",
        "benchmark_definition.yaml",
        "assembly_definition.yaml",
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


def _manufacturing_config(workspace: Path):
    config_path = workspace / "manufacturing_config.yaml"
    return load_planner_manufacturing_config(config_path=config_path)


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
                "benchmark_assembly_definition.yaml or "
                "assembly_definition.yaml"
            ],
            node_type=AgentName.ENGINEER_PLANNER,
        )

    required_files = _required_files(agent_name)
    try:
        artifacts = _read_workspace_files(workspace, required_files)
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
        manufacturing_config = _manufacturing_config(workspace)
    except Exception as exc:
        return PlannerSubmissionResult(
            ok=False,
            status="rejected",
            errors=[f"failed to load manufacturing_config.yaml: {exc}"],
            node_type=agent_name,
        )

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
