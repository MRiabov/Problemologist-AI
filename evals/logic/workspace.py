from pathlib import Path
from typing import Any

from controller.agent.node_entry_validation import (
    BENCHMARK_CODER_HANDOVER_CHECK,
    BENCHMARK_PLAN_REVIEWER_HANDOVER_CHECK,
    BENCHMARK_REVIEWER_HANDOVER_CHECK,
    ELECTRONICS_REVIEWER_HANDOVER_CHECK,
    ENGINEER_EXECUTION_REVIEWER_HANDOVER_CHECK,
    ENGINEER_PLAN_REVIEWER_HANDOVER_CHECK,
    ValidationGraph,
    benchmark_coder_handover_custom_check_from_session_id,
    benchmark_plan_reviewer_handover_custom_check_from_session_id,
    build_benchmark_node_contracts,
    build_engineer_node_contracts,
    evaluate_node_entry_contract,
    plan_reviewer_handover_custom_check_from_session_id,
    reviewer_handover_custom_check_from_session_id,
    validate_seeded_workspace_handoff_artifacts,
)
from controller.clients.worker import WorkerClient
from evals.logic.models import AgentEvalSpec, EvalDatasetItem
from shared.agent_templates import load_agent_template_files
from shared.enums import AgentName, EvalMode


def resolve_seed_artifact_dir(item: EvalDatasetItem, *, root: Path) -> Path | None:
    if item.seed_artifact_dir is None:
        return None

    artifact_dir = Path(item.seed_artifact_dir)
    if artifact_dir.is_absolute():
        return artifact_dir

    repo_relative = root / artifact_dir
    if repo_relative.exists():
        return repo_relative

    if item.seed_dataset is not None:
        dataset_relative = (root / item.seed_dataset).parent / artifact_dir
        if dataset_relative.exists():
            return dataset_relative

    return repo_relative


async def seed_eval_workspace(
    *,
    item: EvalDatasetItem,
    session_id: str,
    agent_name: AgentName,
    root: Path,
    worker_light_url: str,
    logger: Any,
) -> None:
    artifact_dir = resolve_seed_artifact_dir(item, root=root)
    inline_files = item.seed_files or {}
    template_files = load_agent_template_files(agent_name)
    if artifact_dir is None and not inline_files and not template_files:
        return

    worker = WorkerClient(base_url=worker_light_url, session_id=session_id)
    seeded_paths: list[str] = []
    try:
        for rel_path, content in template_files.items():
            await worker.write_file(
                rel_path,
                content,
                overwrite=True,
                bypass_agent_permissions=True,
            )
            seeded_paths.append(rel_path)

        if artifact_dir is not None:
            if not artifact_dir.exists():
                raise FileNotFoundError(
                    f"Seed artifact directory not found: {artifact_dir}"
                )
            for path in sorted(p for p in artifact_dir.rglob("*") if p.is_file()):
                rel_path = path.relative_to(artifact_dir).as_posix()
                raw_bytes = path.read_bytes()
                try:
                    content = raw_bytes.decode("utf-8")
                except UnicodeDecodeError:
                    await worker.upload_file(
                        rel_path,
                        raw_bytes,
                        bypass_agent_permissions=True,
                    )
                else:
                    await worker.write_file(
                        rel_path,
                        content,
                        overwrite=True,
                        bypass_agent_permissions=True,
                    )
                seeded_paths.append(rel_path)

        for rel_path, content in inline_files.items():
            await worker.write_file(
                rel_path,
                content,
                overwrite=True,
                bypass_agent_permissions=True,
            )
            seeded_paths.append(rel_path)
    finally:
        await worker.aclose()

    logger.info(
        "eval_seed_workspace_applied",
        session_id=session_id,
        agent_name=agent_name,
        seed_file_count=len(seeded_paths),
        seeded_paths=seeded_paths,
    )


async def preflight_seeded_entry_contract(
    *,
    item: EvalDatasetItem,
    session_id: str,
    agent_name: AgentName,
    spec: AgentEvalSpec,
    worker_light_url: str,
    logger: Any,
) -> None:
    if item.seed_artifact_dir is None and not item.seed_files:
        return

    if spec.mode == EvalMode.BENCHMARK:
        target_node = spec.start_node or agent_name
        contracts = build_benchmark_node_contracts()
        graph = ValidationGraph.BENCHMARK
    elif spec.mode == EvalMode.AGENT:
        target_node = spec.start_node or agent_name
        contracts = build_engineer_node_contracts()
        graph = ValidationGraph.ENGINEER
    else:
        return

    contract = contracts.get(target_node)
    if contract is None:
        return

    custom_checks = {
        BENCHMARK_PLAN_REVIEWER_HANDOVER_CHECK: (
            lambda *, contract, state: (  # noqa: ARG005
                benchmark_plan_reviewer_handover_custom_check_from_session_id(
                    session_id=session_id,
                )
            )
        ),
        BENCHMARK_CODER_HANDOVER_CHECK: (
            lambda *, contract, state: (  # noqa: ARG005
                benchmark_coder_handover_custom_check_from_session_id(
                    session_id=session_id,
                    custom_objectives=None,
                )
            )
        ),
        BENCHMARK_REVIEWER_HANDOVER_CHECK: (
            lambda *, contract, state: reviewer_handover_custom_check_from_session_id(  # noqa: ARG005
                session_id=session_id,
                reviewer_label="Benchmark",
                manifest_path=".manifests/benchmark_review_manifest.json",
                expected_stage="benchmark_reviewer",
            )
        ),
        ENGINEER_PLAN_REVIEWER_HANDOVER_CHECK: (
            lambda *, contract, state: (  # noqa: ARG005
                plan_reviewer_handover_custom_check_from_session_id(
                    session_id=session_id,
                )
            )
        ),
        ENGINEER_EXECUTION_REVIEWER_HANDOVER_CHECK: (
            lambda *, contract, state: reviewer_handover_custom_check_from_session_id(  # noqa: ARG005
                session_id=session_id,
                reviewer_label="Execution",
                manifest_path=".manifests/engineering_execution_review_manifest.json",
                expected_stage="engineering_execution_reviewer",
            )
        ),
        ELECTRONICS_REVIEWER_HANDOVER_CHECK: (
            lambda *, contract, state: reviewer_handover_custom_check_from_session_id(  # noqa: ARG005
                session_id=session_id,
                reviewer_label="Electronics",
                manifest_path=".manifests/electronics_review_manifest.json",
                expected_stage="electronics_reviewer",
            )
        ),
    }

    worker = WorkerClient(base_url=worker_light_url, session_id=session_id)
    try:
        result = await evaluate_node_entry_contract(
            contract=contract,
            state={
                "task": item.task,
                "episode_id": session_id,
                "session": {
                    "session_id": session_id,
                    "custom_objectives": None,
                },
            },
            artifact_exists=worker.exists,
            graph=graph,
            custom_checks=custom_checks,
            integration_mode=True,
        )
        if result.ok:
            supplemental_errors = await validate_seeded_workspace_handoff_artifacts(
                worker_client=worker,
                target_node=target_node,
            )

            if supplemental_errors:
                raise ValueError(
                    f"Seeded entry contract invalid for {target_node.value}: "
                    + "; ".join(
                        f"{error.code}: {error.message}"
                        for error in supplemental_errors
                    )
                )
    finally:
        await worker.aclose()

    if result.ok:
        logger.info(
            "eval_seed_entry_preflight_passed",
            session_id=session_id,
            agent_name=agent_name,
            target_node=target_node,
        )
        return

    errors = [error.model_dump(mode="json") for error in result.errors]
    raise ValueError(
        f"Seeded entry contract invalid for {target_node.value}: "
        + "; ".join(f"{error['code']}: {error['message']}" for error in errors)
    )
