import hashlib
from typing import Literal

import yaml

from controller.clients.worker import WorkerClient
from shared.enums import AgentName
from shared.models.schemas import AssemblyDefinition
from shared.models.simulation import SimulationResult
from shared.workers.schema import (
    PlanReviewManifest,
    ReviewerStage,
    ReviewManifest,
    ValidationResultRecord,
)
from worker_heavy.utils.file_validation import (
    validate_benchmark_definition_yaml,
    validate_declared_planner_cost_contract,
    validate_environment_attachment_contract,
)
from worker_heavy.workbenches.config import load_config, load_merged_config

PlanReviewerStage = Literal[
    AgentName.BENCHMARK_PLAN_REVIEWER,
    AgentName.ENGINEER_PLAN_REVIEWER,
]


def _goal_reached(summary: str) -> bool:
    text = (summary or "").lower()
    return "goal achieved" in text or "green zone" in text or "goal zone" in text


async def validate_reviewer_handover(
    worker_client: WorkerClient,
    *,
    manifest_path: str,
    expected_stage: ReviewerStage,
) -> str | None:
    """
    Validate that reviewer handoff artifacts are present and correspond to the
    latest script revision.
    """
    if not await worker_client.exists(manifest_path):
        return (
            f"{manifest_path} missing; call submit_for_review(compound) "
            "with the correct reviewer stage first."
        )

    try:
        manifest_raw = await worker_client.read_file(manifest_path)
        manifest = ReviewManifest.model_validate_json(manifest_raw)
    except Exception as e:
        return f"{manifest_path} invalid: {e}"

    if manifest.status != "ready_for_review":
        return "review manifest status is not ready_for_review."
    if manifest.reviewer_stage != expected_stage:
        return (
            f"review manifest stage mismatch: expected {expected_stage}, "
            f"got {manifest.reviewer_stage}."
        )

    if not await worker_client.exists(manifest.script_path):
        return f"script missing at manifest path: {manifest.script_path}"

    script_content = await worker_client.read_file(manifest.script_path)
    script_sha = hashlib.sha256(script_content.encode("utf-8")).hexdigest()
    if script_sha != manifest.script_sha256:
        return (
            "review manifest does not match latest script revision; "
            "re-run validate, simulate, and submit_for_review(compound)."
        )

    if not await worker_client.exists("validation_results.json"):
        return "validation_results.json missing."
    try:
        val_raw = await worker_client.read_file("validation_results.json")
        val_record = ValidationResultRecord.model_validate_json(val_raw)
    except Exception as e:
        return f"validation_results.json invalid: {e}"
    if not val_record.success or not manifest.validation_success:
        return "validation gate failed for latest revision."
    if val_record.script_sha256 != manifest.script_sha256:
        return (
            "validation results do not match review manifest script revision; "
            "re-run validate, simulate, and submit_for_review(compound)."
        )

    if not await worker_client.exists("simulation_result.json"):
        return "simulation_result.json missing."
    try:
        sim_raw = await worker_client.read_file("simulation_result.json")
        sim_result = SimulationResult.model_validate_json(sim_raw)
    except Exception as e:
        return f"simulation_result.json invalid: {e}"

    if not sim_result.success or not manifest.simulation_success:
        return "simulation gate failed for latest revision."
    if not _goal_reached(sim_result.summary):
        return "simulation did not reach objective (green/goal zone)."
    if not manifest.goal_reached:
        return "review manifest indicates goal not reached."

    if not _goal_reached(manifest.simulation_summary):
        return "review manifest simulation summary does not confirm goal completion."

    return None


async def validate_plan_reviewer_handover(
    worker_client: WorkerClient,
    *,
    manifest_path: str = ".manifests/engineering_plan_review_manifest.json",
    expected_stage: PlanReviewerStage = AgentName.ENGINEER_PLAN_REVIEWER,
) -> str | None:
    """Validate planner-to-plan-reviewer handoff using stage-specific manifest."""
    if not await worker_client.exists(manifest_path):
        return f"{manifest_path} missing; call submit_plan() first."

    try:
        manifest_raw = await worker_client.read_file(manifest_path)
        manifest = PlanReviewManifest.model_validate_json(manifest_raw)
    except Exception as e:
        return f"{manifest_path} invalid: {e}"

    if manifest.status != "ready_for_review":
        return "plan review manifest status is not ready_for_review."
    if manifest.reviewer_stage != expected_stage:
        return (
            f"plan review manifest stage mismatch: expected {expected_stage}, "
            f"got {manifest.reviewer_stage}."
        )

    if not manifest.artifact_hashes:
        return "plan review manifest has no artifact_hashes."

    for rel_path, expected_hash in manifest.artifact_hashes.items():
        if not await worker_client.exists(rel_path):
            return f"planner artifact missing: {rel_path}"
        content = await worker_client.read_file(rel_path)
        actual_hash = hashlib.sha256(content.encode("utf-8")).hexdigest()
        if actual_hash != expected_hash:
            return (
                f"planner artifact hash mismatch for {rel_path}; "
                "re-run submit_plan() for latest planner revision."
            )

    return await validate_planner_artifacts_cross_contract(
        worker_client,
        expected_stage=expected_stage,
    )


async def validate_planner_artifacts_cross_contract(
    worker_client: WorkerClient,
    *,
    expected_stage: PlanReviewerStage = AgentName.ENGINEER_PLAN_REVIEWER,
) -> str | None:
    """Validate benchmark + assembly planner artifacts without requiring a manifest."""
    if not await worker_client.exists("benchmark_definition.yaml"):
        return f"benchmark_definition.yaml missing for {expected_stage} handoff."

    assembly_definition_path = (
        "benchmark_assembly_definition.yaml"
        if expected_stage == AgentName.BENCHMARK_PLAN_REVIEWER
        else "assembly_definition.yaml"
    )
    if not await worker_client.exists(assembly_definition_path):
        return f"{assembly_definition_path} missing for {expected_stage} handoff."

    try:
        benchmark_raw = await worker_client.read_file("benchmark_definition.yaml")
        is_valid, benchmark_result = validate_benchmark_definition_yaml(
            benchmark_raw,
            session_id=worker_client.session_id,
        )
        if not is_valid:
            return "planner handoff cross-validation parse failure: " + "; ".join(
                benchmark_result
            )
        benchmark_definition = benchmark_result
        assembly_definition = AssemblyDefinition.model_validate(
            yaml.safe_load(await worker_client.read_file(assembly_definition_path))
            or {}
        )
    except Exception as e:
        return f"planner handoff cross-validation parse failure: {e}"

    attachment_errors = validate_environment_attachment_contract(
        benchmark_definition=benchmark_definition,
        assembly_definition=assembly_definition,
    )
    if attachment_errors:
        return "; ".join(attachment_errors)

    try:
        if await worker_client.exists("manufacturing_config.yaml"):
            manufacturing_config = load_merged_config(
                override_data=(
                    yaml.safe_load(
                        await worker_client.read_file("manufacturing_config.yaml")
                    )
                    or {}
                )
            )
        else:
            manufacturing_config = load_config()
    except Exception as e:
        return f"planner handoff pricing-config parse failure: {e}"

    cost_errors = validate_declared_planner_cost_contract(
        assembly_definition=assembly_definition,
        manufacturing_config=manufacturing_config,
    )
    if cost_errors:
        return "; ".join(cost_errors)

    return None
