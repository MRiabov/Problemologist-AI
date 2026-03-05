import hashlib

from controller.clients.worker import WorkerClient
from shared.models.simulation import SimulationResult
from shared.workers.schema import ReviewManifest, ValidationResultRecord


def _goal_reached(summary: str) -> bool:
    text = (summary or "").lower()
    return "goal achieved" in text or "green zone" in text or "goal zone" in text


async def validate_reviewer_handover(worker_client: WorkerClient) -> str | None:
    """
    Validate that reviewer handoff artifacts are present and correspond to the
    latest script revision.
    """
    manifest_path = ".manifests/review_manifest.json"
    if not await worker_client.exists(manifest_path):
        return "review_manifest.json missing; call submit_for_review(compound) first."

    try:
        manifest_raw = await worker_client.read_file(manifest_path)
        manifest = ReviewManifest.model_validate_json(manifest_raw)
    except Exception as e:
        return f"review_manifest.json invalid: {e}"

    if manifest.status != "ready_for_review":
        return "review manifest status is not ready_for_review."

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
