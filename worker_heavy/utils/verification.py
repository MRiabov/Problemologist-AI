import asyncio
from pathlib import Path

import structlog
from fastapi import HTTPException

from shared.enums import FailureReason
from shared.models.simulation import SimulationFailure
from shared.simulation.schemas import SimulatorBackendType
from shared.workers.loader import load_component_from_script
from shared.workers.persistence import (
    collect_and_cleanup_events,
    record_validation_result,
)
from shared.workers.schema import (
    BenchmarkToolResponse,
    SimulationArtifacts,
    ValidationResultRecord,
    VerificationRequest,
)
from worker_heavy.simulation.factory import get_simulation_builder
from worker_heavy.simulation.verification import verify_with_jitter
from worker_heavy.utils.file_validation import validate_benchmark_definition_yaml

logger = structlog.get_logger(__name__)


def _load_workspace_benchmark_definition(root: Path, *, session_id: str | None = None):
    benchmark_path = root / "benchmark_definition.yaml"
    if not benchmark_path.exists():
        return None

    raw = benchmark_path.read_text(encoding="utf-8")
    is_valid, objectives_or_errors = validate_benchmark_definition_yaml(
        raw,
        session_id=session_id,
    )
    if not is_valid:
        raise ValueError("; ".join(objectives_or_errors))
    return objectives_or_errors


async def run_verification_job(
    root: Path,
    request: VerificationRequest,
    *,
    session_id: str,
) -> BenchmarkToolResponse:
    """Run runtime-randomization verification from an extracted session root."""
    try:
        backend_type = request.backend
        if isinstance(backend_type, str):
            backend_type = SimulatorBackendType(backend_type)

        num_scenes = request.num_scenes
        duration = request.duration
        if request.smoke_test_mode:
            # Keep smoke verification lightweight unless the caller explicitly
            # requested a larger batch or longer duration.
            provided_fields = getattr(request, "model_fields_set", set())
            if "num_scenes" not in provided_fields:
                num_scenes = 1
            if "duration" not in provided_fields:
                duration = 1.0

        objectives = _load_workspace_benchmark_definition(root, session_id=session_id)
        component = load_component_from_script(
            script_path=root / request.script_path,
            session_root=root,
            script_content=request.script_content,
        )

        builder = get_simulation_builder(root, backend_type=backend_type)
        scene_path = await asyncio.to_thread(
            builder.build_from_assembly,
            component,
            objectives,
            smoke_test_mode=request.smoke_test_mode,
        )

        result = await asyncio.to_thread(
            verify_with_jitter,
            xml_path=str(scene_path),
            control_inputs={},
            jitter_range=request.jitter_range,
            num_scenes=num_scenes or 5,
            duration=duration or 10.0,
            seed=request.seed,
            smoke_test_mode=request.smoke_test_mode,
            backend_type=backend_type.value,
            session_id=session_id,
            explicit_target_body_name=(
                objectives.payload.label if objectives else None
            ),
        )

        events = collect_and_cleanup_events(root, session_id=session_id)

        validation_result_path = root / "validation_results.json"
        if not validation_result_path.exists():
            raise HTTPException(
                status_code=422,
                detail=(
                    "validation_results.json missing; run /benchmark/validate "
                    "or /engineering/validate "
                    "before requesting runtime verification."
                ),
            )
        try:
            validation_record = ValidationResultRecord.model_validate_json(
                validation_result_path.read_text(encoding="utf-8")
            )
        except Exception as exc:
            raise HTTPException(
                status_code=422,
                detail=f"validation_results.json invalid: {exc}",
            ) from exc

        record_validation_result(
            root,
            validation_record.success,
            validation_record.message,
            script_path=validation_record.script_path or request.script_path,
            session_id=session_id,
            verification_result=result,
        )

        validation_result_json = validation_result_path.read_text(encoding="utf-8")
        fail_obj = None
        if result.success_rate < 0.7 and result.fail_reasons:
            fail_obj = SimulationFailure(
                reason=FailureReason.VALIDATION_FAILED,
                detail="; ".join(result.fail_reasons),
            )

        artifacts = SimulationArtifacts(
            verification_result=result,
            validation_results_json=validation_result_json,
            scene_path=str(scene_path.relative_to(root)),
            failure=fail_obj,
        )

        return BenchmarkToolResponse(
            success=result.success_rate >= 0.7,
            message=(
                f"Verification complete. Success rate: {result.success_rate:.2f} "
                f"({result.success_count}/{result.num_scenes})"
            ),
            confidence="high" if result.num_scenes >= 5 else "medium",
            artifacts=artifacts,
            events=events,
        )
    except HTTPException:
        raise
    except Exception as exc:
        logger.warning("api_benchmark_verify_failed", error=str(exc))
        return BenchmarkToolResponse(
            success=False,
            message=str(exc),
            artifacts=SimulationArtifacts(
                failure=SimulationFailure(
                    reason=FailureReason.VERIFICATION_ERROR, detail=str(exc)
                )
            ),
        )
