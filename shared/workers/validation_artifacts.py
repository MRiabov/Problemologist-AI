from __future__ import annotations

import base64
from pathlib import Path

from shared.enums import FailureReason
from shared.models.simulation import SimulationFailure
from shared.workers.persistence import (
    collect_and_cleanup_events,
    record_validation_result,
)
from shared.workers.schema import BenchmarkToolResponse, SimulationArtifacts


def normalize_render_paths(root: Path, render_paths: list[str]) -> list[str]:
    normalized: list[str] = []
    resolved_root = root.resolve()
    for raw_path in render_paths:
        try:
            candidate = Path(raw_path)
            if candidate.is_absolute():
                normalized.append(str(candidate.resolve().relative_to(resolved_root)))
            else:
                normalized.append(str(candidate))
        except Exception:
            normalized.append(raw_path)
    return normalized


def collect_validation_render_artifacts(
    root: Path,
) -> tuple[list[str], dict[str, str]]:
    render_paths: list[str] = []
    render_blobs_base64: dict[str, str] = {}
    renders_dir = root / "renders"
    if not renders_dir.exists():
        return render_paths, render_blobs_base64

    for render_path in sorted(renders_dir.rglob("*")):
        if not render_path.is_file():
            continue
        if render_path.suffix.lower() not in {".png", ".jpg", ".jpeg", ".mp4"}:
            continue
        rel_path = str(render_path.relative_to(root))
        render_paths.append(rel_path)
        render_blobs_base64[rel_path] = base64.b64encode(
            render_path.read_bytes()
        ).decode("ascii")

    render_manifest_path = renders_dir / "render_manifest.json"
    if render_manifest_path.exists():
        render_blobs_base64[str(Path("renders") / "render_manifest.json")] = (
            base64.b64encode(render_manifest_path.read_bytes()).decode("ascii")
        )

    return render_paths, render_blobs_base64


def build_validation_response(
    *,
    root: Path,
    is_valid: bool,
    message: str | None,
    script_path: str,
    session_id: str | None,
) -> BenchmarkToolResponse:
    record_validation_result(
        root,
        is_valid,
        message,
        script_path=script_path,
        session_id=session_id,
    )

    events = collect_and_cleanup_events(root, session_id=session_id)
    artifacts = SimulationArtifacts()
    validation_result_path = root / "validation_results.json"
    if validation_result_path.exists():
        artifacts.validation_results_json = validation_result_path.read_text(
            encoding="utf-8"
        )

    render_paths, render_blobs_base64 = collect_validation_render_artifacts(root)
    artifacts.render_paths = normalize_render_paths(root, render_paths)
    artifacts.render_blobs_base64 = render_blobs_base64

    if not is_valid:
        artifacts.failure = SimulationFailure(
            reason=FailureReason.VALIDATION_FAILED,
            detail=message,
        )

    return BenchmarkToolResponse(
        success=is_valid,
        message=message or "Validation successful",
        events=events,
        artifacts=artifacts,
    )
