from __future__ import annotations

import contextlib
from pathlib import Path

import structlog

from shared.workers.schema import RenderBundleObjectPoseRecord

logger = structlog.get_logger(__name__)

_PAYLOAD_SAMPLE_LABELS = ("first", "25%", "50%", "75%", "final")
_PAYLOAD_SAMPLE_FRACTIONS = (0.0, 0.25, 0.5, 0.75, 1.0)


def _format_position(position: tuple[float, float, float] | None) -> str:
    if position is None:
        return "n/a"
    return "[" + ", ".join(f"{coord:.3f}" for coord in position) + "]"


def _matches_payload_label(
    record: RenderBundleObjectPoseRecord, payload_label: str
) -> bool:
    candidates = (
        record.label,
        record.instance_name,
        record.semantic_label,
        record.body_name,
        record.geom_name,
    )
    return any(candidate == payload_label for candidate in candidates if candidate)


def summarize_payload_position_history(
    object_pose_path: Path,
    *,
    payload_label: str | None,
) -> str | None:
    """Build a compact payload pose summary from a bundle-local parquet sidecar."""
    if payload_label is None or not str(payload_label).strip():
        return None
    if not object_pose_path.exists():
        return None

    try:
        import pyarrow.parquet as pq
    except Exception as exc:
        logger.warning(
            "payload_pose_summary_parquet_unavailable",
            error=str(exc),
            path=str(object_pose_path),
        )
        return None

    try:
        table = pq.read_table(object_pose_path)
    except Exception as exc:
        logger.warning(
            "payload_pose_summary_parquet_read_failed",
            error=str(exc),
            path=str(object_pose_path),
        )
        return None

    records: list[RenderBundleObjectPoseRecord] = []
    for row in table.to_pylist():
        with contextlib.suppress(Exception):
            records.append(RenderBundleObjectPoseRecord.model_validate(row))

    if not records:
        return None

    payload_records = [
        record for record in records if _matches_payload_label(record, payload_label)
    ]
    if not payload_records:
        return (
            f"Payload positions unavailable: label {payload_label!r} not found in "
            f"{object_pose_path.name}."
        )

    frame_to_record: dict[int, RenderBundleObjectPoseRecord] = {}
    for record in payload_records:
        if record.frame_index is None or record.position is None:
            continue
        frame_to_record.setdefault(record.frame_index, record)

    frame_indices = sorted(frame_to_record)
    if not frame_indices:
        return (
            f"Payload positions unavailable: no frame-indexed rows found for "
            f"{payload_label!r} in {object_pose_path.name}."
        )

    sample_indices = [
        int((len(frame_indices) - 1) * fraction)
        for fraction in _PAYLOAD_SAMPLE_FRACTIONS
    ]
    sample_lines = [
        (
            label,
            frame_indices[index],
            frame_to_record[frame_indices[index]].position,
        )
        for label, index in zip(_PAYLOAD_SAMPLE_LABELS, sample_indices)
    ]

    lines = [
        f"Payload frame count: {len(frame_indices)}",
        f"Payload label: {payload_label}",
        "Payload positions:",
    ]
    for label, frame_index, position in sample_lines:
        lines.append(
            f"- {label} frame (frame_index={frame_index}): {_format_position(position)}"
        )
    return "\n".join(lines)


def write_object_pose_parquet(
    bundle_root: Path,
    records: list[RenderBundleObjectPoseRecord],
    *,
    source_path: str | None = None,
    session_id: str | None = None,
) -> Path | None:
    """Persist backend object poses as a bundle-local parquet sidecar."""
    if not records:
        return None

    try:
        import pyarrow as pa
        import pyarrow.parquet as pq
    except Exception as exc:
        logger.warning(
            "object_pose_parquet_writer_unavailable",
            error=str(exc),
            session_id=session_id,
        )
        return None

    bundle_root.mkdir(parents=True, exist_ok=True)
    object_pose_path = bundle_root / "objects.parquet"
    temp_path = object_pose_path.with_suffix(".parquet.tmp")

    rows: list[dict[str, object]] = []
    for record in records:
        row = record.model_dump(mode="json")
        if source_path is not None:
            row["source_path"] = source_path
        rows.append(row)

    try:
        table = pa.Table.from_pylist(rows)
        pq.write_table(table, temp_path)
        temp_path.replace(object_pose_path)
        logger.info(
            "object_pose_parquet_written",
            path=str(object_pose_path),
            row_count=len(rows),
            session_id=session_id,
        )
        return object_pose_path
    except Exception as exc:
        logger.warning(
            "object_pose_parquet_write_failed",
            error=str(exc),
            path=str(object_pose_path),
            session_id=session_id,
        )
        return None
    finally:
        if temp_path.exists():
            with contextlib.suppress(Exception):
                temp_path.unlink()
