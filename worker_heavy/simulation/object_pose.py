from __future__ import annotations

import contextlib
from pathlib import Path

import structlog

from shared.workers.schema import RenderBundleObjectPoseRecord

logger = structlog.get_logger(__name__)


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
