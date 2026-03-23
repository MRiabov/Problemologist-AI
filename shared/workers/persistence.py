import json
import time
from hashlib import sha256
from pathlib import Path
from typing import Any

import structlog

from shared.models.simulation import MultiRunResult
from shared.workers.schema import ValidationResultRecord

logger = structlog.get_logger(__name__)


def collect_and_cleanup_events(
    session_root: Path, session_id: str | None = None
) -> list[dict[str, Any]]:
    """Read and delete events.jsonl from the workspace root."""
    events = []
    events_path = session_root / "events.jsonl"
    try:
        if events_path.exists():
            content = events_path.read_text(encoding="utf-8")
            for line in content.strip().split("\n"):
                if line:
                    try:
                        events.append(json.loads(line))
                    except json.JSONDecodeError:
                        logger.warning("failed_to_decode_event_line", line=line)

            # Delete the file after reading to avoid cross-contamination between runs
            events_path.unlink()
    except Exception as e:
        logger.error("failed_to_collect_events", error=str(e), session_id=session_id)
    return events


def record_validation_result(
    session_root: Path,
    is_valid: bool,
    message: str | None,
    script_path: str = "script.py",
    session_id: str | None = None,
    verification_result: MultiRunResult | None = None,
) -> None:
    """Record validation results to satisfy the handover gate."""
    results_path = session_root / "validation_results.json"
    resolved_script = session_root / script_path
    script_hash: str | None = None
    if resolved_script.exists():
        try:
            script_hash = sha256(resolved_script.read_bytes()).hexdigest()
        except Exception as e:
            logger.warning("failed_to_hash_validation_script", error=str(e))
    try:
        record = ValidationResultRecord(
            success=is_valid,
            message=message,
            timestamp=time.time(),
            script_path=script_path,
            script_sha256=script_hash,
            verification_result=verification_result,
        )
        results_path.write_text(record.model_dump_json(indent=2), encoding="utf-8")
    except Exception as e:
        logger.error(
            "failed_to_record_validation_result", error=str(e), session_id=session_id
        )
