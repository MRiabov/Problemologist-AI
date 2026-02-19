import json
import time
from pathlib import Path
from typing import Any

import structlog

logger = structlog.get_logger(__name__)


def collect_and_cleanup_events(session_root: Path) -> list[dict[str, Any]]:
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
        logger.error("failed_to_collect_events", error=str(e))
    return events


def record_validation_result(
    session_root: Path, is_valid: bool, message: str | None
) -> None:
    """Record validation results to satisfy the handover gate."""
    results_path = session_root / "validation_results.json"
    try:
        results_path.write_text(
            json.dumps(
                {"success": is_valid, "message": message, "timestamp": time.time()}
            ),
            encoding="utf-8",
        )
    except Exception as e:
        logger.error("failed_to_record_validation_result", error=str(e))
