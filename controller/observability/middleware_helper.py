import asyncio
import uuid
from datetime import UTC, datetime
from typing import Any

import structlog

from controller.observability.broadcast import EpisodeBroadcaster
from controller.observability.tracing import record_worker_events, sync_asset
from shared.enums import FailureReason as SimulationFailureMode
from shared.observability.schemas import (
    SimulationInstabilityEvent,
    SimulationResultEvent,
)

logger = structlog.get_logger(__name__)


async def record_events(episode_id: str, events: list[Any]):
    """Proxy to record_worker_events."""
    await record_worker_events(episode_id=episode_id, events=events)


async def broadcast_file_update(episode_id_str: str, path: str, content: str):
    """Broadcast file update to frontend and sync to Asset table."""
    global logger
    if "logger" not in globals() or logger is None:
        logger = structlog.get_logger(__name__)
    try:
        # Sync to Asset table first to get DB record.
        # sync_asset now handles prefixed UUID strings.
        asset = await sync_asset(episode_id_str, path, content)

        try:
            # We still need a UUID for broadcasting via EpisodeBroadcaster
            # Attempt to parse it using the same logic as sync_asset
            clean_id = episode_id_str
            if "-" in episode_id_str and not episode_id_str.startswith("{"):
                parts = episode_id_str.split("-")
                if len(parts) > 1 and len(parts[0]) <= 4:
                    potential_uuid = "-".join(parts[1:])
                    if len(potential_uuid) >= 32:
                        clean_id = potential_uuid

            episode_id = uuid.UUID(clean_id)
        except ValueError:
            # Fallback to direct conversion if my stripping was too aggressive/wrong
            episode_id = uuid.UUID(episode_id_str)

        payload = {
            "type": "file_update",
            "data": {
                "path": path,
                "content": content,
            },
            "timestamp": datetime.now(UTC).isoformat(),
        }

        if asset:
            payload["data"].update(
                {
                    "id": asset.id,
                    "asset_type": asset.asset_type,
                    "created_at": asset.created_at.isoformat()
                    if asset.created_at
                    else None,
                }
            )

        await EpisodeBroadcaster.get_instance().broadcast(episode_id, payload)
    except ValueError:
        # If session_id is not a UUID, we can't broadcast (standard in some dev/test setups)
        pass
    except Exception as e:
        logger.error("broadcast_file_update_failed", error=str(e), path=path)
        # Don't fail the write operation if broadcast fails
        pass


async def sync_file_asset(episode_id_str: str, path: str, content: str):
    """Sync file content to Asset table."""
    try:
        await sync_asset(episode_id_str, path, content)
    except Exception:
        pass


def map_simulation_failure_reason(res_dict: dict[str, Any]) -> SimulationFailureMode:
    """Map raw failure reasons from simulation result to SimulationFailureMode enum."""
    if res_dict.get("success", True):
        return SimulationFailureMode.NONE

    # Structured failure from SimulationFailure model
    failure_data = res_dict.get("failure")
    if isinstance(failure_data, dict) and "reason" in failure_data:
        try:
            return SimulationFailureMode(failure_data["reason"])
        except ValueError:
            pass

    raw_reason = str(res_dict.get("failure_reason", "")).upper()
    if "TIMEOUT" in raw_reason:
        return SimulationFailureMode.TIMEOUT
    if "OUT_OF_BOUNDS" in raw_reason:
        return SimulationFailureMode.OUT_OF_BOUNDS
    if "FORBID" in raw_reason:
        return SimulationFailureMode.FORBID_ZONE_HIT
    if "BREAK" in raw_reason or "STRESS" in raw_reason:
        return SimulationFailureMode.PART_BREAKAGE
    if "NAN" in raw_reason or "INSTABILITY" in raw_reason:
        return SimulationFailureMode.PHYSICS_INSTABILITY
    if "SHORT_CIRCUIT" in raw_reason:
        return SimulationFailureMode.SHORT_CIRCUIT
    if "OVERCURRENT" in raw_reason:
        return SimulationFailureMode.OVERCURRENT
    if "WIRE_TORN" in raw_reason:
        return SimulationFailureMode.WIRE_TORN
    if "OPEN_CIRCUIT" in raw_reason:
        return SimulationFailureMode.OPEN_CIRCUIT
    if "ELECTRONICS_FLUID_DAMAGE" in raw_reason:
        return SimulationFailureMode.ELECTRONICS_FLUID_DAMAGE

    return SimulationFailureMode.NONE


async def record_simulation_result(episode_id: str, res_dict: dict[str, Any]):
    """Records simulation result and instability events."""
    failure_reason = map_simulation_failure_reason(res_dict)

    await record_worker_events(
        episode_id=episode_id,
        events=[
            SimulationResultEvent(
                success=res_dict.get("success", True),
                failure_reason=failure_reason,
                failure=res_dict.get("failure"),
                time_elapsed_s=res_dict.get("time_elapsed_s", 0.0),
                compute_time_ms=res_dict.get("compute_time_ms", 0.0),
                metadata=res_dict,
            )
        ],
    )

    # Detect instability
    if not res_dict.get("success", True):
        raw_reason = res_dict.get("failure_reason", "").lower()
        if any(
            word in raw_reason
            for word in ["nan", "penetration", "instability", "joint violation"]
        ):
            instability_type = "unknown"
            if "nan" in raw_reason:
                instability_type = "nan"
            elif "penetration" in raw_reason:
                instability_type = "penetration"
            elif "joint violation" in raw_reason:
                instability_type = "joint_violation"

            await record_worker_events(
                episode_id=episode_id,
                events=[
                    SimulationInstabilityEvent(
                        instability_type=instability_type,
                        part_ids=res_dict.get("offending_parts", []),
                        message=res_dict.get("failure_reason"),
                    )
                ],
            )
