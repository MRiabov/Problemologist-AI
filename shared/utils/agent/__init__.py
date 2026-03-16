import os
from pathlib import Path
from typing import Any

import httpx
import structlog
from build123d import Compound
from pydantic import BaseModel

from shared.enums import AgentName
from shared.utils.fasteners import HoleType as HoleType
from shared.utils.fasteners import fastener_hole as fastener_hole
from shared.workers.schema import BenchmarkToolResponse, PlanRefusal

logger = structlog.get_logger(__name__)
SCRIPT_IMPORT_MODE_ENV = "PROBLEMOLOGIST_SCRIPT_IMPORT_MODE"
SCRIPT_IMPORT_DEFERRED_MESSAGE = "Deferred during control-plane script import."

# --- Proxy Logic ---


def _call_heavy_worker(endpoint: str, payload: dict | BaseModel) -> dict:
    heavy_url = os.getenv("WORKER_HEAVY_URL", "http://worker-heavy:8002")
    session_id = os.getenv("SESSION_ID", "default")

    url = f"{heavy_url.rstrip('/')}/{endpoint.lstrip('/')}"
    headers = {"X-Session-ID": session_id}

    json_payload = payload.model_dump() if isinstance(payload, BaseModel) else payload

    try:
        with httpx.Client(timeout=300.0) as client:
            resp = client.post(url, json=json_payload, headers=headers)
            resp.raise_for_status()
            return resp.json()
    except Exception as e:
        logger.error(f"Heavy worker call failed: {e}")
        return {"success": False, "message": str(e)}


# --- Agent Utils ---


def _is_script_import_mode() -> bool:
    return os.getenv(SCRIPT_IMPORT_MODE_ENV) == "1"


def simulate(compound: Compound, **kwargs) -> BenchmarkToolResponse:
    """Proxy for heavy simulation."""
    if _is_script_import_mode():
        return BenchmarkToolResponse(
            success=True,
            message=SCRIPT_IMPORT_DEFERRED_MESSAGE,
        )
    if os.getenv("IS_HEAVY_WORKER"):
        from worker_heavy.utils.validation import simulate as real_simulate

        return real_simulate(compound, **kwargs)

    # In light worker, we call heavy worker
    payload = {"script_path": "script.py", **kwargs}
    res = _call_heavy_worker("/benchmark/simulate", payload)
    return BenchmarkToolResponse.model_validate(res)


def validate(compound: Compound, **kwargs) -> tuple[bool, str | None]:
    """Proxy for benchmark geometric validation."""
    if _is_script_import_mode():
        return True, SCRIPT_IMPORT_DEFERRED_MESSAGE
    if os.getenv("IS_HEAVY_WORKER"):
        from worker_heavy.utils.validation import validate as real_validate

        return real_validate(compound, **kwargs)

    # In light worker, we call heavy worker
    payload = {"script_path": "script.py", **kwargs}
    res = _call_heavy_worker("/benchmark/validate", payload)
    parsed = BenchmarkToolResponse.model_validate(res)
    return parsed.success, parsed.message


def validate_and_price(
    part: Any, method: Any = None, config: Any = None
) -> BenchmarkToolResponse:
    """Proxy for DFM analysis."""
    if os.getenv("IS_HEAVY_WORKER"):
        from worker_heavy.utils.dfm import validate_and_price as real_val

        return real_val(part, method, config)

    from shared.workers.workbench_models import ManufacturingMethod

    if isinstance(method, ManufacturingMethod):
        method_str = method.value
    else:
        method_str = str(method)

    payload = {
        "script_path": "script.py",
        "method": method_str,
        "quantity": 1,  # Default
    }
    res = _call_heavy_worker("/benchmark/analyze", payload)
    return BenchmarkToolResponse.model_validate(res)


def submit_for_review(compound: Compound) -> bool:
    """Proxy for benchmark submission to the benchmark reviewer stage."""
    if _is_script_import_mode():
        return True
    if os.getenv("IS_HEAVY_WORKER"):
        from worker_heavy.utils.handover import submit_for_review as real_submit

        return real_submit(compound, reviewer_stage=AgentName.BENCHMARK_REVIEWER)

    # In light-worker execution (e.g., script runtime checks), avoid emitting
    # heavy-worker gate errors before prerequisites are present. The controller
    # runs the authoritative submit call after validate+simulate in sequence.
    if not Path("validation_results.json").exists():
        logger.info(
            "submit_for_review_deferred_missing_validation",
            extra={"path": "validation_results.json"},
        )
        return False
    if not Path("simulation_result.json").exists():
        logger.info(
            "submit_for_review_deferred_missing_simulation",
            extra={"path": "simulation_result.json"},
        )
        return False

    payload = {
        "script_path": "script.py",
        "reviewer_stage": AgentName.BENCHMARK_REVIEWER,
    }
    res = _call_heavy_worker("/benchmark/submit", payload)
    return BenchmarkToolResponse.model_validate(res).success


def refuse_plan(reason: str) -> bool:
    """Refuse the current plan."""
    if os.getenv("IS_HEAVY_WORKER"):
        from worker_heavy.utils.handover import refuse_plan as real_refuse

        return real_refuse(reason)

    refusal_data = PlanRefusal(
        reason=reason,
        timestamp=os.getenv("TIMESTAMP"),
        session_id=os.getenv("SESSION_ID", "default"),
    )
    with Path("refusal.json").open("w") as f:
        f.write(refusal_data.model_dump_json(indent=2))
    return True


# Re-export fasteners
