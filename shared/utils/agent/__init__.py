import os
import httpx
import logging
from typing import Any, Optional
from pathlib import Path

from build123d import Compound
from pydantic import BaseModel
from shared.workers.schema import BenchmarkToolResponse, PlanRefusal

logger = logging.getLogger(__name__)

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


def simulate(compound: Compound, **kwargs) -> BenchmarkToolResponse:
    """Proxy for heavy simulation."""
    if os.getenv("IS_HEAVY_WORKER"):
        from worker_heavy.utils.validation import simulate as real_simulate

        return real_simulate(compound, **kwargs)

    # In light worker, we call heavy worker
    payload = {"script_path": "script.py", **kwargs}
    res = _call_heavy_worker("/benchmark/simulate", payload)
    return BenchmarkToolResponse.model_validate(res)


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
    """Proxy for submission."""
    if os.getenv("IS_HEAVY_WORKER"):
        from worker_heavy.utils.handover import submit_for_review as real_submit

        return real_submit(compound)

    payload = {"script_path": "script.py"}
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
from shared.utils.fasteners import fastener_hole, HoleType
