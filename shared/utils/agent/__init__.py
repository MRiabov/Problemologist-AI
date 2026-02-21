import os
import httpx
import json
import logging
from pathlib import Path
from typing import Any, Optional

from build123d import Compound, Part

logger = logging.getLogger(__name__)

# --- Proxy Logic ---

def _call_heavy_worker(endpoint: str, payload: dict) -> dict:
    heavy_url = os.getenv("WORKER_HEAVY_URL", "http://worker-heavy:8002")
    session_id = os.getenv("SESSION_ID", "default")

    url = f"{heavy_url.rstrip('/')}/{endpoint.lstrip('/')}"
    headers = {"X-Session-ID": session_id}

    try:
        with httpx.Client(timeout=300.0) as client:
            resp = client.post(url, json=payload, headers=headers)
            resp.raise_for_status()
            return resp.json()
    except Exception as e:
        logger.error(f"Heavy worker call failed: {e}")
        return {"success": False, "message": str(e)}

# --- Agent Utils ---

def simulate(compound: Compound, **kwargs) -> Any:
    """Proxy for heavy simulation."""
    if os.getenv("IS_HEAVY_WORKER"):
        from worker_heavy.utils.validation import simulate as real_simulate
        return real_simulate(compound, **kwargs)

    # In light worker, we call heavy worker
    # We assume script.py is what we want to simulate
    payload = {
        "script_path": "script.py",
        **kwargs
    }
    res = _call_heavy_worker("/benchmark/simulate", payload)

    # Wrap result in a SimpleNamespace or similar to match expected API
    from types import SimpleNamespace
    return SimpleNamespace(
        success=res.get("success", False),
        summary=res.get("message", ""),
        artifacts=res.get("artifacts", {})
    )

def validate_and_price(part: Any, method: Any = None, config: Any = None) -> Any:
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
        "quantity": 1 # Default
    }
    res = _call_heavy_worker("/benchmark/analyze", payload)

    from types import SimpleNamespace
    return SimpleNamespace(
        is_manufacturable=res.get("is_manufacturable", False),
        unit_cost=res.get("unit_cost", 0.0),
        violations=res.get("violations", []),
        weight_g=res.get("weight_g", 0.0)
    )

def submit_for_review(compound: Compound) -> bool:
    """Proxy for submission."""
    if os.getenv("IS_HEAVY_WORKER"):
        from worker_heavy.utils.handover import submit_for_review as real_submit
        return real_submit(compound)

    payload = {"script_path": "script.py"}
    res = _call_heavy_worker("/benchmark/submit", payload)
    return res.get("success", False)

def refuse_plan(
    reason: str,
    current_price: float | None = None,
    current_weight: float | None = None,
) -> bool:
    """Refuse the current plan."""
    if os.getenv("IS_HEAVY_WORKER"):
        from worker_heavy.utils.handover import refuse_plan as real_refuse
        return real_refuse(reason, current_price=current_price, current_weight=current_weight)

    # Emit observability event even in light worker
    from shared.observability.events import emit_event
    from shared.observability.schemas import EscalationRequestEvent

    emit_event(
        EscalationRequestEvent(
            reason=reason,
            current_price=current_price,
            current_weight=current_weight,
        )
    )

    import json
    refusal_data = {
        "status": "plan_refused",
        "reason": reason,
        "current_price": current_price,
        "current_weight": current_weight,
        "timestamp": os.getenv("TIMESTAMP"),
        "session_id": os.getenv("SESSION_ID", "default"),
    }
    with open("refusal.json", "w") as f:
        json.dump(refusal_data, f)
    return True

# Re-export fasteners
from shared.utils.fasteners import fastener_hole, HoleType
