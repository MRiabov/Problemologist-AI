import os
from pathlib import Path
from typing import Any

import httpx
import structlog
from build123d import Compound
from pydantic import BaseModel

from shared.enums import AgentName
from shared.simulation.schemas import get_default_simulator_backend
from shared.utils.fasteners import HoleType as HoleType
from shared.utils.fasteners import fastener_hole as fastener_hole
from shared.workers.schema import BenchmarkToolResponse, PlanRefusal

logger = structlog.get_logger(__name__)
SCRIPT_IMPORT_MODE_ENV = "PROBLEMOLOGIST_SCRIPT_IMPORT_MODE"
SCRIPT_IMPORT_DEFERRED_MESSAGE = "Deferred during control-plane script import."
CONTROLLER_URL_ENV = "CONTROLLER_URL"
AGENT_NAME_ENV = "AGENT_NAME"

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


def _controller_base_url() -> str | None:
    url = os.getenv(CONTROLLER_URL_ENV)
    if not url:
        return None
    return url.rstrip("/")


def _script_agent_role() -> str:
    return os.getenv(AGENT_NAME_ENV, AgentName.ENGINEER_CODER.value)


def _call_controller_script_tool(action: str, payload: dict[str, Any]) -> dict | None:
    controller_url = _controller_base_url()
    if not controller_url:
        return None
    session_id = os.getenv("SESSION_ID", "default")
    url = f"{controller_url}/api/script-tools/{action.lstrip('/')}"
    headers = {"X-Session-ID": session_id}
    try:
        with httpx.Client(timeout=1000.0) as client:
            resp = client.post(url, json=payload, headers=headers)
            resp.raise_for_status()
            return resp.json()
    except Exception as e:
        logger.error(
            "controller_script_tool_failed",
            action=action,
            error=str(e),
            session_id=session_id,
        )
        raise


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

    controller_payload = {
        "script_path": "script.py",
        "agent_role": _script_agent_role(),
        "backend": kwargs.get("backend", get_default_simulator_backend()),
        "smoke_test_mode": kwargs.get("smoke_test_mode"),
    }
    controller_res = _call_controller_script_tool(
        "simulate",
        controller_payload,
    )
    if controller_res is not None:
        return BenchmarkToolResponse.model_validate(controller_res)

    # In non-controller contexts, fall back to the heavy worker for standalone
    # local tooling and worker-level tests.
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

    controller_payload = {
        "script_path": "script.py",
        "agent_role": _script_agent_role(),
    }
    controller_res = _call_controller_script_tool(
        "validate",
        controller_payload,
    )
    if controller_res is not None:
        parsed = BenchmarkToolResponse.model_validate(controller_res)
        return parsed.success, parsed.message

    # In non-controller contexts, fall back to the heavy worker for standalone
    # local tooling and worker-level tests.
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
    if os.getenv("IS_HEAVY_WORKER"):
        from worker_heavy.utils.handover import submit_for_review as real_submit

        return real_submit(compound, reviewer_stage=AgentName.BENCHMARK_REVIEWER)

    controller_payload = {
        "script_path": "script.py",
        "agent_role": _script_agent_role(),
        "reviewer_stage": AgentName.BENCHMARK_REVIEWER.value,
    }
    controller_res = _call_controller_script_tool(
        "submit",
        controller_payload,
    )
    if controller_res is not None:
        parsed = BenchmarkToolResponse.model_validate(controller_res)
        return parsed.success

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
