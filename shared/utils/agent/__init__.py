import asyncio
import contextlib
import importlib.util
import math
import os
import sys
from enum import Enum
from pathlib import Path
from typing import Any
from uuid import uuid4

import httpx
import structlog
from build123d import Compound, Part
from pydantic import BaseModel

from shared.enums import AgentName
from shared.rendering import (
    export_preview_scene_bundle,
    materialize_preview_response,
    render_preview,
    select_single_preview_render_subdir,
)
from shared.script_contracts import authored_script_path_for_agent
from shared.simulation.schemas import get_default_simulator_backend
from shared.utils.fasteners import HoleType as HoleType
from shared.utils.fasteners import fastener_hole as fastener_hole
from shared.workers.loader import sys_path_context
from shared.workers.schema import (
    BenchmarkToolResponse,
    PlanRefusal,
    PreviewDesignRequest,
    PreviewDesignResponse,
    PreviewRenderingType,
)

logger = structlog.get_logger(__name__)
SCRIPT_IMPORT_MODE_ENV = "PROBLEMOLOGIST_SCRIPT_IMPORT_MODE"
SCRIPT_IMPORT_DEFERRED_MESSAGE = "Deferred during control-plane script import."
CONTROLLER_URL_ENV = "CONTROLLER_URL"
AGENT_NAME_ENV = "AGENT_NAME"

# --- Proxy Logic ---


def _call_heavy_worker(endpoint: str, payload: dict | BaseModel) -> dict:
    heavy_url = os.getenv("WORKER_HEAVY_URL", "http://worker-heavy:8002")
    session_id = os.getenv("SESSION_ID", "default")
    json_payload = payload.model_dump() if isinstance(payload, BaseModel) else payload
    agent_role = str(json_payload.get("agent_role") or _script_agent_role())
    stage = str(json_payload.get("reviewer_stage") or agent_role)

    url = f"{heavy_url.rstrip('/')}/{endpoint.lstrip('/')}"
    headers = {
        "X-Session-ID": session_id,
        "X-Agent-Role": agent_role,
        "X-Stage": stage,
    }
    if json_payload.get("reviewer_stage") is not None:
        headers["X-Reviewer-Stage"] = str(json_payload["reviewer_stage"])

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


def _worker_light_base_url() -> str | None:
    url = os.getenv("WORKER_LIGHT_URL")
    if not url:
        return None
    return url.rstrip("/")


def _script_agent_role() -> str:
    return os.getenv(AGENT_NAME_ENV, AgentName.ENGINEER_CODER.value)


def _workspace_script_path() -> str:
    workspace = _workspace_root()
    role_default = authored_script_path_for_agent(_script_agent_role())
    for candidate in (
        role_default,
        "solution_script.py",
        "benchmark_script.py",
        "script.py",
    ):
        if (workspace / candidate).exists():
            return candidate
    return role_default


def _workspace_root() -> Path:
    sessions_dir = os.getenv("WORKER_SESSIONS_DIR")
    session_id = os.getenv("SESSION_ID")
    if sessions_dir and session_id:
        return Path(sessions_dir) / session_id
    return Path.cwd()


def _preview_output_dir() -> Path:
    return (
        _workspace_root()
        / "renders"
        / select_single_preview_render_subdir(_workspace_root())
    )


def _load_module_from_path(module_path: Path) -> Any:
    if not module_path.exists():
        raise FileNotFoundError(f"Module not found at {module_path}")

    module_name = f"dynamic_preview_{module_path.stem}_{uuid4().hex}"
    with sys_path_context(str(module_path.parent)):
        spec = importlib.util.spec_from_file_location(module_name, str(module_path))
        if spec is None or spec.loader is None:
            raise RuntimeError(f"Could not load module spec for {module_path}")
        module = importlib.util.module_from_spec(spec)
        try:
            spec.loader.exec_module(module)
        finally:
            with contextlib.suppress(Exception):
                sys.modules.pop(module_name, None)
    return module


def objectives_geometry() -> Any:
    """Materialize benchmark objective geometry for the current workspace."""
    benchmark_script = _workspace_root() / "benchmark_script.py"
    module = _load_module_from_path(benchmark_script)
    factory = getattr(module, "objectives_geometry", None)
    if not callable(factory):
        raise AttributeError(
            "benchmark_script.py must define an objectives_geometry() function"
        )
    return factory()


def _default_reviewer_stage() -> str:
    """Infer the correct reviewer stage for the current workspace.

    The fallback no-render helper is used by both benchmark and engineer Codex
    workspaces. Routing everything to the benchmark reviewer silently produces
    the wrong manifest for engineer/electronics coder workspaces, so we infer
    the stage from the current agent role and workspace handoff files first.
    """

    role_value = _script_agent_role()
    role_to_stage: dict[str, str] = {
        AgentName.BENCHMARK_CODER.value: AgentName.BENCHMARK_REVIEWER.value,
        AgentName.BENCHMARK_REVIEWER.value: AgentName.BENCHMARK_REVIEWER.value,
        AgentName.ENGINEER_CODER.value: AgentName.ENGINEER_EXECUTION_REVIEWER.value,
        AgentName.ENGINEER_EXECUTION_REVIEWER.value: AgentName.ENGINEER_EXECUTION_REVIEWER.value,
        AgentName.ELECTRONICS_ENGINEER.value: AgentName.ELECTRONICS_REVIEWER.value,
        AgentName.ELECTRONICS_REVIEWER.value: AgentName.ELECTRONICS_REVIEWER.value,
    }
    if role_value in role_to_stage:
        return role_to_stage[role_value]

    workspace = Path.cwd()
    if (workspace / "assembly_definition.yaml").exists():
        return AgentName.ENGINEER_EXECUTION_REVIEWER.value
    if (workspace / "benchmark_assembly_definition.yaml").exists():
        return AgentName.BENCHMARK_REVIEWER.value
    return AgentName.BENCHMARK_REVIEWER.value


def _call_controller_script_tool(action: str, payload: dict[str, Any]) -> dict | None:
    controller_url = _controller_base_url()
    if not controller_url:
        return None
    session_id = os.getenv("SESSION_ID", "default")
    agent_role = str(payload.get("agent_role") or _script_agent_role())
    stage = str(payload.get("reviewer_stage") or agent_role)
    url = f"{controller_url}/api/script-tools/{action.lstrip('/')}"
    headers = {
        "X-Session-ID": session_id,
        "X-Agent-Role": agent_role,
        "X-Stage": stage,
    }
    if payload.get("reviewer_stage") is not None:
        headers["X-Reviewer-Stage"] = str(payload["reviewer_stage"])
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


def _call_controller_preview_tool(
    payload: PreviewDesignRequest,
) -> PreviewDesignResponse | None:
    controller_url = _controller_base_url()
    if not controller_url:
        return None
    session_id = os.getenv("SESSION_ID", "default")
    agent_role = _script_agent_role()
    url = f"{controller_url}/api/script-tools/preview"
    headers = {
        "X-Session-ID": session_id,
        "X-Agent-Role": agent_role,
        "X-Stage": agent_role,
    }
    try:
        with httpx.Client(timeout=1000.0) as client:
            resp = client.post(
                url, json=payload.model_dump(mode="json"), headers=headers
            )
            resp.raise_for_status()
            return PreviewDesignResponse.model_validate(resp.json())
    except Exception as exc:
        logger.error(
            "controller_preview_tool_failed",
            error=str(exc),
            session_id=session_id,
        )
        raise


def _call_worker_light_preview(
    payload: PreviewDesignRequest, *, session_id: str | None
) -> PreviewDesignResponse | None:
    worker_light_url = _worker_light_base_url()
    if not worker_light_url:
        return None

    try:
        with httpx.Client(timeout=300.0) as client:
            resp = client.post(
                f"{worker_light_url}/benchmark/preview",
                json=payload.model_dump(mode="json"),
                headers={"X-Session-ID": session_id or "default"},
            )
            resp.raise_for_status()
            return PreviewDesignResponse.model_validate(resp.json())
    except Exception as exc:
        logger.error(
            "worker_light_preview_failed",
            error=str(exc),
            session_id=session_id,
        )
        raise


# --- Agent Utils ---


def _is_script_import_mode() -> bool:
    return os.getenv(SCRIPT_IMPORT_MODE_ENV) == "1"


def _normalize_signature_value(value: Any, *, depth: int = 0) -> Any:
    """Normalize build123d objects into a tolerance-aware signature."""
    if depth > 6:
        return repr(value)
    if value is None or isinstance(value, (str, bool, int)):
        return value
    if isinstance(value, Enum):
        return value.name
    if isinstance(value, float):
        return round(value, 3) if math.isfinite(value) else repr(value)
    if all(hasattr(value, attr) for attr in ("X", "Y", "Z")):
        return {
            "x": round(float(getattr(value, "X")), 3),
            "y": round(float(getattr(value, "Y")), 3),
            "z": round(float(getattr(value, "Z")), 3),
        }
    if all(hasattr(value, attr) for attr in ("x", "y", "z")):
        return {
            "x": round(float(getattr(value, "x")), 3),
            "y": round(float(getattr(value, "y")), 3),
            "z": round(float(getattr(value, "z")), 3),
        }
    if isinstance(value, (list, tuple)):
        return [_normalize_signature_value(item, depth=depth + 1) for item in value]
    if isinstance(value, dict):
        return {
            str(key): _normalize_signature_value(item, depth=depth + 1)
            for key, item in value.items()
            if not str(key).startswith("_")
        }
    if hasattr(value, "__dict__"):
        data: dict[str, Any] = {"__type__": type(value).__name__}
        for key, item in vars(value).items():
            if key.startswith("_") or key in {"wrapped", "topo_parent", "joints"}:
                continue
            if callable(item):
                continue
            data[key] = _normalize_signature_value(item, depth=depth + 1)
        return data
    return repr(value)


def _component_semantic_signature(component: Any) -> dict[str, Any]:
    """Build a semantic signature for a build123d component."""
    signature: dict[str, Any] = {
        "type": type(component).__name__,
        "label": _normalize_signature_value(getattr(component, "label", None)),
    }

    public_fields = {
        key: _normalize_signature_value(value)
        for key, value in vars(component).items()
        if not key.startswith("_")
        and key not in {"wrapped", "topo_parent", "joints"}
        and not callable(value)
    }
    if public_fields:
        signature["fields"] = public_fields

    children = getattr(component, "children", ())
    if isinstance(children, tuple) and children:
        signature["children"] = [
            _component_semantic_signature(child) for child in children
        ]

    metrics: dict[str, Any] = {}
    for metric_name, attr_name in (
        ("solids_count", "solids"),
        ("faces_count", "faces"),
        ("edges_count", "edges"),
        ("wires_count", "wires"),
    ):
        if hasattr(component, attr_name):
            try:
                metrics[metric_name] = len(list(getattr(component, attr_name)()))
            except Exception:
                continue

    if hasattr(component, "bounding_box"):
        try:
            metrics["bbox"] = _normalize_signature_value(component.bounding_box())
        except Exception:
            pass

    if hasattr(component, "volume"):
        try:
            metrics["volume"] = _normalize_signature_value(component.volume)
        except Exception:
            pass

    if metrics:
        signature["metrics"] = metrics

    return signature


def _signature_summary(signature: dict[str, Any]) -> dict[str, Any]:
    metrics = signature.get("metrics", {}) if isinstance(signature, dict) else {}
    return {
        "type": signature.get("type") if isinstance(signature, dict) else None,
        "label": signature.get("label") if isinstance(signature, dict) else None,
        "children": len(signature.get("children", []))
        if isinstance(signature, dict)
        else None,
        "solids": metrics.get("solids_count"),
        "faces": metrics.get("faces_count"),
        "edges": metrics.get("edges_count"),
        "wires": metrics.get("wires_count"),
        "volume": metrics.get("volume"),
    }


def _load_script_component(script_path: str | Path | None = None) -> Any:
    from shared.workers.loader import load_component_from_script

    script_path = script_path or _workspace_script_path()
    return load_component_from_script(
        script_path=script_path,
        session_root=_workspace_root(),
    )


def _ensure_component_matches_workspace_script(
    component: Any, *, script_path: str | Path | None = None
) -> tuple[bool, str | None]:
    script_path = script_path or _workspace_script_path()
    try:
        script_component = _load_script_component(script_path)
    except Exception as exc:
        return (
            False,
            f"Unable to load persisted script {script_path!s} for validation: {exc}",
        )

    live_signature = _component_semantic_signature(component)
    script_signature = _component_semantic_signature(script_component)
    if live_signature != script_signature:
        return (
            False,
            "Workspace component does not match persisted script semantic "
            "signature. Persist the authored build script before validating. "
            f"live={_signature_summary(live_signature)} "
            f"script={_signature_summary(script_signature)}",
        )
    return True, None


def simulate(compound: Compound, **kwargs) -> BenchmarkToolResponse:
    """Proxy for heavy simulation."""
    if _is_script_import_mode():
        return BenchmarkToolResponse(
            success=True,
            message=SCRIPT_IMPORT_DEFERRED_MESSAGE,
        )
    script_path = kwargs.get("script_path") or _workspace_script_path()
    matches, mismatch_message = _ensure_component_matches_workspace_script(
        compound, script_path=script_path
    )
    if not matches:
        return BenchmarkToolResponse(success=False, message=mismatch_message or "")
    if os.getenv("IS_HEAVY_WORKER"):
        from worker_heavy.utils.validation import simulate as real_simulate

        if "output_dir" in kwargs and kwargs["output_dir"] is not None:
            kwargs = {**kwargs, "output_dir": Path(kwargs["output_dir"])}
        kwargs = {**kwargs, "script_path": script_path}
        return real_simulate(compound, **kwargs)

    controller_payload = {
        "script_path": str(script_path),
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
    payload = {"script_path": script_path, **kwargs}
    res = _call_heavy_worker("/benchmark/simulate", payload)
    return BenchmarkToolResponse.model_validate(res)


def validate(compound: Compound, **kwargs) -> tuple[bool, str | None]:
    """Proxy for benchmark geometric validation."""
    if _is_script_import_mode():
        return True, SCRIPT_IMPORT_DEFERRED_MESSAGE
    script_path = kwargs.get("script_path") or _workspace_script_path()
    matches, mismatch_message = _ensure_component_matches_workspace_script(
        compound, script_path=script_path
    )
    if not matches:
        return False, mismatch_message
    if os.getenv("IS_HEAVY_WORKER"):
        from worker_heavy.utils.validation import validate as real_validate

        if "output_dir" in kwargs and kwargs["output_dir"] is not None:
            kwargs = {**kwargs, "output_dir": Path(kwargs["output_dir"])}
        kwargs = {**kwargs, "script_path": script_path}
        return real_validate(compound, **kwargs)

    controller_payload = {
        "script_path": str(script_path),
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
    payload = {"script_path": script_path, **kwargs}
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
    script_path = _workspace_script_path()
    matches, mismatch_message = _ensure_component_matches_workspace_script(
        compound, script_path=script_path
    )
    if not matches:
        logger.info(
            "submit_for_review_deferred_script_mismatch",
            message=mismatch_message,
        )
        return False
    reviewer_stage = _default_reviewer_stage()
    episode_id = os.getenv("EPISODE_ID") or None
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

        return real_submit(
            compound,
            reviewer_stage=reviewer_stage,
            episode_id=episode_id,
            script_path=script_path,
        )

    controller_payload = {
        "script_path": script_path,
        "agent_role": _script_agent_role(),
        "reviewer_stage": reviewer_stage,
        "episode_id": episode_id,
    }
    controller_res = _call_controller_script_tool(
        "submit",
        controller_payload,
    )
    if controller_res is not None:
        parsed = BenchmarkToolResponse.model_validate(controller_res)
        return parsed.success

    payload = {
        "script_path": script_path,
        "reviewer_stage": reviewer_stage,
        "episode_id": episode_id,
    }
    res = _call_heavy_worker("/benchmark/submit", payload)
    return BenchmarkToolResponse.model_validate(res).success


class _PreviewResponseProxy:
    def __init__(self, factory):
        self._factory = factory
        self._resolved: PreviewDesignResponse | None = None

    async def _resolve_async(self) -> PreviewDesignResponse:
        if self._resolved is None:
            self._resolved = await self._factory()
        return self._resolved

    def _resolve_sync(self) -> PreviewDesignResponse:
        if self._resolved is not None:
            return self._resolved
        try:
            asyncio.get_running_loop()
        except RuntimeError:
            self._resolved = asyncio.run(self._factory())
            return self._resolved
        raise RuntimeError(
            "preview() must be awaited when called from an async context"
        )

    def __await__(self):
        return self._resolve_async().__await__()

    def __getattr__(self, item):
        return getattr(self._resolve_sync(), item)

    def __bool__(self):
        return bool(self._resolve_sync())

    def __repr__(self):
        if self._resolved is not None:
            return repr(self._resolved)
        return "<_PreviewResponseProxy unresolved>"


async def _preview_async(
    component: Part | Compound,
    orbit_pitch: float | list[float] = 45,
    orbit_yaw: float | list[float] = 45,
    rgb: bool | None = None,
    depth: bool | None = None,
    segmentation: bool | None = None,
    rendering_type: PreviewRenderingType | str | None = None,
) -> PreviewDesignResponse:
    """Render an on-demand preview for a live build123d component."""
    if _is_script_import_mode():
        return PreviewDesignResponse(
            success=True,
            status_text=SCRIPT_IMPORT_DEFERRED_MESSAGE,
            message=SCRIPT_IMPORT_DEFERRED_MESSAGE,
            rendering_type=(
                PreviewRenderingType(str(rendering_type))
                if rendering_type is not None
                else PreviewRenderingType.RGB
            ),
        )

    preview_scene_bundle = export_preview_scene_bundle(
        component,
        objectives=None,
        workspace_root=_workspace_root(),
    )
    preview_request = PreviewDesignRequest(
        script_path="preview_scene.json",
        bundle_base64=preview_scene_bundle,
        orbit_pitch=orbit_pitch,
        orbit_yaw=orbit_yaw,
        rgb=rgb,
        depth=depth,
        segmentation=segmentation,
        rendering_type=(
            PreviewRenderingType(str(rendering_type))
            if rendering_type is not None
            else None
        ),
    )
    response = _call_worker_light_preview(
        preview_request, session_id=os.getenv("SESSION_ID")
    )
    if response is None:
        response = _call_controller_preview_tool(preview_request)
    if response is None:
        response = render_preview(
            bundle_base64=preview_scene_bundle,
            script_path="preview_scene.json",
            orbit_pitch=orbit_pitch,
            orbit_yaw=orbit_yaw,
            rgb=rgb,
            depth=depth,
            segmentation=segmentation,
            rendering_type=(
                PreviewRenderingType(str(rendering_type))
                if rendering_type is not None
                else None
            ),
            session_id=os.getenv("SESSION_ID"),
        )
    if not response.success:
        return response

    workspace_root = _workspace_root()
    preview_output_dir = _preview_output_dir()
    for candidate in (response.artifact_path, response.image_path):
        if not candidate:
            continue
        candidate_path = Path(candidate)
        if candidate_path.parent == Path("."):
            continue
        if candidate_path.is_absolute():
            preview_output_dir = candidate_path.parent
        else:
            preview_output_dir = workspace_root / candidate_path.parent
        break

    materialized_path = materialize_preview_response(response, preview_output_dir)
    if materialized_path is not None:
        local_path = str(materialized_path.relative_to(workspace_root))
        response.image_path = local_path
        response.artifact_path = local_path
    elif response.artifact_path is None and response.image_path is not None:
        response.artifact_path = response.image_path
    if response.manifest_path is None:
        response.manifest_path = str(Path("renders") / "render_manifest.json")
    if response.pitch is None:
        if isinstance(orbit_pitch, float):
            response.pitch = orbit_pitch
        elif isinstance(orbit_pitch, list) and len(orbit_pitch) == 1:
            response.pitch = orbit_pitch[0]
    if response.yaw is None:
        if isinstance(orbit_yaw, float):
            response.yaw = orbit_yaw
        elif isinstance(orbit_yaw, list) and len(orbit_yaw) == 1:
            response.yaw = orbit_yaw[0]
    if response.status_text is None:
        response.status_text = response.message or "Preview generated successfully"
    return response


def preview(
    component: Part | Compound,
    orbit_pitch: float | list[float] = 45,
    orbit_yaw: float | list[float] = 45,
    rgb: bool | None = None,
    depth: bool | None = None,
    segmentation: bool | None = None,
    rendering_type: PreviewRenderingType | str | None = None,
) -> _PreviewResponseProxy:
    return _PreviewResponseProxy(
        lambda: _preview_async(
            component,
            orbit_pitch=orbit_pitch,
            orbit_yaw=orbit_yaw,
            rgb=rgb,
            depth=depth,
            segmentation=segmentation,
            rendering_type=rendering_type,
        )
    )


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
