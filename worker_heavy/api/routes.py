import asyncio
import base64
import contextlib
import io
import multiprocessing
import tarfile
import tempfile
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
from typing import Any

import structlog
from fastapi import (
    APIRouter,
    Depends,
    Header,
    HTTPException,
)

from shared.workers.persistence import (
    collect_and_cleanup_events,
    record_validation_result,
)
from shared.workers.schema import (
    AnalyzeRequest,
    BenchmarkToolRequest,
    BenchmarkToolResponse,
    ElectronicsValidationRequest,
    PreviewDesignRequest,
    PreviewDesignResponse,
)
from shared.workers.workbench_models import WorkbenchResult
from shared.workers.loader import load_component_from_script
from worker_heavy.utils.topology import analyze_component
from worker_heavy.utils.validation import validate_fem_manufacturability
from worker_heavy.utils import (
    simulate,
    submit_for_review,
    validate,
    validate_circuit as utils_validate_circuit,
)
from worker_heavy.utils.preview import preview_design

logger = structlog.get_logger(__name__)
heavy_router = APIRouter()

# In heavy worker, we still need a way to access the filesystem
# This usually comes from the bundled session
from shared.workers.filesystem.router import create_filesystem_router


async def get_router(x_session_id: str = Header(...)):
    """Dependency to create a filesystem router for the current session."""
    try:
        from worker_heavy.config import settings

        return create_filesystem_router(
            session_id=x_session_id, base_dir=settings.sessions_dir
        )
    except Exception as e:
        logger.error("router_creation_failed", error=str(e))
        raise HTTPException(status_code=500, detail="Failed to initialize filesystem")


def _collect_events(fs_router, root: Path | None = None) -> list[dict[str, Any]]:
    """Read and delete events.jsonl from the workspace."""
    search_root = root or fs_router.local_backend.root
    return collect_and_cleanup_events(search_root)


@contextlib.contextmanager
def bundle_context(bundle_base64: str | None, default_root: Path):
    """Context manager to optionally extract a workspace bundle."""
    if not bundle_base64:
        yield default_root
        return

    with tempfile.TemporaryDirectory() as tmpdir:
        tmp_root = Path(tmpdir)
        try:
            bundle_bytes = base64.b64decode(bundle_base64)
            # Use system tar for robustness against metadata/comparison bugs
            import subprocess

            with tempfile.NamedTemporaryFile(suffix=".tar.gz", delete=False) as tf:
                tf.write(bundle_bytes)
                tf_path = tf.name
            try:
                # -z: gzip, -x: extract, -f: file, -C: directory
                # --no-same-owner: avoid permission issues
                # --no-same-permissions: avoid permission issues
                subprocess.run(
                    ["tar", "-zxf", tf_path, "-C", str(tmp_root), "--no-same-owner"],
                    check=True,
                    capture_output=True,
                    text=True,
                )
            except subprocess.CalledProcessError as spe:
                logger.error("tar_subprocess_failed", stderr=spe.stderr)
                raise RuntimeError(f"tar extraction failed: {spe.stderr}")
            finally:
                if Path(tf_path).exists():
                    Path(tf_path).unlink()

            logger.info(
                "bundle_extracted_via_system_tar", path=str(tmp_root), session_id=None
            )
            yield tmp_root
        except Exception as e:
            import traceback

            tb = traceback.format_exc()
            logger.error("bundle_extraction_failed", error=str(e), traceback=tb)
            raise HTTPException(
                status_code=400, detail=f"Failed to extract bundle: {e}"
            ) from e


# Global lock to ensure only one simulation/render runs at a time cross-session
HEAVY_OPERATION_LOCK = asyncio.Lock()
SIMULATION_QUEUE_DEPTH = 0


def _init_genesis_worker():
    """Pre-warm Genesis in the worker process."""
    try:
        import os

        # Force headless mode for pyglet (used by genesis/pyrender)
        os.environ["PYGLET_HEADLESS"] = "1"
        # Force EGL platform for headless rendering
        if "PYOPENGL_PLATFORM" not in os.environ:
            os.environ["PYOPENGL_PLATFORM"] = "egl"

        import genesis as gs
        import torch
        import time

        # Basic init
        has_gpu = torch.cuda.is_available()
        gs.init(backend=gs.gpu if has_gpu else gs.cpu, logging_level="warning")
        time.sleep(1.0)  # Give it a second to stabilize

        # Build a tiny scene to trigger kernel compilation
        scene = gs.Scene(show_viewer=False)
        scene.add_entity(gs.morphs.Plane())
        scene.build()

        logger.info(
            "genesis_worker_prewarmed", pid=multiprocessing.current_process().pid
        )
    except Exception as e:
        logger.warning("genesis_worker_prewarm_failed", error=str(e))


SIMULATION_EXECUTOR = ProcessPoolExecutor(
    max_workers=1,
    max_tasks_per_child=None,
    mp_context=multiprocessing.get_context("spawn"),
    initializer=_init_genesis_worker,
)


async def run_simulation_task(
    script_path,
    root,
    script_content,
    smoke_test_mode,
    backend_type,
    x_session_id,
    particle_budget,
):
    """Helper to run simulation in executor, allows easier mocking in tests."""
    from worker_heavy.utils.validation import simulate_subprocess

    loop = asyncio.get_running_loop()
    return await loop.run_in_executor(
        SIMULATION_EXECUTOR,
        simulate_subprocess,
        script_path,
        root,
        script_content,
        root,
        smoke_test_mode,
        backend_type,
        x_session_id,
        particle_budget,
    )


@heavy_router.post("/benchmark/simulate", response_model=BenchmarkToolResponse)
async def api_simulate(
    request: BenchmarkToolRequest,
    x_session_id: str = Header(...),
    fs_router=Depends(get_router),
):
    """Physics-backed stability check in isolated session."""
    global SIMULATION_QUEUE_DEPTH
    SIMULATION_QUEUE_DEPTH += 1
    wait_pos = SIMULATION_QUEUE_DEPTH
    try:
        from shared.simulation.schemas import SimulatorBackendType

        backend_type = request.backend
        if isinstance(backend_type, str):
            backend_type = SimulatorBackendType(backend_type)

        async with HEAVY_OPERATION_LOCK:
            with bundle_context(
                request.bundle_base64, fs_router.local_backend.root
            ) as root:
                result = await run_simulation_task(
                    str(root / request.script_path)
                    if request.bundle_base64
                    else fs_router.local_backend._resolve(request.script_path),
                    root,
                    request.script_content,
                    request.smoke_test_mode,
                    backend_type,
                    x_session_id,
                    request.particle_budget,
                )

                summary = result.summary
                if wait_pos > 1:
                    summary = (
                        f"{summary} (Queued: wait position {wait_pos})"
                        if summary
                        else f"Queued: wait position {wait_pos}"
                    )

                events = _collect_events(fs_router, root=root)
                return BenchmarkToolResponse(
                    success=result.success,
                    message=summary,
                    confidence=result.confidence,
                    artifacts={
                        "render_paths": result.render_paths,
                        "mjcf_content": result.mjcf_content,
                        "stress_summaries": [
                            s.model_dump() for s in result.stress_summaries
                        ],
                        "fluid_metrics": [m.model_dump() for m in result.fluid_metrics],
                        "failure": result.failure.model_dump()
                        if result.failure
                        else None,
                    },
                    events=events,
                )

    except Exception as e:
        logger.error("api_benchmark_simulate_failed", error=str(e))
        return BenchmarkToolResponse(
            success=False,
            message=str(e),
            artifacts={
                "failure": {"reason": "PHYSICS_INSTABILITY", "detail": str(e)},
            },
        )
    finally:
        SIMULATION_QUEUE_DEPTH -= 1


@heavy_router.post("/benchmark/validate", response_model=BenchmarkToolResponse)
async def api_validate(
    request: BenchmarkToolRequest,
    x_session_id: str = Header(...),
    fs_router=Depends(get_router),
):
    """Geometric validity check in isolated session."""
    global SIMULATION_QUEUE_DEPTH
    SIMULATION_QUEUE_DEPTH += 1
    try:
        async with HEAVY_OPERATION_LOCK:
            with bundle_context(
                request.bundle_base64, fs_router.local_backend.root
            ) as root:
                component = load_component_from_script(
                    script_path=root / request.script_path
                    if request.bundle_base64
                    else fs_router.local_backend._resolve(request.script_path),
                    session_root=root,
                    script_content=request.script_content,
                )
                is_valid, message = await asyncio.to_thread(
                    validate,
                    component,
                    output_dir=root,
                    session_id=x_session_id,
                    smoke_test_mode=request.smoke_test_mode,
                    particle_budget=request.particle_budget,
                )

                fem_valid, fem_msg = await asyncio.to_thread(
                    validate_fem_manufacturability,
                    component,
                    root,
                )
                if is_valid and not fem_valid:
                    is_valid = False
                    message = (message + "; " + fem_msg) if message else fem_msg

                record_validation_result(root, is_valid, message)

                events = _collect_events(fs_router, root=root)
                return BenchmarkToolResponse(
                    success=is_valid,
                    message=message or "Validation successful",
                    events=events,
                    artifacts={
                        "failure": {
                            "reason": "VALIDATION_FAILED",
                            "detail": message,
                        }
                        if not is_valid
                        else None
                    },
                )

    except Exception as e:
        logger.error("api_benchmark_validate_failed", error=str(e))
        return BenchmarkToolResponse(
            success=False,
            message=str(e),
            artifacts={
                "failure": {"reason": "VALIDATION_FAILED", "detail": str(e)},
            },
        )
    finally:
        SIMULATION_QUEUE_DEPTH -= 1


@heavy_router.post("/benchmark/validate_circuit", response_model=BenchmarkToolResponse)
async def api_validate_circuit(
    request: ElectronicsValidationRequest,
    x_session_id: str = Header(...),
):
    """Run SPICE validation on the provided electronics section."""
    try:
        from shared.circuit_builder import build_circuit_from_section
        from shared.pyspice_utils import validate_circuit

        circuit = build_circuit_from_section(request.section)
        res = validate_circuit(
            circuit, request.section.power_supply, section=request.section
        )

        return BenchmarkToolResponse(
            success=res.valid,
            message="; ".join(res.errors) if not res.valid else "Circuit is valid",
            artifacts={
                "circuit_validation_result": res.model_dump(),
                "failure": {
                    "reason": "VALIDATION_FAILED",
                    "detail": "; ".join(res.errors),
                }
                if not res.valid
                else None,
            },
        )
    except Exception as e:
        logger.error("api_validate_circuit_failed", error=str(e))
        return BenchmarkToolResponse(
            success=False,
            message=str(e),
            artifacts={
                "failure": {"reason": "VALIDATION_FAILED", "detail": str(e)},
            },
        )


@heavy_router.post("/benchmark/analyze", response_model=WorkbenchResult)
async def api_analyze(
    request: AnalyzeRequest,
    x_session_id: str = Header(...),
    fs_router=Depends(get_router),
):
    """Component analysis (topology, material, weight) in isolated session."""
    global SIMULATION_QUEUE_DEPTH
    SIMULATION_QUEUE_DEPTH += 1
    try:
        async with HEAVY_OPERATION_LOCK:
            with bundle_context(
                request.bundle_base64, fs_router.local_backend.root
            ) as root:
                component = load_component_from_script(
                    script_path=root / request.script_path
                    if request.bundle_base64
                    else fs_router.local_backend._resolve(request.script_path),
                    session_root=root,
                    script_content=request.script_content,
                )
                result = await asyncio.to_thread(
                    analyze_component,
                    component,
                    output_dir=root,
                    method=request.method,
                    quantity=request.quantity,
                )
                return result
    except Exception as e:
        logger.error("api_benchmark_analyze_failed", error=str(e))
        return WorkbenchResult(
            is_manufacturable=False, unit_cost=0.0, violations=[str(e)]
        )
    finally:
        SIMULATION_QUEUE_DEPTH -= 1


@heavy_router.post("/benchmark/preview", response_model=PreviewDesignResponse)
async def api_preview(
    request: PreviewDesignRequest,
    x_session_id: str = Header(...),
    fs_router=Depends(get_router),
):
    """Render a preview of the CAD design from specified camera angles."""
    global SIMULATION_QUEUE_DEPTH
    SIMULATION_QUEUE_DEPTH += 1
    try:
        async with HEAVY_OPERATION_LOCK:
            with bundle_context(
                request.bundle_base64, fs_router.local_backend.root
            ) as root:
                component = load_component_from_script(
                    script_path=root / request.script_path
                    if request.bundle_base64
                    else fs_router.local_backend._resolve(request.script_path),
                    session_root=root,
                    script_content=request.script_content,
                )

                image_path = await asyncio.to_thread(
                    preview_design,
                    component,
                    pitch=request.pitch,
                    yaw=request.yaw,
                    output_dir=root / "renders",
                )
                events = _collect_events(fs_router, root=root)

                return PreviewDesignResponse(
                    success=True,
                    message="Preview generated successfully",
                    image_path=str(image_path.relative_to(root)),
                    events=events,
                )

    except Exception as e:
        logger.error("api_benchmark_preview_failed", error=str(e))
        return PreviewDesignResponse(success=False, message=str(e))
    finally:
        SIMULATION_QUEUE_DEPTH -= 1


@heavy_router.post("/benchmark/build", response_model=BenchmarkToolResponse)
async def api_build(
    request: PreviewDesignRequest,
    x_session_id: str = Header(...),
    fs_router=Depends(get_router),
):
    """Rebuild simulation assets (GLB) from source without running full simulation."""
    global SIMULATION_QUEUE_DEPTH
    SIMULATION_QUEUE_DEPTH += 1
    try:
        async with HEAVY_OPERATION_LOCK:
            with bundle_context(
                request.bundle_base64, fs_router.local_backend.root
            ) as root:
                component = load_component_from_script(
                    script_path=root / request.script_path
                    if request.bundle_base64
                    else fs_router.local_backend._resolve(request.script_path),
                    session_root=root,
                    script_content=request.script_content,
                )

                from worker_heavy.simulation.factory import get_simulation_builder

                builder = get_simulation_builder(root)

                scene_path = await asyncio.to_thread(
                    builder.build_from_assembly,
                    component,
                    smoke_test_mode=request.smoke_test_mode,
                )

                events = _collect_events(fs_router, root=root)
                return BenchmarkToolResponse(
                    success=True,
                    message=f"Assets rebuilt. Scene saved to {scene_path.name}",
                    artifacts={"scene_path": str(scene_path.relative_to(root))},
                    events=events,
                )

    except Exception as e:
        logger.error("api_benchmark_build_failed", error=str(e))
        return BenchmarkToolResponse(success=False, message=str(e))
    finally:
        SIMULATION_QUEUE_DEPTH -= 1


@heavy_router.post("/benchmark/submit", response_model=BenchmarkToolResponse)
async def api_submit(
    request: BenchmarkToolRequest,
    fs_router=Depends(get_router),
):
    """Handover to reviewer in isolated session (moved to heavy due to DFM dependencies)."""
    try:
        with bundle_context(
            request.bundle_base64, fs_router.local_backend.root
        ) as root:
            component = load_component_from_script(
                script_path=root / request.script_path
                if request.bundle_base64
                else fs_router.local_backend._resolve(request.script_path),
                session_root=root,
                script_content=request.script_content,
            )
            success = submit_for_review(component, cwd=root)
            events = _collect_events(fs_router, root=root)
            return BenchmarkToolResponse(
                success=success,
                message="Handover complete" if success else "Handover failed",
                events=events,
            )

    except Exception as e:
        logger.error("api_benchmark_submit_failed", error=str(e))
        return BenchmarkToolResponse(success=False, message=str(e))
