import asyncio
import multiprocessing
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
    PreviewDesignRequest,
    PreviewDesignResponse,
)
from shared.workers.workbench_models import WorkbenchResult
from shared.workers.loader import load_component_from_script
from worker_heavy.utils.topology import analyze_component
from worker_heavy.utils.validation import validate_fem_manufacturability
from worker_heavy.utils import simulate, submit_for_review, validate
from worker_heavy.utils.preview import preview_design

logger = structlog.get_logger(__name__)
heavy_router = APIRouter()

# In heavy worker, we still need a way to access the filesystem
# This usually comes from the bundled session
from shared.workers.filesystem.router import create_filesystem_router

async def get_router(x_session_id: str = Header(...)):
    """Dependency to create a filesystem router for the current session."""
    try:
        return create_filesystem_router(session_id=x_session_id)
    except Exception as e:
        logger.error("router_creation_failed", error=str(e))
        raise HTTPException(status_code=500, detail="Failed to initialize filesystem")


def _collect_events(fs_router) -> list[dict[str, Any]]:
    """Read and delete events.jsonl from the workspace."""
    return collect_and_cleanup_events(fs_router.local_backend.root)


# Global lock to ensure only one simulation/render runs at a time cross-session
HEAVY_OPERATION_LOCK = asyncio.Lock()
SIMULATION_QUEUE_DEPTH = 0


def _init_genesis_worker():
    """Pre-warm Genesis in the worker process."""
    try:
        import genesis as gs
        import torch

        # Basic init
        has_gpu = torch.cuda.is_available()
        gs.init(backend=gs.gpu if has_gpu else gs.cpu, logging_level="warning")

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

        result = await run_simulation_task(
            fs_router.local_backend._resolve(request.script_path),
            fs_router.local_backend.root,
            request.script_content,
            request.smoke_test_mode,
            backend_type,
            x_session_id,
            request.particle_budget,
        )

        summary = result.summary
        if wait_pos > 1:
            summary = f"{summary} (Queued: wait position {wait_pos})" if summary else f"Queued: wait position {wait_pos}"

        events = _collect_events(fs_router)
        return BenchmarkToolResponse(
            success=result.success,
            message=summary,
            confidence=result.confidence,
            artifacts={
                "render_paths": result.render_paths,
                "mjcf_content": result.mjcf_content,
                "stress_summaries": [s.model_dump() for s in result.stress_summaries],
                "fluid_metrics": [m.model_dump() for m in result.fluid_metrics],
            },
            events=events,
        )

    except Exception as e:
        logger.error("api_benchmark_simulate_failed", error=str(e))
        return BenchmarkToolResponse(success=False, message=str(e))
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
            component = load_component_from_script(
                script_path=fs_router.local_backend._resolve(request.script_path),
                session_root=fs_router.local_backend.root,
                script_content=request.script_content,
            )
            is_valid, message = await asyncio.to_thread(
                validate,
                component,
                output_dir=fs_router.local_backend.root,
                session_id=x_session_id,
                smoke_test_mode=request.smoke_test_mode,
                particle_budget=request.particle_budget,
            )

            fem_valid, fem_msg = await asyncio.to_thread(
                validate_fem_manufacturability,
                component,
                fs_router.local_backend.root,
            )
            if is_valid and not fem_valid:
                is_valid = False
                message = (message + "; " + fem_msg) if message else fem_msg

        record_validation_result(fs_router.local_backend.root, is_valid, message)

        events = _collect_events(fs_router)
        return BenchmarkToolResponse(
            success=is_valid,
            message=message or "Validation successful",
            events=events,
        )

    except Exception as e:
        logger.error("api_benchmark_validate_failed", error=str(e))
        return BenchmarkToolResponse(success=False, message=str(e))
    finally:
        SIMULATION_QUEUE_DEPTH -= 1


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
            component = load_component_from_script(
                script_path=fs_router.local_backend._resolve(request.script_path),
                session_root=fs_router.local_backend.root,
                script_content=request.script_content,
            )
            result = await asyncio.to_thread(
                analyze_component,
                component,
                output_dir=fs_router.local_backend.root,
            )
            return result
    except Exception as e:
        logger.error("api_benchmark_analyze_failed", error=str(e))
        return WorkbenchResult(is_manufacturable=False, unit_cost=0.0, violations=[str(e)])
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
        component = load_component_from_script(
            script_path=fs_router.local_backend._resolve(request.script_path),
            session_root=fs_router.local_backend.root,
        )

        async with HEAVY_OPERATION_LOCK:
            image_path = await asyncio.to_thread(
                preview_design,
                component,
                pitch=request.pitch,
                yaw=request.yaw,
                output_dir=fs_router.local_backend.root / "renders",
            )
        events = _collect_events(fs_router)

        return PreviewDesignResponse(
            success=True,
            message="Preview generated successfully",
            image_path=str(image_path.relative_to(fs_router.local_backend.root)),
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
            component = load_component_from_script(
                script_path=fs_router.local_backend._resolve(request.script_path),
                session_root=fs_router.local_backend.root,
                script_content=request.script_content,
            )

            from worker_heavy.simulation.factory import get_simulation_builder
            builder = get_simulation_builder(fs_router.local_backend.root)

            scene_path = await asyncio.to_thread(
                builder.build_from_assembly,
                component,
                smoke_test_mode=request.smoke_test_mode,
            )

        events = _collect_events(fs_router)
        return BenchmarkToolResponse(
            success=True,
            message=f"Assets rebuilt. Scene saved to {scene_path.name}",
            artifacts={"scene_path": str(scene_path.relative_to(fs_router.local_backend.root))},
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
        component = load_component_from_script(
            script_path=fs_router.local_backend._resolve(request.script_path),
            session_root=fs_router.local_backend.root,
            script_content=request.script_content,
        )
        success = submit_for_review(component, cwd=fs_router.local_backend.root)
        events = _collect_events(fs_router)
        return BenchmarkToolResponse(
            success=success,
            message="Handover complete" if success else "Handover failed",
            events=events,
        )

    except Exception as e:
        logger.error("api_benchmark_submit_failed", error=str(e))
        return BenchmarkToolResponse(success=False, message=str(e))
