import asyncio
import multiprocessing
import os
from concurrent.futures import ProcessPoolExecutor
from concurrent.futures.process import BrokenProcessPool
from pathlib import Path
from typing import Any

import structlog

from shared.models.simulation import SimulationResult
from worker_heavy.utils.validation import (
    simulate_subprocess,
    validate_subprocess,
)

logger = structlog.get_logger(__name__)

_SPAWN_CONTEXT = multiprocessing.get_context("spawn")


def init_genesis_worker() -> None:
    """Best-effort Genesis pre-warm inside simulation child process."""
    try:
        # Force headless defaults for child worker.
        os.environ["PYGLET_HEADLESS"] = "1"
        if "PYOPENGL_PLATFORM" not in os.environ:
            os.environ["PYOPENGL_PLATFORM"] = "egl"

        import genesis as gs
        import torch

        has_gpu = torch.cuda.is_available()
        gs.init(backend=gs.gpu if has_gpu else gs.cpu, logging_level="warning")

        # Tiny scene build to trigger first-use compilation in child.
        scene = gs.Scene(show_viewer=False)
        scene.add_entity(gs.morphs.Plane())
        scene.build()
    except Exception as exc:  # pragma: no cover - best effort prewarm
        logger.warning("genesis_child_prewarm_failed", error=str(exc))


async def run_simulation_in_isolated_process(
    *,
    script_path: str | Path,
    session_root: str | Path,
    script_content: str | None,
    output_dir: Path,
    smoke_test_mode: bool,
    backend: Any,
    session_id: str,
    particle_budget: int | None,
) -> SimulationResult:
    """
    Run one simulation task in a fresh child process.

    This provides crash containment for heavy physics calls and avoids keeping
    a long-lived in-process executor that can become permanently broken.
    """
    loop = asyncio.get_running_loop()
    try:
        with ProcessPoolExecutor(
            max_workers=1,
            max_tasks_per_child=1,
            mp_context=_SPAWN_CONTEXT,
            initializer=init_genesis_worker,
        ) as executor:
            return await loop.run_in_executor(
                executor,
                simulate_subprocess,
                script_path,
                session_root,
                script_content,
                output_dir,
                smoke_test_mode,
                backend,
                session_id,
                particle_budget,
            )
    except BrokenProcessPool as exc:
        logger.error("simulation_child_process_crashed", error=str(exc))
        raise RuntimeError("SIMULATION_CHILD_PROCESS_CRASHED") from exc


async def run_validation_in_isolated_process(
    *,
    script_path: str | Path,
    session_root: str | Path,
    script_content: str | None,
    output_dir: Path,
    smoke_test_mode: bool,
    session_id: str,
    particle_budget: int | None,
) -> tuple[bool, str | None]:
    """
    Run one validation task in a fresh child process.

    Validation prerenders native Genesis/OpenGL views, so it needs the same
    crash-containment boundary as simulation.
    """
    loop = asyncio.get_running_loop()
    try:
        with ProcessPoolExecutor(
            max_workers=1,
            max_tasks_per_child=1,
            mp_context=_SPAWN_CONTEXT,
            initializer=init_genesis_worker,
        ) as executor:
            return await loop.run_in_executor(
                executor,
                validate_subprocess,
                script_path,
                session_root,
                script_content,
                output_dir,
                smoke_test_mode,
                session_id,
                particle_budget,
            )
    except BrokenProcessPool as exc:
        logger.error("validation_child_process_crashed", error=str(exc))
        raise RuntimeError("VALIDATION_CHILD_PROCESS_CRASHED") from exc
