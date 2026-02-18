from pathlib import Path

import structlog

from shared.simulation.backends import PhysicsBackend
from shared.simulation.schemas import SimulatorBackendType

logger = structlog.get_logger(__name__)

# Global cache for session-isolated backends
# session_id -> PhysicsBackend
BACKEND_CACHE: dict[str, PhysicsBackend] = {}
MAX_ACTIVE_SESSIONS = 4


def get_physics_backend(
    backend_type: SimulatorBackendType,
    session_id: str | None = None,
    smoke_test_mode: bool = False,
) -> PhysicsBackend:
    """
    Returns a physics backend instance.
    If session_id is provided, it returns a cached instance for that session
    or creates a new one if not present (subject to MAX_ACTIVE_SESSIONS).
    """
    if not session_id:
        # Legacy behavior or one-off runs
        return _create_backend(backend_type, smoke_test_mode=smoke_test_mode)

    if session_id in BACKEND_CACHE:
        logger.debug("backend_cache_hit", session_id=session_id)
        backend = BACKEND_CACHE[session_id]
        if hasattr(backend, "smoke_test_mode"):
            backend.smoke_test_mode = smoke_test_mode
        return backend

    # Enforce session limit
    if len(BACKEND_CACHE) >= MAX_ACTIVE_SESSIONS:
        # Evict oldest session (simple FIFO for now)
        oldest_sid = next(iter(BACKEND_CACHE))
        logger.info("evicting_oldest_session", session_id=oldest_sid)
        backend = BACKEND_CACHE.pop(oldest_sid)
        try:
            backend.close()
        except Exception as e:
            logger.warning("backend_close_failed", session_id=oldest_sid, error=str(e))

    logger.info(
        "creating_new_session_backend", session_id=session_id, backend_type=backend_type
    )
    backend = _create_backend(backend_type, smoke_test_mode=smoke_test_mode)
    BACKEND_CACHE[session_id] = backend
    return backend


def _create_backend(
    backend_type: SimulatorBackendType, smoke_test_mode: bool = False
) -> PhysicsBackend:
    if backend_type == SimulatorBackendType.MUJOCO:
        from worker.simulation.mujoco_backend import MuJoCoBackend

        return MuJoCoBackend()
    if backend_type == SimulatorBackendType.GENESIS:
        from worker.simulation.genesis_backend import GenesisBackend

        backend = GenesisBackend()
        backend.smoke_test_mode = smoke_test_mode
        # Trigger re-init if needed with correct mode
        if hasattr(backend, "_ensure_initialized"):
            backend._ensure_initialized()
        return backend
    raise ValueError(f"Unknown backend type: {backend_type}")


def get_simulation_builder(
    output_dir: Path,
    backend_type: SimulatorBackendType = SimulatorBackendType.GENESIS,
    use_vhacd: bool = False,
):
    from worker.simulation.builder import (
        GenesisSimulationBuilder,
        MuJoCoSimulationBuilder,
    )

    if backend_type == SimulatorBackendType.MUJOCO:
        return MuJoCoSimulationBuilder(output_dir, use_vhacd)
    if backend_type == SimulatorBackendType.GENESIS:
        return GenesisSimulationBuilder(output_dir, use_vhacd)
    raise ValueError(f"Unknown backend type: {backend_type}")


def close_session_backend(session_id: str):
    """Explicitly close and remove a backend from cache."""
    if session_id in BACKEND_CACHE:
        backend = BACKEND_CACHE.pop(session_id)
        backend.close()
        logger.info("session_backend_closed", session_id=session_id)
