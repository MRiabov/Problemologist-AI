import pytest

from shared.simulation.schemas import (
    SimulatorBackendType,
    get_default_simulator_backend,
)


def selected_backend() -> SimulatorBackendType:
    """Return the integration run's selected simulation backend."""
    return get_default_simulator_backend()


def skip_unless_genesis(reason: str) -> None:
    """Skip the current test unless the integration run selected Genesis."""
    if selected_backend() != SimulatorBackendType.GENESIS:
        pytest.skip(reason)
