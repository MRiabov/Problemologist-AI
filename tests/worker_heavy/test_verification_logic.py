from unittest.mock import MagicMock, patch
import pytest
from worker_heavy.simulation.verification import verify_with_jitter
from worker_heavy.simulation.loop import SimulationMetrics

@patch("worker_heavy.simulation.verification.SimulationLoop")
def test_verify_with_jitter_logic(mock_loop_cls):
    # Setup mock loop
    mock_loop = MagicMock()
    mock_loop_cls.return_value = mock_loop

    # Setup mock backend
    mock_backend = MagicMock()
    mock_loop.backend = mock_backend

    # Setup mock step result using real Pydantic model
    real_metrics = SimulationMetrics(
        total_time=10.0,
        total_energy=100.0,
        max_velocity=1.0,
        max_stress=100.0,
        success=True,
        fail_reason=None,
        events=[],
        stress_summaries=[],
        fluid_metrics=[]
    )
    mock_loop.step.return_value = real_metrics

    result = verify_with_jitter(
        scene_path="test.xml",
        num_simulations=5
    )

    assert result.num_simulations == 5
    assert result.success_count == 5
    assert result.success_rate == 1.0
    assert result.is_consistent is True

    # Verify loop creation and steps
    assert mock_loop_cls.call_count == 5
    assert mock_loop.step.call_count == 5

    # Verify jitter application
    assert mock_backend.apply_jitter.call_count == 5
