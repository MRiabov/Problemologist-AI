import pytest
from unittest.mock import MagicMock, patch
from shared.simulation.backends import SimulatorBackendType
from worker.utils.validation import simulate
from build123d import Box


@pytest.mark.integration
def test_gpu_oom_retry_logic(tmp_path, monkeypatch):
    """
    Verify that a GPU OOM error triggers a retry with a lower particle budget.
    """
    monkeypatch.setenv("RENDERS_DIR", str(tmp_path / "renders"))
    (tmp_path / "renders").mkdir()

    # Mock the SimulationLoop to raise OOM on first call, then succeed
    with patch("worker.utils.validation.SimulationLoop") as mock_loop_cls:
        # First call raises OOM
        # Second call returns a mock loop that succeeds
        mock_loop_success = MagicMock()

        mock_loop_cls.side_effect = [
            RuntimeError("CUDA_ERROR_OUT_OF_MEMORY: out of memory"),
            mock_loop_success,
        ]

        # Setup mock metrics for success
        mock_metrics = MagicMock()
        mock_metrics.success = True
        mock_metrics.fail_reason = None
        mock_metrics.confidence = "approximate"
        mock_metrics.stress_summaries = []
        mock_metrics.stress_fields = {}
        mock_metrics.fluid_metrics = []

        mock_loop_success.step.return_value = mock_metrics
        mock_loop_success.particle_budget = 100000

        # We also need to mock get_simulation_builder and prerender
        with (
            patch("worker.utils.validation.get_simulation_builder"),
            patch("worker.utils.validation.prerender_24_views"),
            patch("worker.utils.validation.calculate_assembly_totals") as mock_totals,
        ):
            mock_totals.return_value = (10.0, 100.0)

            component = Box(10, 10, 10)
            res = simulate(component, output_dir=tmp_path, particle_budget=100000)

            assert res.success is True
            assert res.confidence == "approximate"
            # Verify that the second loop's particle budget was reduced (0.75 * 100000)
            assert mock_loop_success.particle_budget == 75000
            assert mock_loop_cls.call_count == 2


if __name__ == "__main__":
    pytest.main([__file__])
