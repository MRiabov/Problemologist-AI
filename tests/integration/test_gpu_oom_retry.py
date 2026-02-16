from unittest.mock import MagicMock, patch

import pytest
from build123d import Box

from worker.simulation.loop import SimulationMetrics
from worker.utils.validation import simulate


@pytest.mark.integration
def test_gpu_oom_retry_logic(tmp_path):
    """
    Verify that a GPU OOM during Genesis simulation triggers a retry with lower fidelity.
    """
    component = Box(1, 1, 1)

    with (
        patch("worker.utils.validation.get_simulation_builder") as mock_builder,
        patch("worker.utils.validation.SimulationLoop") as mock_loop_cls,
        patch("worker.utils.validation.prerender_24_views") as mock_render,
        patch("worker.utils.validation.calculate_assembly_totals") as mock_totals,
    ):
        # Setup mocks
        mock_scene_path = MagicMock()
        mock_scene_path.exists.return_value = True
        mock_builder.return_value.build_from_assembly.return_value = mock_scene_path

        mock_loop = mock_loop_cls.return_value

        # First call fails with OOM
        oom_metrics = SimulationMetrics(
            total_time=0.0,
            total_energy=0.0,
            max_velocity=0.0,
            success=False,
            fail_reason="CUDA out of memory",
        )
        # Second call succeeds
        success_metrics = SimulationMetrics(
            total_time=10.0,
            total_energy=100.0,
            max_velocity=1.0,
            success=True,
            confidence="approximate",
        )

        mock_loop.step.side_effect = [oom_metrics, success_metrics]
        mock_render.return_value = ["render.png"]
        mock_totals.return_value = (10.0, 5.0)

        # Run simulate
        result = simulate(component, output_dir=tmp_path)

        assert result.success
        assert result.confidence == "approximate"
        assert mock_loop.step.call_count == 2
        assert mock_loop.smoke_test_mode is True
