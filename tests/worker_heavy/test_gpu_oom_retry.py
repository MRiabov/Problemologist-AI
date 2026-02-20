import pytest
from unittest.mock import MagicMock, patch
from pathlib import Path
import tempfile
import shutil
import os
from build123d import Box
from shared.enums import SimulationFailureMode
from shared.simulation.backends import StepResult
from worker_heavy.utils.validation import simulate

@pytest.fixture
def mock_component():
    b = Box(1, 1, 1)
    b.label = "target_box"
    from shared.models.schemas import PartMetadata
    b.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)
    return b

def test_simulate_oom_retry_during_step(mock_component):
    """
    Test that simulate() retries if the backend returns an 'out of memory' error during step().
    """
    tmpdir = tempfile.mkdtemp()
    os.makedirs(os.path.join(tmpdir, "renders"), exist_ok=True)

    # Mocking environment variables
    with patch.dict(os.environ, {"RENDERS_DIR": os.path.join(tmpdir, "renders")}):
        with patch("worker_heavy.simulation.factory.get_simulation_builder") as mock_builder_factory, \
             patch("worker_heavy.simulation.loop.get_physics_backend") as mock_backend_factory, \
             patch("worker_heavy.utils.validation.prerender_24_views") as mock_prerender, \
             patch("worker_heavy.utils.validation.calculate_assembly_totals") as mock_totals, \
             patch("worker_heavy.utils.dfm.validate_and_price") as mock_v_and_p, \
             patch("worker_heavy.workbenches.config.load_config"), \
             patch("worker_heavy.utils.validation.save_simulation_result"):

            from shared.workers.workbench_models import WorkbenchResult
            mock_v_and_p.return_value = WorkbenchResult(is_manufacturable=True, unit_cost=1.0, weight_g=1.0)
            mock_totals.return_value = (0.0, 0.0)

            # Setup mock builder
            mock_builder = MagicMock()
            scene_path = Path(tmpdir) / "scene.xml"
            scene_path.write_text("<mjcf/>")
            mock_builder.build_from_assembly.return_value = scene_path
            mock_builder_factory.return_value = mock_builder

            # Setup mock backend
            mock_backend = MagicMock()
            mock_backend.get_all_site_names.return_value = []
            mock_backend.get_all_actuator_names.return_value = []
            mock_backend.get_all_body_names.return_value = ["world", "target_box"]

            mock_body_state = MagicMock()
            mock_body_state.pos = [0, 0, 0]
            mock_body_state.vel = [0, 0, 0]
            mock_backend.get_body_state.return_value = mock_body_state

            mock_backend.get_max_stress.return_value = 0.0
            mock_backend.get_stress_summaries.return_value = []
            mock_backend.get_particle_positions.return_value = None
            mock_backend.get_stress_field.return_value = None

            # First call to step fails with OOM
            # Subsequent calls (on retry) succeed
            def side_effect_step(dt):
                if side_effect_step.fail:
                    side_effect_step.fail = False
                    return StepResult(time=0.1, success=False, failure_reason="CUDA error: out of memory")
                return StepResult(time=0.1, success=True)
            side_effect_step.fail = True
            mock_backend.step.side_effect = side_effect_step

            mock_backend_factory.return_value = mock_backend

            # Prerender mock
            mock_prerender.return_value = ["render1.png"]

            # Run simulate
            res = simulate(mock_component, session_id="test_oom_session")

            if not res.success:
                print(f"DEBUG: Simulation failed with summary: {res.summary}")

            # Verify it eventually succeeded
            assert res.success is True
            # Check that we got approximate confidence
            assert res.confidence == "approximate"

            # Verify that get_physics_backend was called with reduced particle budget (5000)
            budgets = [call.kwargs.get('particle_budget') for call in mock_backend_factory.call_args_list]
            assert 5000 in budgets

    shutil.rmtree(tmpdir)
