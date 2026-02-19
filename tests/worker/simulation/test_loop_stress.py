from unittest.mock import MagicMock, patch

from shared.simulation.schemas import SimulatorBackendType
from worker.simulation.loop import SimulationLoop


class TestSimulationLoopStress:
    @patch("worker.simulation.loop.get_physics_backend")
    def test_stress_collection(self, mock_get_backend):
        # Setup mock backend
        mock_backend = MagicMock()
        mock_get_backend.return_value = mock_backend

        mock_backend.get_all_site_names.return_value = []
        mock_backend.get_all_actuator_names.return_value = []
        mock_backend.get_all_body_names.return_value = ["target_box"]
        mock_backend.get_body_state.return_value = MagicMock(
            pos=(0, 0, 0), vel=(0, 0, 0)
        )
        mock_backend.step.return_value = MagicMock(time=0.1, success=True)

        # Define sequential stress values to return
        mock_backend.get_max_stress.side_effect = [100.0, 500.0, 300.0]

        loop = SimulationLoop(
            xml_path="dummy.xml", backend_type=SimulatorBackendType.GENESIS
        )

        # Run 3 steps
        # SimulationLoop uses dt=0.002
        # duration=0.006 should result in 3 steps
        metrics = loop.step(control_inputs={}, duration=0.006)

        # Verify get_max_stress was called
        assert mock_backend.get_max_stress.call_count >= 3

        # MetricCollector should track the maximum
        assert metrics.max_stress == 500.0

    @patch("worker.simulation.loop.get_physics_backend")
    def test_stress_objective_exceeded(self, mock_get_backend):
        from shared.enums import SimulationFailureMode
        from shared.models.schemas import (
            BoundingBox,
            Constraints,
            MaxStressObjective,
            MovedObject,
            ObjectivesSection,
            ObjectivesYaml,
        )
        from shared.models.simulation import StressSummary

        # Setup mock objectives
        objectives = ObjectivesYaml(
            objectives=ObjectivesSection(
                goal_zone=BoundingBox(min=(0, 0, 0), max=(1, 1, 1)),
                build_zone=BoundingBox(min=(-10, -10, -10), max=(10, 10, 10)),
                stress_objectives=[
                    MaxStressObjective(part_label="weak_part", max_von_mises_mpa=100.0)
                ],
            ),
            simulation_bounds=BoundingBox(min=(-100, -100, -100), max=(100, 100, 100)),
            moved_object=MovedObject(
                label="obj",
                shape="sphere",
                start_position=(0, 0, 0),
                runtime_jitter=(0, 0, 0),
            ),
            constraints=Constraints(max_unit_cost=100.0, max_weight_g=100.0),
        )

        mock_backend = MagicMock()
        mock_get_backend.return_value = mock_backend

        mock_backend.get_all_site_names.return_value = []
        mock_backend.get_all_actuator_names.return_value = []
        mock_backend.get_all_body_names.return_value = ["world"]
        mock_backend.get_particle_positions.return_value = None
        mock_backend.get_max_stress.return_value = 0.0
        mock_backend.step.return_value = MagicMock(time=1.0, success=True)

        # Mock stress summaries to VIOLATE the objective (150 MPa > 100 MPa)
        mock_backend.get_stress_summaries.return_value = [
            StressSummary(
                part_label="weak_part",
                max_von_mises_pa=150e6,
                mean_von_mises_pa=50e6,
                safety_factor=1.0,
                location_of_max=(0, 0, 0),
                utilization_pct=150.0,
            )
        ]

        loop = SimulationLoop(
            xml_path="fake.xml",
            objectives=objectives,
            backend_type=SimulatorBackendType.GENESIS,
        )

        # Run 1 step
        metrics = loop.step(control_inputs={}, duration=0.002)

        assert metrics.success is False
        assert metrics.fail_mode == SimulationFailureMode.STRESS_OBJECTIVE_EXCEEDED
