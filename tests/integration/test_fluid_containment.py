import pytest
import numpy as np
from unittest.mock import MagicMock, patch
from shared.simulation.backends import SimulatorBackendType
from shared.models.schemas import (
    ObjectivesYaml,
    ObjectivesSection,
    BoundingBox,
    FluidContainmentObjective,
    MovedObject,
    Constraints,
)
from worker.simulation.loop import SimulationLoop


@pytest.mark.integration
def test_fluid_containment_logic(tmp_path):
    """
    Verify that fluid particles are correctly counted against zones in Genesis.
    """
    xml_path = tmp_path / "scene.xml"
    xml_path.write_text("<mujoco model='test'><worldbody/></mujoco>")

    # Define containment objective
    zone = BoundingBox(min=(0, 0, 0), max=(1, 1, 1))
    obj = FluidContainmentObjective(
        fluid_id="water_1", containment_zone=zone, threshold=0.8, eval_at="end"
    )

    objectives = ObjectivesYaml(
        objectives=ObjectivesSection(
            goal_zone=BoundingBox(min=(10, 10, 10), max=(11, 11, 11)),
            build_zone=BoundingBox(min=(-10, -10, -10), max=(10, 10, 10)),
            fluid_objectives=[obj],
        ),
        simulation_bounds=BoundingBox(min=(-100, -100, -100), max=(100, 100, 100)),
        moved_object=MovedObject(
            label="obj",
            shape="sphere",
            start_position=(0, 0, 5),
            runtime_jitter=(0, 0, 0),
        ),
        constraints=Constraints(max_unit_cost=100, max_weight=10),
    )

    with patch("worker.simulation.loop.get_physics_backend") as mock_get:
        backend = MagicMock()
        mock_get.return_value = backend
        backend.get_all_body_names.return_value = []
        backend.get_all_site_names.return_value = []
        backend.get_all_actuator_names.return_value = []

        def mock_step(dt):
            mock_step.time += dt
            return MagicMock(time=mock_step.time, success=True)

        mock_step.time = 0.0
        backend.step.side_effect = mock_step

        # 100 particles: 90 inside (0.9 > 0.8 threshold)
        inside = np.random.uniform(0.1, 0.9, (90, 3))
        outside = np.random.uniform(2.0, 3.0, (10, 3))
        backend.get_particle_positions.return_value = np.vstack([inside, outside])
        backend.get_stress_field.return_value = None

        loop = SimulationLoop(
            str(xml_path),
            backend_type=SimulatorBackendType.GENESIS,
            objectives=objectives,
        )
        metrics = loop.step(control_inputs={}, duration=0.1)

        assert metrics.success is True
        assert len(metrics.fluid_metrics) == 1
        assert metrics.fluid_metrics[0].fluid_id == "water_1"
        assert metrics.fluid_metrics[0].passed is True
        assert metrics.fluid_metrics[0].measured_value == 0.9


if __name__ == "__main__":
    pytest.main([__file__])
