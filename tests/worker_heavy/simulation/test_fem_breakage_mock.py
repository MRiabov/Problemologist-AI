from unittest.mock import MagicMock, patch

import numpy as np
import pytest

from shared.enums import FailureReason as SimulationFailureMode
from shared.simulation.backends import StressField
from shared.simulation.schemas import SimulatorBackendType
from worker_heavy.simulation.loop import SimulationLoop


@pytest.mark.unit
def test_fem_breakage_detection(tmp_path):
    """
    Verify that exceeding ultimate stress triggers PART_BREAKAGE failure.
    We mock the stress field to simulate high load.
    """
    xml_path = tmp_path / "scene.xml"
    xml_content = """
<mujoco model="fem_test">
    <worldbody>
        <body name="deformable_part" pos="0 0 1">
            <geom name="part_geom" type="box" size="0.1 0.1 0.1"/>
        </body>
    </worldbody>
</mujoco>
"""
    xml_path.write_text(xml_content)

    with (
        patch("worker_heavy.simulation.loop.get_physics_backend") as mock_get_backend,
        patch("worker_heavy.simulation.loop.validate_and_price") as mock_validate,
    ):
        mock_backend = MagicMock()
        mock_get_backend.return_value = mock_backend

        # Setup mock backend behavior
        # Numeric values to avoid max(mock, float) errors
        mock_backend.step.return_value = MagicMock(
            time=0.002, success=True, energy=0.0, velocity=0.0, stress=0.0
        )
        mock_backend.get_max_stress.return_value = 0.5
        mock_backend.get_all_actuator_names.return_value = []
        mock_backend.get_all_body_names.return_value = ["deformable_part"]
        mock_backend.get_all_site_names.return_value = []

        # High stress exceeding default ultimate (310MPa)
        high_stress = 400e6
        mock_backend.get_stress_field.return_value = StressField(
            nodes=np.array([[0, 0, 0]]), stress=np.array([high_stress])
        )

        from shared.models.schemas import (
            BoundingBox,
            Constraints,
            MovedObject,
            ObjectivesSection,
            ObjectivesYaml,
            PhysicsConfig,
        )
        from shared.models.simulation import StressSummary
        from shared.workers.workbench_models import ManufacturingMethod

        physics = PhysicsConfig(fem_enabled=True, backend=SimulatorBackendType.GENESIS)
        objectives = ObjectivesYaml(
            physics=physics,
            objectives=ObjectivesSection(
                goal_zone=BoundingBox(min=(10, 10, 10), max=(11, 11, 11)),
                build_zone=BoundingBox(min=(-100, -100, -100), max=(100, 100, 100)),
            ),
            simulation_bounds=BoundingBox(
                min=(-1000, -1000, -1000), max=(1000, 1000, 1000)
            ),
            moved_object=MovedObject(
                label="deformable_part",
                shape="box",
                start_position=(0, 0, 1),
                runtime_jitter=(0, 0, 0),
            ),
            constraints=Constraints(max_unit_cost=1000, max_weight_g=1000),
        )

        mock_backend.get_stress_summaries.return_value = [
            StressSummary(
                part_label="deformable_part",
                max_von_mises_pa=400e6,  # > 310MPa (aluminum_6061 ultimate)
                mean_von_mises_pa=100e6,
                safety_factor=0.5,
                location_of_max=(0, 0, 0),
                utilization_pct=120.0,
            )
        ]

        # Provide a mock component to build material_lookup
        mock_component = MagicMock()
        mock_child = MagicMock()
        mock_child.label = "deformable_part"
        mock_child.metadata = MagicMock()
        mock_child.metadata.material_id = "aluminum_6061"
        mock_child.metadata.manufacturing_method = ManufacturingMethod.CNC
        mock_component.children = [mock_child]

        loop = SimulationLoop(
            str(xml_path),
            component=mock_component,
            backend_type=SimulatorBackendType.GENESIS,
            objectives=objectives,
        )

        # We need to ensure smoke_test_mode is True or step once
        loop.smoke_test_mode = True

        metrics = loop.step(control_inputs={}, duration=0.1)

        assert not metrics.success
        assert metrics.failure.reason == SimulationFailureMode.PART_BREAKAGE
        assert metrics.failure.detail == "deformable_part"
