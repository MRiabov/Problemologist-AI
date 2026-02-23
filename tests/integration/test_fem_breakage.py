from unittest.mock import MagicMock, patch

import numpy as np
import pytest
from build123d import Box

from shared.enums import FailureReason as SimulationFailureMode
from shared.models.schemas import (
    ObjectivesYaml,
    PartMetadata,
    PhysicsConfig,
)
from shared.simulation.backends import StressField, StressSummary
from shared.simulation.schemas import SimulatorBackendType
from worker_heavy.simulation.loop import SimulationLoop


@pytest.mark.integration
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

    # Create dummy component with metadata
    comp = Box(0.1, 0.1, 0.1)
    comp.label = "deformable_part"
    comp.metadata = PartMetadata(material_id="aluminum_6061")

    # Create objectives with FEM enabled
    objectives = ObjectivesYaml(
        objectives={"goal_zone": {"min": [0,0,0], "max": [1,1,1]}, "build_zone": {"min": [-1,-1,-1], "max": [1,1,1]}, "stress_objectives": []},
        physics=PhysicsConfig(fem_enabled=True),
        simulation_bounds={"min": [-1, -1, -1], "max": [1, 1, 1]},
        moved_object={"label": "ball", "shape": "sphere", "start_position": [0,0,0.5], "runtime_jitter": [0,0,0]},
        constraints={"max_unit_cost": 100, "max_weight_g": 100},
    )

    with patch("worker_heavy.simulation.loop.get_physics_backend") as mock_get_backend:
        mock_backend = MagicMock()
        mock_get_backend.return_value = mock_backend

        # Setup mock backend behavior
        mock_backend.get_all_body_names.return_value = ["deformable_part"]
        mock_backend.get_all_site_names.return_value = []
        mock_backend.get_all_actuator_names.return_value = []
        mock_backend.step.return_value = MagicMock(time=0.002, success=True)

        # High stress exceeding default ultimate (310MPa)
        high_stress = 400e6
        mock_backend.get_max_stress.return_value = high_stress
        mock_backend.get_stress_field.return_value = StressField(
            nodes=np.array([[0, 0, 0]]), stress=np.array([high_stress])
        )
        mock_backend.get_stress_summaries.return_value = [
            StressSummary(
                part_label="deformable_part",
                max_von_mises_pa=high_stress,
                mean_von_mises_pa=high_stress / 2,
                safety_factor=0.5,
                location_of_max=(0, 0, 0),
                utilization_pct=200.0,
            )
        ]

        # Mock validate_and_price and load_config to avoid actual DFM logic
        with (
            patch("worker_heavy.simulation.loop.validate_and_price") as mock_val,
            patch("worker_heavy.simulation.loop.load_config") as mock_load,
        ):
            mock_val.return_value = MagicMock(is_manufacturable=True)
            from shared.workers.workbench_models import (
                ManufacturingConfig,
                MaterialDefinition,
            )

            mock_cfg = ManufacturingConfig(
                materials={
                    "aluminum_6061": MaterialDefinition(
                        name="Aluminum",
                        ultimate_stress_pa=310e6,
                        density_g_cm3=2.7,
                        cost_per_kg=10.0,
                    )
                }
            )
            mock_load.return_value = mock_cfg

            loop = SimulationLoop(
                str(xml_path),
                component=comp,
                objectives=objectives,
                backend_type=SimulatorBackendType.GENESIS,
            )

            # We need to ensure smoke_test_mode is True or step once
            loop.smoke_test_mode = True

            metrics = loop.step(control_inputs={}, duration=0.1)

            assert not metrics.success
            assert metrics.failure.reason == SimulationFailureMode.PART_BREAKAGE
            assert metrics.failure.detail == "deformable_part"
