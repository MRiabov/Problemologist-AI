from unittest.mock import MagicMock, patch

import numpy as np
import pytest

from shared.enums import SimulationFailureMode
from shared.simulation.backends import StressField
from shared.simulation.schemas import SimulatorBackendType
from worker.simulation.loop import SimulationLoop


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

    with patch("worker.simulation.loop.get_physics_backend") as mock_get_backend:
        mock_backend = MagicMock()
        mock_get_backend.return_value = mock_backend

        # Setup mock backend behavior
        mock_backend.get_all_body_names.return_value = ["deformable_part"]
        mock_backend.step.return_value = MagicMock(time=0.002, success=True)

        # High stress exceeding default ultimate (310MPa)
        high_stress = 400e6
        mock_backend.get_stress_field.return_value = StressField(
            nodes=np.array([[0, 0, 0]]), stress=np.array([high_stress])
        )

        loop = SimulationLoop(str(xml_path), backend_type=SimulatorBackendType.GENESIS)

        # We need to ensure smoke_test_mode is True or step once
        loop.smoke_test_mode = True

        metrics = loop.step(control_inputs={}, duration=0.1)

        assert not metrics.success
        assert SimulationFailureMode.PART_BREAKAGE in metrics.fail_reason
        assert "deformable_part" in metrics.fail_reason
