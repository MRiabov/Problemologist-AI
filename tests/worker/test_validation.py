import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

from build123d import Box
from worker.simulation.interface import SimulationScene
from worker.utils.validation import simulate

class TestValidation(unittest.TestCase):

    @patch("worker.utils.validation.SimulationBuilder")
    @patch("worker.utils.validation.mujoco")
    @patch("worker.utils.validation.prerender_24_views")
    def test_simulate_with_scene(self, mock_render, mock_mujoco, mock_builder_class):
        # Setup mocks
        mock_builder = MagicMock()
        mock_builder_class.return_value = mock_builder
        mock_builder.build_from_assembly.return_value = Path("dummy_scene.xml")

        mock_model = MagicMock()
        mock_mujoco.MjModel.from_xml_path.return_value = mock_model

        mock_data = MagicMock()
        mock_mujoco.MjData.return_value = mock_data

        # Simulate clean run
        mock_data.qvel = [0.0] * 10
        mock_data.qpos = [0.0] * 10

        mock_render.return_value = ["render.png"]

        # Create dummy scene
        scene = SimulationScene(assembly=Box(1,1,1))

        # Execute
        result = simulate(scene, output_dir=Path("tmp"))

        # Assert
        self.assertTrue(result.success)
        mock_builder.build_from_assembly.assert_called_once_with(scene.assembly)
        mock_mujoco.mj_step.assert_called()
        mock_render.assert_called_once()

    @patch("worker.utils.validation.SimulationBuilder")
    @patch("worker.utils.validation.mujoco")
    @patch("worker.utils.validation.prerender_24_views")
    def test_simulate_with_compound(self, mock_render, mock_mujoco, mock_builder_class):
        # Setup mocks
        mock_builder = MagicMock()
        mock_builder_class.return_value = mock_builder
        mock_builder.build_from_assembly.return_value = Path("dummy_scene.xml")

        mock_model = MagicMock()
        mock_mujoco.MjModel.from_xml_path.return_value = mock_model
        mock_data = MagicMock()
        mock_mujoco.MjData.return_value = mock_data
        mock_data.qvel = [0.0]
        mock_data.qpos = [0.0]

        mock_render.return_value = ["render.png"]

        # Create dummy compound
        compound = Box(1,1,1)

        # Execute
        result = simulate(compound, output_dir=Path("tmp"))

        # Assert
        self.assertTrue(result.success)
        mock_builder.build_from_assembly.assert_called_once_with(compound)

if __name__ == "__main__":
    unittest.main()
