import unittest
from build123d import Box, Location
from worker.simulation.interface import to_mjcf, SimulationScene

class TestSimulationInterface(unittest.TestCase):
    def test_to_mjcf_generation(self):
        # Create simple objects
        wall = Box(10, 10, 1)
        agent = Box(1, 1, 1)
        agent.location = Location((0, 0, 2))

        # Test to_mjcf
        scene = to_mjcf(
            env_compound=[wall],
            agent_compound=[agent],
            env_labels=["floor"],
            agent_labels=["box_agent"]
        )

        self.assertIsInstance(scene, SimulationScene)
        self.assertIsNotNone(scene.assembly)
        # Depending on implementation, Compound children count should be 2
        self.assertEqual(len(scene.assembly.children), 2)

        # Verify labels
        labels = [child.label for child in scene.assembly.children]
        self.assertIn("floor", labels)
        self.assertIn("box_agent", labels)

if __name__ == "__main__":
    unittest.main()
