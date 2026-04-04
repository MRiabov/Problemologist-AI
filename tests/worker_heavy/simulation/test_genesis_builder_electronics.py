import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

import numpy as np

from shared.enums import MotorControlMode, MovingPartType
from shared.models.schemas import MotorControl, MovingPart, WireTerminal
from worker_heavy.simulation.builder import AssemblyPartData, GenesisSimulationBuilder
from worker_heavy.simulation.genesis_backend import GenesisBackend


class TestGenesisBuilderElectronics(unittest.TestCase):
    def setUp(self):
        self.temp_dir = tempfile.TemporaryDirectory()
        self.output_dir = Path(self.temp_dir.name)
        self.builder = GenesisSimulationBuilder(self.output_dir)

    def tearDown(self):
        self.temp_dir.cleanup()

    def test_cables_in_scene_json(self):
        # Mock assembly
        mock_assembly = MagicMock()
        mock_assembly.children = []

        # Mock electronics with one wire
        mock_wire = MagicMock()
        mock_wire.wire_id = "wire_1"
        mock_wire.gauge_awg = 20
        mock_wire.routed_in_3d = True
        mock_wire.waypoints = [(0, 0, 0), (1, 1, 1)]
        mock_wire.from_terminal = WireTerminal(component="comp1", terminal="t1")
        mock_wire.to_terminal = WireTerminal(component="comp2", terminal="t2")

        mock_electronics = MagicMock()
        mock_electronics.wiring = [mock_wire]
        mock_electronics.components = []

        # Mock traverse to avoid complex logic
        with patch(
            "worker_heavy.simulation.builder.CommonAssemblyTraverser.traverse",
            return_value=[],
        ):
            # Also patch load_config to avoid config loading issues
            with patch("worker_heavy.workbenches.config.load_config"):
                self.builder.build_from_assembly(
                    assembly=mock_assembly, electronics=mock_electronics
                )

        scene_path = self.output_dir / "scene.json"
        self.assertTrue(scene_path.exists())

        with open(scene_path) as f:
            data = json.load(f)

        self.assertIn("cables", data, "'cables' key missing in scene.json")
        self.assertEqual(len(data["cables"]), 1)
        self.assertEqual(data["cables"][0]["wire_id"], "wire_1")
        self.assertEqual(len(data["cables"][0]["points"]), 2)

    def test_solution_motors_are_serialized_as_controlled_contracts(self):
        mock_assembly = MagicMock()
        mock_assembly.children = []

        mock_part = MagicMock()
        mock_part_data = AssemblyPartData(
            label="drive_motor",
            part=mock_part,
            pos=[0.0, 0.0, 0.0],
            euler=[0.0, 0.0, 0.0],
            is_fixed=False,
            material_id="aluminum_6061",
            cots_id="ServoMotor_DS3218",
        )

        moving_parts = [
            MovingPart(
                part_name="drive_motor",
                type=MovingPartType.MOTOR,
                dofs=["rotate_z"],
                control=MotorControl(mode=MotorControlMode.CONSTANT, speed=1.0),
            ),
            MovingPart(
                part_name="guide_rail",
                type=MovingPartType.PASSIVE,
                dofs=["slide_z"],
                control=None,
            ),
        ]

        def _fake_process_geometry(_part, mesh_path_base, **_kwargs):
            obj_path = mesh_path_base.with_suffix(".obj")
            obj_path.parent.mkdir(parents=True, exist_ok=True)
            obj_path.write_text("dummy mesh", encoding="utf-8")
            return [obj_path]

        mock_config = MagicMock(
            materials={},
            cnc=None,
            injection_molding=None,
            three_dp=None,
        )

        with (
            patch(
                "worker_heavy.simulation.builder.CommonAssemblyTraverser.traverse",
                return_value=[mock_part_data],
            ),
            patch(
                "worker_heavy.workbenches.config.load_config", return_value=mock_config
            ),
            patch.object(
                self.builder.processor,
                "process_geometry",
                side_effect=_fake_process_geometry,
            ),
        ):
            self.builder.build_from_assembly(
                assembly=mock_assembly, moving_parts=moving_parts
            )

        scene_path = self.output_dir / "scene.json"
        with scene_path.open() as f:
            data = json.load(f)

        self.assertIn("motors", data)
        self.assertEqual(len(data["motors"]), 1)

        motor = data["motors"][0]
        self.assertEqual(motor["part_name"], "drive_motor")
        self.assertEqual(motor["cots_id"], "ServoMotor_DS3218")
        self.assertEqual(motor["actuator_type"], "velocity")
        self.assertEqual(motor["control"]["mode"], "CONSTANT")
        self.assertEqual(motor["control"]["speed"], 1.0)
        self.assertEqual(motor["force_range"], [-1.96, 1.96])

    def test_applied_control_persistence(self):
        backend = GenesisBackend()
        # Mock entity
        mock_entity = MagicMock()
        mock_entity.get_dofs_force.return_value = np.array([0.0], dtype=np.float32)
        mock_entity.get_dofs_velocity.return_value = np.array([0.0], dtype=np.float32)
        backend.entities = {"motor1": mock_entity}
        backend.entity_configs = {"motor1": {"fixed": False}}
        backend.motors = [
            {
                "part_name": "motor1",
                "control": {"mode": "CONSTANT", "speed": 0.5},
                "cots_id": "ServoMotor_DS3218",
                "joint": "motor1_joint",
            }
        ]
        backend._materialize_solution_motor_contracts()

        # Apply control
        backend.apply_control({"motor1": 0.5})

        # Check state
        state = backend.get_actuator_state("motor1")

        self.assertEqual(state.ctrl, 0.5, f"Expected ctrl 0.5, got {state.ctrl}")
        self.assertEqual(state.forcerange, (-1.96, 1.96))

    def test_unresolved_solution_motor_contract_fails_closed(self):
        backend = GenesisBackend()
        mock_entity = MagicMock()
        mock_entity.get_dofs_force.return_value = np.array([0.0], dtype=np.float32)
        mock_entity.get_dofs_velocity.return_value = np.array([0.0], dtype=np.float32)
        backend.entities = {"motor1": mock_entity}
        backend.entity_configs = {"motor1": {"fixed": False}}
        backend.motors = [
            {
                "part_name": "motor1",
                "control": {"mode": "CONSTANT", "speed": 1.0},
                "cots_id": "ServoMotor_DOES_NOT_EXIST",
                "joint": "motor1_joint",
            }
        ]

        with self.assertRaises(ValueError):
            backend._materialize_solution_motor_contracts()

    def test_tendon_support(self):
        backend = GenesisBackend()
        # Inject cables manually as if loaded from scene
        # It should be a dict mapping name to entity
        backend.cables = {"wire_1": MagicMock()}

        self.assertTrue(
            hasattr(backend, "cables"), "GenesisBackend has no 'cables' attribute"
        )

        # Check get_all_tendon_names
        names = backend.get_all_tendon_names()
        self.assertIsInstance(names, list)
        self.assertIn("wire_1", names)

        # Check get_tendon_tension
        # Mocking the tendon tension logic which uses waypoint distances
        backend.tendon_rest_lengths = {"wire_1": 1.0}
        backend.tendon_waypoints = {"wire_1": [(0, 0, 0), (1, 0, 0)]}

        # We need to mock gs if we want it to actually run distance calc,
        # or just mock get_tendon_tension if we can.
        # But get_tendon_tension is what we are testing (or at least its existence).

        # In GenesisBackend, get_tendon_tension calculates Euclidean distance between global waypoints.
        # It needs self.entities and self.tendon_waypoints

        tension = backend.get_tendon_tension("wire_1")
        self.assertEqual(tension, 0.0)

        with self.assertRaises(ValueError):
            backend.get_tendon_tension("non_existent_wire")


if __name__ == "__main__":
    unittest.main()
