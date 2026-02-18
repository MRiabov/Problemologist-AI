import sys
from unittest.mock import MagicMock, patch
import pytest
import numpy as np

# Mock genesis before importing backend
sys.modules["genesis"] = MagicMock()

from worker.simulation.genesis_backend import GenesisBackend
from worker.simulation.loop import SimulationLoop
from shared.simulation.schemas import SimulatorBackendType
from shared.models.schemas import ElectronicsSection, WireConfig, WireTerminal

class MockGenesisBackend(GenesisBackend):
    def __init__(self):
        # Skip super init that tries to load genesis
        self.entities = {}
        self.entity_configs = {}
        self.motors = []
        self.cables = []
        self.applied_controls = {}
        self.current_time = 0.0
        self._is_built = True
        self.scene = MagicMock()
        self.scene.is_built = True

    def load_scene(self, scene):
        pass

    def get_all_actuator_names(self):
        return [m["part_name"] for m in self.motors]

    def get_all_body_names(self):
        return ["body1"]

    def get_actuator_state(self, name):
        # Real implementation logic we want to test
        ctrl = self.applied_controls.get(name, 0.0)
        return MagicMock(ctrl=ctrl, velocity=1.0, force=1.0, forcerange=(-10, 10))

    def get_tendon_tension(self, name):
        if name == "wire_1":
            return 2000.0  # High tension to trigger failure
        return 0.0

@pytest.fixture
def genesis_loop(tmp_path):
    # Create dummy scene file
    (tmp_path / "scene.json").write_text("{}")

    with patch("worker.simulation.loop.get_physics_backend") as mock_get_backend:
        backend = MockGenesisBackend()
        backend.motors = [{"part_name": "motor1"}]
        mock_get_backend.return_value = backend

        loop = SimulationLoop(
            xml_path=str(tmp_path / "scene.json"),
            backend_type=SimulatorBackendType.GENESIS
        )
        return loop

def test_genesis_control_persistence_in_loop(genesis_loop):
    """Test that control values applied in the loop are persisted and affect metrics."""
    # Step with control input
    metrics = genesis_loop.step({"motor1": 0.5}, duration=0.01)

    # Verify apply_control was called on backend
    assert genesis_loop.backend.applied_controls.get("motor1") == 0.5

    # Verify energy metric calculation used the control value
    # Energy ~= sum(abs(ctrl * velocity))
    # In MockGenesisBackend: ctrl=0.5, velocity=1.0 -> energy rate = 0.5
    # Total energy = 0.5 * steps
    assert metrics.total_energy > 0

def test_genesis_wire_failure_in_loop(genesis_loop):
    """Test that high tendon tension triggers failure in the loop."""
    # Setup electronics with a wire
    wire = WireConfig(
        wire_id="wire_1",
        from_terminal=WireTerminal(component="c1", terminal="t1"),
        to_terminal=WireTerminal(component="c2", terminal="t2"),
        gauge_awg=20, # Tensile strength ~110N
        length_mm=100.0,
        routed_in_3d=True
    )
    genesis_loop.electronics = ElectronicsSection(
        power_supply={"voltage_dc": 12.0, "max_current_a": 10.0},
        components=[],
        wiring=[wire]
    )
    genesis_loop.backend.cables = [{"name": "wire_1"}]

    # Run step
    metrics = genesis_loop.step({}, duration=0.01)

    # Should fail due to wire torn (tension 2000 > 110)
    assert not metrics.success
    assert "wire_torn" in str(metrics.fail_reason)
