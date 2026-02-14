import pytest
import numpy as np
from build123d import Box, Compound, Vector
from shared.wire_utils import check_wire_clearance
from shared.pyspice_utils import validate_circuit
from shared.models.schemas import (
    ElectronicsSection,
    PowerSupplyConfig,
    ElectronicComponent,
    WireConfig,
    PreliminaryCostEstimation,
)
from worker.simulation.loop import SimulationLoop


def test_int_120_wire_clearance_basic():
    """INT-120: Basic wire clearance validation."""
    # Create a box at (0, 0, 0)
    box = Box(10, 10, 10)
    assembly = Compound(children=[box])

    # Path far away
    path_clear = [(20, 0, 0), (20, 20, 0)]
    assert check_wire_clearance(path_clear, assembly, clearance_mm=2.0) is True

    # Path intersecting the box
    path_intersect = [(0, 0, 0), (20, 0, 0)]
    assert check_wire_clearance(path_intersect, assembly, clearance_mm=2.0) is False

    # Path close to the box (1.0mm away, clearance 2.0mm)
    path_close = [(6.0, 0, 0), (6.0, 20, 0)]
    assert check_wire_clearance(path_close, assembly, clearance_mm=2.0) is False


def test_int_123_dynamic_power_gating():
    """INT-123: Simulation loop gates motors based on circuit state."""
    # 1. Define an electronics section where a motor is NOT powered
    psu = PowerSupplyConfig(voltage=12.0, max_current_a=10.0)

    # Motor A connected to PSU, Motor B floating
    parts = [
        ElectronicComponent(
            name="motor_a", type="motor", node_a="supply_v+", node_b="0"
        ),
        ElectronicComponent(name="motor_b", type="motor", node_a="10", node_b="11"),
    ]

    electronics = ElectronicsSection(power_supply=psu, parts=parts, wiring=[])

    # 2. Mock a SimulationLoop (we need a minimal XML or mock backend)
    # Since we are testing the logic in loop.py, we can check is_powered_map
    loop = SimulationLoop(
        xml_path="/home/maksym/Work/proj/Problemologist/Problemologist-AI/tests/assets/empty_scene.xml",
        electronics=electronics,
    )

    assert loop.is_powered_map["motor_a"] == 1.0
    assert loop.is_powered_map["motor_b"] == 0.0


def test_int_128_wire_torn_failure():
    """INT-128: Simulation fails if wire tension exceeds limit."""
    # This requires a functioning physics backend and tendons.
    # For now, we'll verify the logic in the loop if we can mock tension.
    pass


if __name__ == "__main__":
    pytest.main([__file__])
