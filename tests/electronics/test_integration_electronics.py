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
    from shared.models.schemas import WireTerminal, WireConfig

    # 1. Define an electronics section where motor_a is powered, motor_b is NOT
    psu_config = PowerSupplyConfig(voltage_dc=12.0, max_current_a=10.0)

    components = [
        ElectronicComponent(
            component_id="motor_a", 
            type="motor", 
            rated_voltage=12.0, 
            stall_current_a=2.0
        ),
        ElectronicComponent(
            component_id="motor_b", 
            type="motor", 
            rated_voltage=12.0, 
            stall_current_a=2.0
        ),
    ]

    # wiring: PSU (+) -> motor_a_+, motor_a_- -> PSU (-)
    # motor_b is left unconnected
    wiring = [
        WireConfig(
            wire_id="w1",
            from_terminal=WireTerminal(component="supply", terminal="v+"),
            to_terminal=WireTerminal(component="motor_a", terminal="+"),
            gauge_awg=22,
            length_mm=100.0,
        ),
        WireConfig(
            wire_id="w2",
            from_terminal=WireTerminal(component="motor_a", terminal="-"),
            to_terminal=WireTerminal(component="supply", terminal="0"),
            gauge_awg=22,
            length_mm=100.0,
        ),
    ]

    electronics = ElectronicsSection(
        power_supply=psu_config, components=components, wiring=wiring
    )

    # 2. Mock a SimulationLoop
    # We use a real SimulationLoop but point to a minimal XML
    loop = SimulationLoop(
        xml_path="/home/maksym/Work/proj/Problemologist/Problemologist-AI/tests/assets/empty_scene.xml",
        electronics=electronics,
    )

    # motor_a should be powered (v_diff > 0.1V)
    # motor_b should be 0.0
    assert loop.is_powered_map["motor_a"] == 1.0
    assert loop.is_powered_map.get("motor_b", 0.0) == 0.0


def test_int_128_wire_torn_failure(monkeypatch):
    """INT-128: Simulation fails if wire tension exceeds limit."""
    from shared.models.schemas import WireTerminal, WireConfig

    psu_config = PowerSupplyConfig(voltage_dc=12.0, max_current_a=10.0)
    electronics = ElectronicsSection(
        power_supply=psu_config,
        components=[ElectronicComponent(component_id="m1", type="motor")],
        wiring=[
            WireConfig(
                wire_id="wire_torn_test",
                from_terminal=WireTerminal(component="supply", terminal="v+"),
                to_terminal=WireTerminal(component="m1", terminal="+"),
                gauge_awg=24,
                length_mm=100.0,
                routed_in_3d=True
            ),
            WireConfig(
                wire_id="w2",
                from_terminal=WireTerminal(component="m1", terminal="-"),
                to_terminal=WireTerminal(component="supply", terminal="0"),
                gauge_awg=24,
                length_mm=100.0,
                routed_in_3d=True
            )
        ]
    )

    loop = SimulationLoop(
        xml_path="tests/assets/empty_scene.xml",
        electronics=electronics,
    )

    # Mock backend to return high tension for our wire
    def mock_get_tendon_tension(wire_id):
        if wire_id == "wire_torn_test":
            return 1000.0 # Way over limit
        return 0.0

    monkeypatch.setattr(loop.backend, "get_tendon_tension", mock_get_tendon_tension)
    
    # Run one step
    metrics = loop.step(control_inputs={}, duration=0.01)
    
    assert metrics.success is False
    assert "WIRE_TORN:wire_torn_test" in metrics.fail_reason
    # Motors should be unpowered after tear
    assert loop.is_powered_map["m1"] == 0.0


if __name__ == "__main__":
    pytest.main([__file__])
