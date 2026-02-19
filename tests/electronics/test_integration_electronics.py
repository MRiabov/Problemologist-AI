import pytest

pytestmark = [pytest.mark.integration, pytest.mark.xdist_group(name="physics_sims")]
from unittest.mock import MagicMock

from build123d import Box, Compound

from shared.models.schemas import (
    ElectronicComponent,
    ElectronicsSection,
    PowerSupplyConfig,
    WireConfig,
    WireTerminal,
)
from shared.pyspice_utils import CircuitValidationResult
from shared.wire_utils import check_wire_clearance
from worker_heavy.simulation.loop import SimulationLoop


def test_int_135_wire_clearance_basic():
    """INT-135: Basic wire clearance validation."""
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


def test_int_125_motor_power_gating():
    """INT-125: Simulation loop gates motors based on circuit state/switch toggling."""
    # 1. Define an electronics section where motor_a is powered, motor_b is NOT
    psu_config = PowerSupplyConfig(voltage_dc=12.0, max_current_a=10.0)

    components = [
        ElectronicComponent(component_id="sw1", type="switch"),
        ElectronicComponent(component_id="m1", type="motor"),
    ]

    # Simple circuit: PSU+ -> sw1 -> m1 -> PSU-
    wiring = [
        WireConfig(
            wire_id="w1",
            from_terminal=WireTerminal(component="supply", terminal="v+"),
            to_terminal=WireTerminal(component="sw1", terminal="in"),
            gauge_awg=22,
            length_mm=50.0,
        ),
        WireConfig(
            wire_id="w2",
            from_terminal=WireTerminal(component="sw1", terminal="out"),
            to_terminal=WireTerminal(component="m1", terminal="+"),
            gauge_awg=22,
            length_mm=50.0,
        ),
        WireConfig(
            wire_id="w3",
            from_terminal=WireTerminal(component="m1", terminal="-"),
            to_terminal=WireTerminal(component="supply", terminal="0"),
            gauge_awg=22,
            length_mm=50.0,
        ),
    ]

    electronics = ElectronicsSection(
        power_supply=psu_config, components=components, wiring=wiring
    )

    # Mock backend to avoid real physics overhead
    loop = SimulationLoop(
        xml_path="tests/assets/empty_scene.xml", electronics=electronics
    )

    # 1. Initial state: Switch closed by default -> Powered
    # _update_electronics is called in __init__
    assert loop.is_powered_map.get("m1") == pytest.approx(1.0, rel=0.01)

    # 2. Toggle switch OFF
    loop.switch_states["sw1"] = False
    loop._electronics_dirty = True
    loop.step(control_inputs={}, duration=0.01)

    assert loop.is_powered_map.get("m1") == pytest.approx(0.0, abs=1e-6)

    # 3. Toggle switch ON
    loop.switch_states["sw1"] = True
    loop._electronics_dirty = True
    loop.step(control_inputs={}, duration=0.01)
    assert loop.is_powered_map.get("m1") == pytest.approx(1.0, rel=0.01)


def test_int_126_wire_tear_failure(monkeypatch):
    """INT-126: Simulation fails if wire tension exceeds limit."""
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
                routed_in_3d=True,
            ),
            WireConfig(
                wire_id="w2",
                from_terminal=WireTerminal(component="m1", terminal="-"),
                to_terminal=WireTerminal(component="supply", terminal="0"),
                gauge_awg=24,
                length_mm=100.0,
                routed_in_3d=True,
            ),
        ],
    )

    loop = SimulationLoop(
        xml_path="tests/assets/empty_scene.xml", electronics=electronics
    )

    # Mock backend to return high tension
    def mock_get_tendon_tension(wire_id):
        if wire_id == "wire_torn_test":
            return 1000.0  # Way over limit for AWG24
        return 0.0

    monkeypatch.setattr(loop.backend, "get_tendon_tension", mock_get_tendon_tension)

    metrics = loop.step(control_inputs={}, duration=0.01)

    assert metrics.success is False
    assert "wire_torn:wire_torn_test" in metrics.fail_reason


def test_int_120_circuit_validation_pre_gate(monkeypatch):
    """INT-120: Basic circuit validation failure rejects simulation."""
    # Define an electronics section (content doesn't matter as we mock validation)
    electronics = ElectronicsSection(
        power_supply=PowerSupplyConfig(voltage_dc=12, max_current_a=1),
        wiring=[],
        components=[],
    )

    # Mock validation to fail generically
    def mock_validate(*args):
        return CircuitValidationResult(
            valid=False,
            errors=["electronics_validation_failed: random_error"],
            node_voltages={},
        )

    monkeypatch.setattr("shared.pyspice_utils.validate_circuit", mock_validate)

    loop = SimulationLoop(
        xml_path="tests/assets/empty_scene.xml", electronics=electronics
    )

    # Should fail on first step due to initial validation failure check
    metrics = loop.step(control_inputs={}, duration=0.01)

    assert metrics.success is False
    assert "validation_failed" in str(
        metrics.fail_reason
    ) or "electronics_validation_failed" in str(metrics.fail_reason)


def test_int_121_short_circuit_detection(monkeypatch):
    """INT-121: Short circuit triggers failure."""
    electronics = ElectronicsSection(
        power_supply=PowerSupplyConfig(voltage_dc=12, max_current_a=1),
        wiring=[],
        components=[],
    )

    # Mock validation failure
    def mock_validate(*args):
        return CircuitValidationResult(
            valid=False, errors=["FAILED_SHORT_CIRCUIT"], node_voltages={}
        )

    monkeypatch.setattr("shared.pyspice_utils.validate_circuit", mock_validate)

    loop = SimulationLoop(
        xml_path="tests/assets/empty_scene.xml", electronics=electronics
    )

    metrics = loop.step(control_inputs={}, duration=0.01)
    assert metrics.success is False
    # Expect failure reason to propagate
    assert "FAILED_SHORT_CIRCUIT" in str(metrics.fail_reason)


def test_int_122_overcurrent_supply_detection(monkeypatch):
    """INT-122: Overcurrent supply triggers failure."""
    electronics = ElectronicsSection(
        power_supply=PowerSupplyConfig(voltage_dc=12, max_current_a=1),
        wiring=[],
        components=[],
    )

    def mock_validate(*args):
        return CircuitValidationResult(
            valid=False,
            errors=["FAILED_OVERCURRENT_SUPPLY"],
            node_voltages={},
            total_draw_a=100.0,
        )

    monkeypatch.setattr("shared.pyspice_utils.validate_circuit", mock_validate)

    loop = SimulationLoop(
        xml_path="tests/assets/empty_scene.xml", electronics=electronics
    )

    metrics = loop.step(control_inputs={}, duration=0.01)
    assert metrics.success is False
    assert "FAILED_OVERCURRENT_SUPPLY" in str(metrics.fail_reason)


def test_int_124_open_circuit_detection(monkeypatch):
    """INT-124: Open circuit/floating node triggers failure."""
    electronics = ElectronicsSection(
        power_supply=PowerSupplyConfig(voltage_dc=12, max_current_a=1),
        wiring=[],
        components=[],
    )

    def mock_validate(*args):
        return CircuitValidationResult(
            valid=False, errors=["FAILED_OPEN_CIRCUIT:node_x"], node_voltages={}
        )

    monkeypatch.setattr("shared.pyspice_utils.validate_circuit", mock_validate)

    loop = SimulationLoop(
        xml_path="tests/assets/empty_scene.xml", electronics=electronics
    )

    metrics = loop.step(control_inputs={}, duration=0.01)
    assert metrics.success is False
    assert "FAILED_OPEN_CIRCUIT" in str(metrics.fail_reason)


def test_int_127_backward_compat():
    """INT-127: Implicit power=1.0 when no electronics section is present."""
    # Initialize loop WITHOUT electronics
    loop = SimulationLoop(xml_path="tests/assets/empty_scene.xml", electronics=None)

    # We check if a theoretical motor 'm1' would be treated as powered
    # The SimulationLoop applies controls in step().

    # We can inspect logic via a mock dynamic controller or check internal state
    # But wait, is_powered_map is empty if electronics is None.
    # Logic in loop.py:
    #   if self.electronics:
    #       power_scale = self.is_powered_map.get(name, 0.0) -> This logic MUST BE CHANGED for backward compat
    # Update logic should be: if electronics is None, power_scale = 1.0

    # To test this, we can try to "step" with a dynamic controller and see if it gets scaled to 0 or 1

    val_received = {}

    def verify_ctrl(name, val):
        val_received[name] = val

    # We need to monkeypatch backend.apply_control or observe it
    # We can pass a dynamic controller that we know returns 1.0

    mock_ctrl = MagicMock(return_value=1.0)

    # Mock apply_control to capture what was passed
    loop.backend.apply_control = MagicMock()

    loop.step(control_inputs={}, duration=0.01, dynamic_controllers={"m1": mock_ctrl})

    # Capture the args passed to apply_control
    # apply_control(ctrls) -> ctrls is dict
    call_args = loop.backend.apply_control.call_args
    assert call_args is not None
    ctrls = call_args[0][0]  # first arg

    # If backward compat works, m1 should be 1.0
    # If broken, it might be 0.0 or missing
    # But wait, 'm1' isn't in electronics (None), checking fail-safe
    assert ctrls.get("m1") == 1.0


if __name__ == "__main__":
    pytest.main([__file__])
