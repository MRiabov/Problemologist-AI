
import sys
from unittest.mock import MagicMock, patch

# Mock PySpice before importing shared.circuit_builder
mock_pyspice = MagicMock()
mock_circuit = MagicMock()
mock_pyspice.Spice.Netlist.Circuit = MagicMock(return_value=mock_circuit)
sys.modules["PySpice"] = mock_pyspice
sys.modules["PySpice.Spice"] = MagicMock()
sys.modules["PySpice.Spice.Netlist"] = MagicMock()

# Mock PySpice.Unit for 'from ... import *'
mock_unit = MagicMock()
class UnitMock(MagicMock):
    def __rmatmul__(self, other):
        return other

mock_unit.u_V = UnitMock()
mock_unit.u_Ohm = UnitMock()
mock_unit.u_MOhm = UnitMock()
mock_unit.__all__ = ["u_V", "u_Ohm", "u_MOhm"]
sys.modules["PySpice.Unit"] = mock_unit

import pytest
from shared.models.schemas import (
    ElectronicsSection,
    PowerSupplyConfig,
    ElectronicComponent,
    WireConfig,
    WireTerminal
)
from shared.enums import ElectronicComponentType
from shared.circuit_builder import build_circuit_from_section
from worker_heavy.simulation.electronics import ElectronicsManager

def test_switch_terminals_dynamic_resolution():
    """
    Test that circuit builder correctly identifies switch terminals from wiring
    instead of hardcoding 'in'/'out'.
    """
    psu = PowerSupplyConfig(voltage_dc=12.0, max_current_a=10.0)

    comp_switch = ElectronicComponent(
        component_id="sw1",
        type=ElectronicComponentType.SWITCH
    )

    # Wire 1: Supply(+) -> Switch(A)
    w1 = WireConfig(
        wire_id="w1",
        from_terminal=WireTerminal(component="supply", terminal="v+"),
        to_terminal=WireTerminal(component="sw1", terminal="A"),
        gauge_awg=20,
        length_mm=100
    )

    # Wire 2: Switch(B) -> Motor(+)
    w2 = WireConfig(
        wire_id="w2",
        from_terminal=WireTerminal(component="sw1", terminal="B"),
        to_terminal=WireTerminal(component="m1", terminal="+"),
        gauge_awg=20,
        length_mm=100
    )

    section = ElectronicsSection(
        power_supply=psu,
        components=[comp_switch],
        wiring=[w1, w2]
    )

    # Mock Circuit object returned by PySpice
    mock_circuit_instance = MagicMock()
    with patch("shared.circuit_builder.Circuit", return_value=mock_circuit_instance):
        build_circuit_from_section(section, switch_states={"sw1": True})

        # Verify that circuit.R was called with correct nodes for the switch
        # Expected: R("sw_sw1", "sw1_A", "sw1_B", ...)
        # Note: terminals might be sorted, so order is deterministic

        # Find calls to R (Resistor)
        calls = mock_circuit_instance.R.call_args_list

        found_switch_resistor = False
        for call in calls:
            args = call.args
            # args[0] is name, args[1] node1, args[2] node2
            if args[0] == "sw_sw1":
                node1 = args[1]
                node2 = args[2]
                print(f"Switch nodes: {node1}, {node2}")
                assert "sw1_A" in [node1, node2]
                assert "sw1_B" in [node1, node2]
                found_switch_resistor = True

        assert found_switch_resistor, "Switch resistor not found with correct terminals"

def test_fallback_power_logic_switch_state():
    """
    Test that fallback logic respects switch state (Open vs Closed).
    """
    psu = PowerSupplyConfig(voltage_dc=12.0, max_current_a=10.0)

    comp_switch = ElectronicComponent(
        component_id="sw1",
        type=ElectronicComponentType.SWITCH
    )
    comp_motor = ElectronicComponent(
        component_id="m1",
        type=ElectronicComponentType.MOTOR
    )

    # Wire 1: Supply(+) -> Switch(A)
    w1 = WireConfig(
        wire_id="w1",
        from_terminal=WireTerminal(component="supply", terminal="v+"),
        to_terminal=WireTerminal(component="sw1", terminal="A"),
        gauge_awg=20,
        length_mm=100
    )

    # Wire 2: Switch(B) -> Motor(+)
    w2 = WireConfig(
        wire_id="w2",
        from_terminal=WireTerminal(component="sw1", terminal="B"),
        to_terminal=WireTerminal(component="m1", terminal="+"),
        gauge_awg=20,
        length_mm=100
    )

    # Wire 3: Motor(-) -> Supply(0)
    w3 = WireConfig(
        wire_id="w3",
        from_terminal=WireTerminal(component="m1", terminal="-"),
        to_terminal=WireTerminal(component="supply", terminal="0"),
        gauge_awg=20,
        length_mm=100
    )

    section = ElectronicsSection(
        power_supply=psu,
        components=[comp_switch, comp_motor],
        wiring=[w1, w2, w3]
    )

    manager = ElectronicsManager(section)

    # Case 1: Switch Closed
    manager.switch_states["sw1"] = True
    manager._fallback_update()

    print(f"Powered map (Closed): {manager.is_powered_map}")
    assert manager.is_powered_map.get("m1") == 1.0, "Motor should be powered when switch is closed"

    # Case 2: Switch Open
    manager.switch_states["sw1"] = False
    manager._fallback_update()

    print(f"Powered map (Open): {manager.is_powered_map}")
    assert manager.is_powered_map.get("m1") == 0.0, "Motor should NOT be powered when switch is open"
