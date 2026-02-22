import pytest
from PySpice.Unit import *

from shared.circuit_builder import build_circuit_from_section
from shared.models.schemas import (
    ElectronicComponent,
    ElectronicsSection,
    PowerSupplyConfig,
    WireConfig,
    WireTerminal,
)
from shared.pyspice_utils import (
    calculate_static_power_budget,
    create_circuit,
    validate_circuit,
)


def test_validate_circuit_happy_path():
    """T009: Test a happy path circuit (1 PSU, 1 motor)."""
    psu_config = PowerSupplyConfig(voltage_dc=12.0, max_current_a=10.0)
    components = [
        ElectronicComponent(
            component_id="m1",
            type="motor",
            rated_voltage=12.0,
            stall_current_a=2.0,
        )
    ]
    wiring = [
        WireConfig(
            wire_id="w1",
            from_terminal=WireTerminal(component="supply", terminal="v+"),
            to_terminal=WireTerminal(component="m1", terminal="+"),
            gauge_awg=22,
            length_mm=100.0,
        ),
        WireConfig(
            wire_id="w2",
            from_terminal=WireTerminal(component="m1", terminal="-"),
            to_terminal=WireTerminal(component="supply", terminal="0"),
            gauge_awg=22,
            length_mm=100.0,
        ),
    ]
    section = ElectronicsSection(
        power_supply=psu_config, components=components, wiring=wiring
    )

    circuit = build_circuit_from_section(section)
    result = validate_circuit(circuit, psu_config)

    assert result.valid is True
    assert result.total_draw_a > 1.9 and result.total_draw_a < 2.1  # Approx 2A
    assert len(result.errors) == 0


def test_validate_circuit_short_circuit():
    """T009: Test a short circuit case."""
    psu_config = PowerSupplyConfig(voltage_dc=12.0, max_current_a=10.0)
    # A component with very low resistance (0.001 Ohm -> 12000A)
    components = [
        ElectronicComponent(
            component_id="shorty",
            type="motor",
            rated_voltage=12.0,
            stall_current_a=12000.0,
        )
    ]
    wiring = [
        WireConfig(
            wire_id="w1",
            from_terminal=WireTerminal(component="supply", terminal="v+"),
            to_terminal=WireTerminal(component="shorty", terminal="+"),
            gauge_awg=10,  # Thick wire
            length_mm=10.0,
        ),
        WireConfig(
            wire_id="w2",
            from_terminal=WireTerminal(component="shorty", terminal="-"),
            to_terminal=WireTerminal(component="supply", terminal="0"),
            gauge_awg=10,
            length_mm=10.0,
        ),
    ]
    section = ElectronicsSection(
        power_supply=psu_config, components=components, wiring=wiring
    )

    circuit = build_circuit_from_section(section)
    result = validate_circuit(circuit, psu_config)

    assert result.valid is False
    assert any("SHORT_CIRCUIT" in e or "OVERCURRENT" in e for e in result.errors)


def test_validate_circuit_overcurrent():
    """T009: Test an overcurrent case (too many motors)."""
    psu_config = PowerSupplyConfig(voltage_dc=12.0, max_current_a=1.0)  # Small PSU
    components = [
        ElectronicComponent(
            component_id="m1",
            type="motor",
            rated_voltage=12.0,
            stall_current_a=2.0,
        )
    ]
    wiring = [
        WireConfig(
            wire_id="w1",
            from_terminal=WireTerminal(component="supply", terminal="v+"),
            to_terminal=WireTerminal(component="m1", terminal="+"),
            gauge_awg=22,
            length_mm=100.0,
        ),
        WireConfig(
            wire_id="w2",
            from_terminal=WireTerminal(component="m1", terminal="-"),
            to_terminal=WireTerminal(component="supply", terminal="0"),
            gauge_awg=22,
            length_mm=100.0,
        ),
    ]
    section = ElectronicsSection(
        power_supply=psu_config, components=components, wiring=wiring
    )

    circuit = build_circuit_from_section(section)
    result = validate_circuit(circuit, psu_config)

    assert result.valid is False
    assert any("OVERCURRENT" in e for e in result.errors)


def test_static_power_budget():
    """T008: Test static power budget calculation."""
    psu_config = PowerSupplyConfig(voltage_dc=12.0, max_current_a=5.0)
    components = [
        ElectronicComponent(component_id="m1", type="motor", stall_current_a=2.0),
        ElectronicComponent(component_id="m2", type="motor", stall_current_a=4.0),
    ]
    section = ElectronicsSection(
        power_supply=psu_config, components=components, wiring=[]
    )

    budget = calculate_static_power_budget(section)

    assert budget["total_rated_current_a"] == 6.0
    assert budget["is_safe"] is False
    assert "Budget exceeded" in budget["warnings"]


def test_validate_circuit_disconnected_motor():
    """T007: Test if a disconnected motor is detected."""
    psu_config = PowerSupplyConfig(voltage_dc=12.0, max_current_a=10.0)
    components = [
        ElectronicComponent(component_id="m1", type="motor", stall_current_a=2.0),
    ]
    # No wiring!
    section = ElectronicsSection(
        power_supply=psu_config, components=components, wiring=[]
    )

    circuit = build_circuit_from_section(section)
    result = validate_circuit(circuit, psu_config)

    # If circuit_builder adds shunts, this might be "valid" but total_draw_a = 0.
    # We want to know if it's "open".
    # Currently validate_circuit checks for singular matrix.
    # If shunts are present, no singular matrix.

    # Let's see what happens.
    assert result.total_draw_a == 0.0


def test_floating_node_detection():
    """T007: Test detection of floating nodes (unconnected components)."""
    # Note: circuit_builder adds shunts by default, so we might need a way to disable them or
    # check if they are "active".
    # For now, let's test if we can detect an open circuit if we don't use circuit_builder's shunt.

    psu_config = PowerSupplyConfig(voltage_dc=12.0, max_current_a=10.0)
    circuit = create_circuit("FloatingTest")
    circuit.V("supply", "vcc", circuit.gnd, 12 @ u_V)
    # A second PSU that is NOT connected to ground at all
    circuit.V("floating_psu", "node1", "node2", 5 @ u_V)
    circuit.R(1, "node1", "node2", 100 @ u_Ohm)

    # Run validation. This MUST fail with "Singular matrix" or similar.
    result = validate_circuit(circuit, psu_config)
    assert result.valid is False
    print(f"DEBUG ERRORS: {result.errors}")
    assert any(
        "OPEN_CIRCUIT" in e
        or "floating" in e.lower()
        or "failed" in e.lower()
        or "singular" in e.lower()
        for e in result.errors
    )


if __name__ == "__main__":
    pytest.main([__file__])
