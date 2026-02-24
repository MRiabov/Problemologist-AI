import pytest
from shared.models.schemas import (
    ElectronicsSection,
    PowerSupplyConfig,
    ElectronicComponent,
    WireConfig,
    WireTerminal,
)
from shared.enums import ElectronicComponentType
from worker_heavy.simulation.electronics import ElectronicsManager

def create_wire(id, comp1, term1, comp2, term2):
    return WireConfig(
        wire_id=id,
        from_terminal=WireTerminal(component=comp1, terminal=term1),
        to_terminal=WireTerminal(component=comp2, terminal=term2),
        gauge_awg=20,
        length_mm=100
    )

@pytest.fixture
def basic_circuit():
    """
    Setup: PowerSupply -> Switch -> Motor
    Wires:
    1. PS(+) -> Switch(In)
    2. Switch(Out) -> Motor(+)
    """
    ps = PowerSupplyConfig(
        type="battery",
        voltage_dc=12.0,
        max_current_a=10.0,
        location=(0,0,0)
    )

    components = [
        ElectronicComponent(
            component_id="ps_1",
            type=ElectronicComponentType.POWER_SUPPLY
        ),
        ElectronicComponent(
            component_id="switch_1",
            type=ElectronicComponentType.SWITCH
        ),
        ElectronicComponent(
            component_id="motor_1",
            type=ElectronicComponentType.MOTOR
        )
    ]

    wiring = [
        create_wire("wire_1", "ps_1", "plus", "switch_1", "in"),
        create_wire("wire_2", "switch_1", "out", "motor_1", "plus")
    ]

    return ElectronicsSection(
        power_supply=ps,
        components=components,
        wiring=wiring
    )

def test_fallback_switch_closed(basic_circuit):
    """Test that power flows when switch is closed."""
    manager = ElectronicsManager(basic_circuit)

    # Manually close the switch
    manager.switch_states["switch_1"] = True

    # Run fallback update (force it by calling directly)
    manager._fallback_update()

    assert manager.is_powered_map.get("motor_1"), "Motor should be powered when switch is closed"

def test_fallback_switch_open(basic_circuit):
    """Test that power is blocked when switch is open."""
    manager = ElectronicsManager(basic_circuit)

    # Manually open the switch (default might be closed in init, so force open)
    manager.switch_states["switch_1"] = False

    # Run fallback update
    manager._fallback_update()

    # THIS IS EXPECTED TO FAIL BEFORE FIX
    is_powered = manager.is_powered_map.get("motor_1", False)
    assert not is_powered, "Motor should NOT be powered when switch is open"

def test_fallback_complex_path():
    """
    Test a more complex path: PS -> Switch1 -> Relay -> Motor
    Switch1 is Closed.
    Relay is Open.
    Motor should be unpowered.
    """
    ps = PowerSupplyConfig(voltage_dc=12.0, max_current_a=10.0)
    components = [
        ElectronicComponent(component_id="ps", type=ElectronicComponentType.POWER_SUPPLY),
        ElectronicComponent(component_id="sw1", type=ElectronicComponentType.SWITCH),
        ElectronicComponent(component_id="relay1", type=ElectronicComponentType.RELAY),
        ElectronicComponent(component_id="motor", type=ElectronicComponentType.MOTOR),
    ]
    wiring = [
        create_wire("w1", "ps", "+", "sw1", "in"),
        create_wire("w2", "sw1", "out", "relay1", "in"),
        create_wire("w3", "relay1", "out", "motor", "+"),
    ]
    section = ElectronicsSection(power_supply=ps, components=components, wiring=wiring)

    manager = ElectronicsManager(section)
    manager.switch_states["sw1"] = True
    manager.switch_states["relay1"] = False # Open relay

    manager._fallback_update()

    assert manager.is_powered_map.get("sw1"), "Switch1 should be powered"
    assert manager.is_powered_map.get("relay1"), "Relay1 input should be powered (so component is reachable)"
    # Note: Component reachable doesn't mean "output is powered".
    # But currently is_powered_map is per COMPONENT.
    # If relay is reachable, it is "powered" (i.e. receives power).
    # But it shouldn't propagate to motor.

    assert not manager.is_powered_map.get("motor"), "Motor should be unpowered due to open relay"
