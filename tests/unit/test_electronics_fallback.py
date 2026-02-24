import pytest
from shared.enums import ElectronicComponentType
from shared.models.schemas import ElectronicsSection, PowerSupplyConfig, ElectronicComponent, WireConfig, WireTerminal
from worker_heavy.simulation.electronics import ElectronicsManager

def test_electronics_fallback_switch_behavior():
    """
    Test that a circuit with a switch correctly identifies powered/unpowered states
    using the fallback connectivity logic.
    Circuit: PowerSupply -> Wire1 -> Switch -> Wire2 -> Motor -> Wire3 -> Ground
    """
    # Define components
    psu = PowerSupplyConfig(voltage_dc=12.0, max_current_a=10.0)

    comp_switch = ElectronicComponent(component_id="sw1", type=ElectronicComponentType.SWITCH)
    comp_motor = ElectronicComponent(component_id="motor1", type=ElectronicComponentType.MOTOR, rated_voltage=12.0)

    # Wires
    # 1. Supply V+ to Switch IN
    w1 = WireConfig(
        wire_id="w1",
        from_terminal=WireTerminal(component="supply", terminal="v+"),
        to_terminal=WireTerminal(component="sw1", terminal="in"),
        gauge_awg=20, length_mm=100
    )
    # 2. Switch OUT to Motor +
    w2 = WireConfig(
        wire_id="w2",
        from_terminal=WireTerminal(component="sw1", terminal="out"),
        to_terminal=WireTerminal(component="motor1", terminal="+"),
        gauge_awg=20, length_mm=100
    )
    # 3. Motor - to Supply 0 (GND)
    w3 = WireConfig(
        wire_id="w3",
        from_terminal=WireTerminal(component="motor1", terminal="-"),
        to_terminal=WireTerminal(component="supply", terminal="0"),
        gauge_awg=20, length_mm=100
    )

    section = ElectronicsSection(
        power_supply=psu,
        components=[comp_switch, comp_motor],
        wiring=[w1, w2, w3]
    )

    manager = ElectronicsManager(section)

    # Force switch states
    manager.switch_states["sw1"] = False # Open

    # Call _fallback_update directly to verify logic
    manager._fallback_update()

    assert not manager.is_powered_map.get("motor1", 0.0), "Motor should NOT be powered when switch is OPEN"

    # Scenario 2: Switch Closed
    manager.switch_states["sw1"] = True # Closed
    manager._fallback_update()

    assert manager.is_powered_map.get("motor1", 0.0) == 1.0, "Motor SHOULD be powered when switch is CLOSED"

def test_electronics_fallback_polarity_agnostic():
    """
    Test that motor is powered even if connected in reverse polarity.
    Circuit: PowerSupply -> Wire1 -> Motor(-) ... Motor(+) -> Supply 0
    """
    psu = PowerSupplyConfig(voltage_dc=12.0, max_current_a=10.0)
    # Define motor
    comp_motor = ElectronicComponent(component_id="motor1", type=ElectronicComponentType.MOTOR, rated_voltage=12.0)

    # Reverse connection: Supply V+ -> Motor(-)
    w1 = WireConfig(
        wire_id="w1",
        from_terminal=WireTerminal(component="supply", terminal="v+"),
        to_terminal=WireTerminal(component="motor1", terminal="-"),
        gauge_awg=20, length_mm=100
    )
    # Motor(+) -> Supply 0
    w2 = WireConfig(
        wire_id="w2",
        from_terminal=WireTerminal(component="motor1", terminal="+"),
        to_terminal=WireTerminal(component="supply", terminal="0"),
        gauge_awg=20, length_mm=100
    )

    section = ElectronicsSection(
        power_supply=psu,
        components=[comp_motor],
        wiring=[w1, w2]
    )

    manager = ElectronicsManager(section)
    manager._fallback_update()

    assert manager.is_powered_map.get("motor1", 0.0) == 1.0, "Motor should be powered even with reverse polarity"
