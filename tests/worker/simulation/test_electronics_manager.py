import pytest
from shared.models.schemas import ElectronicsSection, ElectronicComponent, PowerSupplyConfig, WireConfig, WireTerminal
from shared.enums import ElectronicComponentType
from worker.simulation.electronics import ElectronicsManager

def test_electronics_manager_fallback_power_supply():
    # Create electronics with a power supply
    comp = ElectronicComponent(
        component_id="psu1",
        type=ElectronicComponentType.POWER_SUPPLY
    )
    electronics = ElectronicsSection(
        power_supply=PowerSupplyConfig(voltage_dc=12.0, max_current_a=10.0),
        components=[comp],
        wiring=[]
    )

    manager = ElectronicsManager(electronics)
    # Force fallback update
    manager._fallback_update()

    # Should identify psu1 as powered
    assert manager.is_powered_map.get("psu1") is True

def test_electronics_manager_fallback_power_propagation():
    # psu1 -> motor1
    psu = ElectronicComponent(component_id="psu1", type=ElectronicComponentType.POWER_SUPPLY)
    motor = ElectronicComponent(component_id="motor1", type=ElectronicComponentType.MOTOR)

    wire = WireConfig(
        wire_id="w1",
        from_terminal=WireTerminal(component="psu1", terminal="+"),
        to_terminal=WireTerminal(component="motor1", terminal="+"),
        gauge_awg=20,
        length_mm=100
    )

    electronics = ElectronicsSection(
        power_supply=PowerSupplyConfig(voltage_dc=12.0, max_current_a=10.0),
        components=[psu, motor],
        wiring=[wire]
    )

    manager = ElectronicsManager(electronics)
    manager._fallback_update()

    # Both should be powered
    assert manager.is_powered_map.get("psu1") is True
    assert manager.is_powered_map.get("motor1") is True
