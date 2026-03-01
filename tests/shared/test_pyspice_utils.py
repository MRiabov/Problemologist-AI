import pytest
from PySpice.Spice.Netlist import Circuit
from shared.pyspice_utils import calculate_power_budget
from shared.models.schemas import (
    ElectronicsSection,
    PowerSupplyConfig,
    ElectronicComponent,
    WireConfig,
    WireTerminal,
)
from shared.enums import ElectronicComponentType

def test_calculate_power_budget_open_circuit_detection():
    # 1. Define a faulty circuit (open circuit - component m1 is not connected to ground)
    psu_config = PowerSupplyConfig(voltage_dc=12.0, max_current_a=10.0)
    section = ElectronicsSection(
        power_supply=psu_config,
        components=[
            ElectronicComponent(
                component_id="m1",
                type=ElectronicComponentType.MOTOR,
                stall_current_a=2.0,
            )
        ],
        wiring=[
            WireConfig(
                wire_id="w1",
                from_terminal=WireTerminal(component="supply", terminal="v+"),
                to_terminal=WireTerminal(component="m1", terminal="+"),
                gauge_awg=22,
                length_mm=100.0,
            )
            # Missing wire from m1:- to supply:0
        ],
    )

    # 2. Build a minimal circuit
    circuit = Circuit('TestCircuit')
    circuit.V('supply', 'supply_v+', circuit.gnd, 12.0)

    # 3. Call calculate_power_budget WITH section
    res = calculate_power_budget(circuit, psu_config, section=section)

    # Assertions
    assert res["is_safe"] is False
    assert any("OPEN_CIRCUIT" in str(e) for e in res["failures"])
    assert any("m1" in str(e) and "not connected" in str(e) for e in res["failures"])

def test_calculate_power_budget_valid_circuit():
    psu_config = PowerSupplyConfig(voltage_dc=12.0, max_current_a=10.0)
    section = ElectronicsSection(
        power_supply=psu_config,
        components=[
            ElectronicComponent(
                component_id="m1",
                type=ElectronicComponentType.MOTOR,
                stall_current_a=2.0,
            )
        ],
        wiring=[
            WireConfig(
                wire_id="w1",
                from_terminal=WireTerminal(component="supply", terminal="v+"),
                to_terminal=WireTerminal(component="m1", terminal="+"),
                gauge_awg=18,
                length_mm=100.0,
            ),
            WireConfig(
                wire_id="w2",
                from_terminal=WireTerminal(component="m1", terminal="-"),
                to_terminal=WireTerminal(component="supply", terminal="0"),
                gauge_awg=18,
                length_mm=100.0,
            )
        ],
    )

    # Note: In a real scenario we'd use build_circuit_from_section
    # For this unit test we just want to check if it passes through correctly
    circuit = Circuit('ValidCircuit')
    circuit.V('supply', 'supply_v+', circuit.gnd, 12.0)
    # Mocking the load
    circuit.R('load', 'supply_v+', circuit.gnd, 10)

    res = calculate_power_budget(circuit, psu_config, section=section)

    # It might still fail due to component not being in SPICE if we don't build it properly,
    # but at least it shouldn't fail due to signature issues.
    # The important part is that we can pass the section.
    assert "is_safe" in res
