import pytest
from shared.models.schemas import (
    AssemblyDefinition,
    AssemblyConstraints,
    CostTotals,
    ElectronicsSection,
    PowerSupplyConfig,
    ElectronicComponent,
    ElectronicComponentType,
    WireConfig,
    WireTerminal,
)
from shared.schematic_utils import generate_schematic_soup, get_schematic_pin_index


def test_get_schematic_pin_index():
    # Test standard positive terminals
    assert get_schematic_pin_index("+") == "1"
    assert get_schematic_pin_index("a") == "1"
    assert get_schematic_pin_index("in") == "1"
    assert get_schematic_pin_index("v+") == "1"
    assert get_schematic_pin_index("supply_v+") == "1"
    assert get_schematic_pin_index("1") == "1"

    # Test standard negative terminals
    assert get_schematic_pin_index("-") == "2"
    assert get_schematic_pin_index("b") == "2"
    assert get_schematic_pin_index("out") == "2"
    assert get_schematic_pin_index("0") == "2"
    assert get_schematic_pin_index("gnd") == "2"
    assert get_schematic_pin_index("2") == "2"

    # Test numerical passthrough
    assert get_schematic_pin_index("3") == "3"
    assert get_schematic_pin_index("10") == "10"

    # Test default fallback
    assert get_schematic_pin_index("unknown") == "1"


def test_generate_schematic_soup_empty():
    assembly = AssemblyDefinition(
        constraints=AssemblyConstraints(
            benchmark_max_unit_cost_usd=100,
            benchmark_max_weight_g=1000,
            planner_target_max_unit_cost_usd=100,
            planner_target_max_weight_g=1000,
        ),
        totals=CostTotals(
            estimated_unit_cost_usd=10,
            estimated_weight_g=100,
            estimate_confidence="high",
        ),
        electronics=None,
    )
    assert generate_schematic_soup(assembly) == []


def test_generate_schematic_soup_components_and_wiring():
    electronics = ElectronicsSection(
        power_supply=PowerSupplyConfig(voltage_dc=12.0, max_current_a=10.0),
        components=[
            ElectronicComponent(
                component_id="motor1", type=ElectronicComponentType.MOTOR
            ),
            ElectronicComponent(
                component_id="switch1", type=ElectronicComponentType.SWITCH
            ),
        ],
        wiring=[
            # Wire from motor1(-) to switch1(in)
            WireConfig(
                wire_id="w1",
                from_terminal=WireTerminal(component="motor1", terminal="-"),
                to_terminal=WireTerminal(component="switch1", terminal="in"),
                gauge_awg=18,
                length_mm=100.0,
            )
        ],
    )

    assembly = AssemblyDefinition(
        constraints=AssemblyConstraints(
            benchmark_max_unit_cost_usd=100,
            benchmark_max_weight_g=1000,
            planner_target_max_unit_cost_usd=100,
            planner_target_max_weight_g=1000,
        ),
        totals=CostTotals(
            estimated_unit_cost_usd=10,
            estimated_weight_g=100,
            estimate_confidence="high",
        ),
        electronics=electronics,
    )

    soup = generate_schematic_soup(assembly)

    # 1. Verify components exist
    motor = next(
        (
            c
            for c in soup
            if c["type"] == "schematic_component" and c["name"] == "motor1"
        ),
        None,
    )
    switch = next(
        (
            c
            for c in soup
            if c["type"] == "schematic_component" and c["name"] == "switch1"
        ),
        None,
    )

    assert motor is not None
    assert motor["symbol_name"] == "resistor"

    assert switch is not None
    assert switch["symbol_name"] == "spst_switch"

    # 2. Verify pins exist
    assert any(c["id"] == "comp_motor1_p1" for c in soup)
    assert any(c["id"] == "comp_motor1_p2" for c in soup)
    assert any(c["id"] == "comp_switch1_p1" for c in soup)
    assert any(c["id"] == "comp_switch1_p2" for c in soup)

    # 3. Verify trace logic (The Critical Fix)
    # motor1(-) should map to pin 2
    # switch1(in) should map to pin 1
    # So trace should go from comp_motor1_p2 to comp_switch1_p1

    trace = next(
        (c for c in soup if c["type"] == "schematic_trace" and c["id"] == "trace_w1"),
        None,
    )
    assert trace is not None

    assert trace["source"] == "comp_motor1_p2"
    assert trace["target"] == "comp_switch1_p1"
