import pytest
from shared.enums import ElectronicComponentType
from shared.cots.parts.controllers import LogicBoard
from shared.models.schemas import ElectronicsSection, ElectronicComponent, PowerSupplyConfig
from shared.circuit_builder import build_circuit_from_section, resolve_node_name

def test_logic_board_enum():
    assert ElectronicComponentType.LOGIC_BOARD == "LOGIC_BOARD"

def test_logic_board_class():
    lb = LogicBoard("Arduino-Uno-R3")
    assert lb.part_number == "Arduino-Uno-R3"
    # COTSPart does not store the full data dict, but it extracts price and weight
    assert lb.price == 23.00
    assert lb.weight_g == 25.0

    # Verify we can access the source data if needed
    assert LogicBoard.board_data["Arduino-Uno-R3"]["voltage_input_min"] == 7.0

    lb_pi = LogicBoard("Raspberry-Pi-4B")
    assert lb_pi.part_number == "Raspberry-Pi-4B"
    assert lb_pi.price == 35.00

def test_resolve_node_name_logic_board():
    assert resolve_node_name("lb1", "vin") == "lb1_+"
    assert resolve_node_name("lb1", "gnd") == "lb1_-"
    assert resolve_node_name("lb1", "+") == "lb1_+"

def test_circuit_builder_with_logic_board():
    section = ElectronicsSection(
        power_supply=PowerSupplyConfig(
            voltage_dc=9.0,
            max_current_a=2.0
        ),
        components=[
            ElectronicComponent(
                component_id="lb1",
                type=ElectronicComponentType.LOGIC_BOARD,
                cots_part_id="Arduino-Uno-R3",
                rated_voltage=9.0,
                stall_current_a=0.05 # 50mA
            )
        ],
        wiring=[]
    )

    circuit = build_circuit_from_section(section)

    # Check if resistor is added for logic board
    # PySpice prepends 'R' to resistor names
    element_name = "Rlb_lb1"

    print(f"Circuit elements: {list(circuit.element_names)}")

    # Resistance = 9V / 0.05A = 180 Ohms

    found = False
    for element in circuit.elements:
        if element.name == element_name:
            found = True
            # Check resistance
            assert float(element.resistance) == pytest.approx(180.0)
            break

    assert found, f"Could not find element {element_name} in circuit"
