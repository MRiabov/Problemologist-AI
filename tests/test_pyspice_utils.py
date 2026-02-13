from PySpice.Spice.Netlist import Circuit
from PySpice.Unit import *
from shared.pyspice_utils import validate_circuit
from shared.models.schemas import PowerSupplyConfig


def test_simple_circuit():
    circuit = Circuit("Simple")
    # Use 'vsupply' prefix so validate_circuit detects it
    circuit.V("supply", "vcc", circuit.gnd, 24 @ u_V)
    circuit.R(1, "vcc", circuit.gnd, 12 @ u_Ohm)  # 2A

    psu = PowerSupplyConfig(voltage_dc=24.0, max_current_a=10.0)
    result = validate_circuit(circuit, psu)
    print(f"Result valid: {result.valid}")
    print(f"Total draw: {result.total_draw_a}A")
    print(f"Errors: {result.errors}")
    assert result.valid == True
    assert result.total_draw_a == 2.0


def test_short_circuit():
    circuit = Circuit("Short")
    circuit.V("supply", "vcc", circuit.gnd, 24 @ u_V)
    circuit.R(1, "vcc", circuit.gnd, 0.0001 @ u_Ohm)  # Huge current

    psu = PowerSupplyConfig(voltage_dc=24.0, max_current_a=10.0)
    result = validate_circuit(circuit, psu)
    print(f"Short circuit result valid: {result.valid}")
    print(f"Errors: {result.errors}")
    assert result.valid == False
    assert any("SHORT_CIRCUIT" in e or "OVERCURRENT_SUPPLY" in e for e in result.errors)


def test_overcurrent():
    circuit = Circuit("Overcurrent")
    circuit.V("supply", "vcc", circuit.gnd, 24 @ u_V)
    circuit.R(1, "vcc", circuit.gnd, 2 @ u_Ohm)  # 12A

    psu = PowerSupplyConfig(voltage_dc=24.0, max_current_a=5.0)  # 5A limit
    result = validate_circuit(circuit, psu)
    print(f"Overcurrent result valid: {result.valid}")
    print(f"Errors: {result.errors}")
    assert result.valid == False
    assert any("OVERCURRENT_SUPPLY" in e for e in result.errors)


if __name__ == "__main__":
    test_simple_circuit()
    test_short_circuit()
    test_overcurrent()
    print("All tests passed!")
