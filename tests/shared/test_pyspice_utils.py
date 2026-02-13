import pytest
from PySpice.Spice.Netlist import Circuit
from PySpice.Unit import *
from shared.pyspice_utils import validate_circuit, calculate_power_budget
from shared.models.schemas import PowerSupplyConfig

def test_validate_circuit_simple():
    circuit = Circuit('Simple')
    circuit.V('supply', 'vcc', circuit.gnd, 24@u_V)
    circuit.R(1, 'vcc', circuit.gnd, 12@u_Ohm) # 2A
    
    psu = PowerSupplyConfig(voltage_dc=24.0, max_current_a=10.0)
    result = validate_circuit(circuit, psu)
    
    assert result.valid == True
    assert result.total_draw_a == 2.0
    assert len(result.errors) == 0

def test_validate_circuit_short():
    circuit = Circuit('Short')
    circuit.V('supply', 'vcc', circuit.gnd, 24@u_V)
    circuit.R(1, 'vcc', circuit.gnd, 0.0001@u_Ohm) # Huge current
    
    psu = PowerSupplyConfig(voltage_dc=24.0, max_current_a=10.0)
    result = validate_circuit(circuit, psu)
    
    assert result.valid == False
    assert any("SHORT_CIRCUIT" in e or "OVERCURRENT_SUPPLY" in e for e in result.errors)

def test_calculate_power_budget():
    circuit = Circuit('Budget')
    circuit.V('supply', 'vcc', circuit.gnd, 24@u_V)
    circuit.R(1, 'vcc', circuit.gnd, 6@u_Ohm) # 4A
    
    psu = PowerSupplyConfig(voltage_dc=24.0, max_current_a=10.0)
    budget = calculate_power_budget(circuit, psu)
    
    assert budget["total_draw_a"] == 4.0
    assert budget["max_capacity_a"] == 10.0
    assert budget["margin_a"] == 6.0
    assert budget["margin_pct"] == 60.0
    assert budget["is_safe"] == True
