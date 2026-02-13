import pytest
from shared.models.schemas import (
    PowerSupplyConfig,
    WiringConstraint,
    ElectronicsRequirements,
    WireTerminal,
    WireConfig,
    ElectronicComponent,
    ElectronicsSection,
    ObjectivesYaml,
    PreliminaryCostEstimation
)

def test_power_supply_config():
    config = PowerSupplyConfig(
        voltage_dc=24.0,
        max_current_a=10.0,
        location=[0, -30, 0]
    )
    assert config.voltage_dc == 24.0
    assert config.max_current_a == 10.0
    assert config.location == (0.0, -30.0, 0.0)

def test_wire_config():
    wire = WireConfig(
        wire_id="w1",
        from_terminal={"component": "psu", "terminal": "+"},
        to_terminal={"component": "motor_a", "terminal": "+"},
        gauge_awg=18,
        length_mm=350.0,
        waypoints=[[0, 0, 0], [10, 10, 10]],
        routed_in_3d=True
    )
    assert wire.wire_id == "w1"
    assert wire.from_terminal.component == "psu"
    assert wire.gauge_awg == 18
    assert len(wire.waypoints) == 2
    assert wire.waypoints[0] == (0.0, 0.0, 0.0)

def test_electronics_section():
    psu = PowerSupplyConfig(voltage_dc=24.0, max_current_a=10.0)
    comp = ElectronicComponent(
        component_id="motor_a",
        type="motor",
        assembly_part_ref="motor_1",
        rated_voltage=24.0,
        stall_current_a=3.5
    )
    wire = WireConfig(
        wire_id="w1",
        from_terminal={"component": "psu", "terminal": "+"},
        to_terminal={"component": "motor_a", "terminal": "+"},
        gauge_awg=18,
        length_mm=100.0
    )
    
    section = ElectronicsSection(
        power_supply=psu,
        components=[comp],
        wiring=[wire]
    )
    
    assert section.power_supply.voltage_dc == 24.0
    assert len(section.components) == 1
    assert section.components[0].component_id == "motor_a"
    assert len(section.wiring) == 1

def test_objectives_yaml_with_electronics():
    data = {
        "objectives": {
            "goal_zone": {"min": [0,0,0], "max": [1,1,1]},
            "build_zone": {"min": [-10,-10,-10], "max": [10,10,10]}
        },
        "simulation_bounds": {"min": [-50,-50,-50], "max": [50,50,50]},
        "moved_object": {
            "label": "ball",
            "shape": "sphere",
            "start_position": [0,0,5],
            "runtime_jitter": [0,0,0]
        },
        "constraints": {"max_unit_cost": 100, "max_weight": 5},
        "electronics_requirements": {
            "power_supply_available": {
                "voltage_dc": 24.0,
                "max_current_a": 10.0
            }
        }
    }
    obj = ObjectivesYaml(**data)
    assert obj.electronics_requirements is not None
    assert obj.electronics_requirements.power_supply_available.voltage_dc == 24.0

def test_preliminary_cost_estimation_with_electronics():
    data = {
        "constraints": {
            "benchmark_max_unit_cost_usd": 100,
            "benchmark_max_weight_kg": 5,
            "planner_target_max_unit_cost_usd": 80,
            "planner_target_max_weight_kg": 4
        },
        "totals": {
            "estimated_unit_cost_usd": 50,
            "estimated_weight_g": 1000,
            "estimate_confidence": "high"
        },
        "electronics": {
            "power_supply": {"voltage_dc": 24.0, "max_current_a": 10.0},
            "components": [
                {"component_id": "psu", "type": "power_supply"},
                {"component_id": "motor_a", "type": "motor", "rated_voltage": 24.0, "stall_current_a": 2.0}
            ],
            "wiring": [
                {
                    "wire_id": "w1",
                    "from": {"component": "psu", "terminal": "+"},
                    "to": {"component": "motor_a", "terminal": "+"},
                    "gauge_awg": 18,
                    "length_mm": 100
                }
            ]
        }
    }
    est = PreliminaryCostEstimation(**data)
    assert est.electronics is not None
    assert len(est.electronics.wiring) == 1
    assert est.electronics.wiring[0].from_terminal.component == "psu"
