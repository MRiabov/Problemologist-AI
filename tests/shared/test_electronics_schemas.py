from shared.enums import ElectronicComponentType
from shared.models.schemas import (
    AssemblyDefinition,
    ElectronicsSection,
)


def test_electronics_section_validation():
    data = {
        "power_supply": {
            "voltage_dc": 24.0,
            "max_current_a": 10.0,
            "type": "mains_ac_rectified",
        },
        "components": [
            {
                "component_id": "motor1",
                "type": "motor",
                "cots_part_id": "SG90",
                "assembly_part_ref": "bracket_a",
            }
        ],
        "wiring": [
            {
                "wire_id": "w1",
                "from": {"component": "power_supply", "terminal": "v+"},
                "to": {"component": "motor1", "terminal": "+"},
                "gauge_awg": 18,
                "length_mm": 100.0,
            }
        ],
    }
    section = ElectronicsSection(**data)
    assert section.power_supply.voltage_dc == 24.0
    assert section.components[0].type == ElectronicComponentType.MOTOR
    assert section.wiring[0].wire_id == "w1"
    assert section.wiring[0].from_terminal.component == "power_supply"


def test_assembly_definition_with_electronics():
    # Minimal assembly definition
    data = {
        "constraints": {
            "benchmark_max_unit_cost_usd": 100.0,
            "benchmark_max_weight_g": 1000.0,
            "planner_target_max_unit_cost_usd": 80.0,
            "planner_target_max_weight_g": 800.0,
        },
        "manufactured_parts": [
            {
                "part_name": "bracket_a",
                "part_id": "b_a",
                "manufacturing_method": "cnc",
                "material_id": "aluminum_6061",
                "quantity": 1,
                "part_volume_mm3": 1000.0,
                "stock_bbox_mm": {"x": 10, "y": 10, "z": 10},
                "stock_volume_mm3": 1000.0,
                "removed_volume_mm3": 0.0,
                "estimated_unit_cost_usd": 10.0,
            }
        ],
        "electronics": {
            "power_supply": {"voltage_dc": 24.0, "max_current_a": 1.0},
            "components": [
                {
                    "component_id": "m1",
                    "type": "motor",
                    "assembly_part_ref": "bracket_a",
                }
            ],
        },
        "totals": {
            "estimated_unit_cost_usd": 50.0,
            "estimated_weight_g": 500.0,
            "estimate_confidence": "high",
        },
    }
    asm = AssemblyDefinition(**data)
    assert asm.electronics.components[0].assembly_part_ref == "bracket_a"


def test_electronics_reference_validation():
    import yaml

    from worker_heavy.utils.file_validation import validate_assembly_definition_yaml

    data = {
        "constraints": {
            "benchmark_max_unit_cost_usd": 100.0,
            "benchmark_max_weight_g": 1000.0,
            "planner_target_max_unit_cost_usd": 80.0,
            "planner_target_max_weight_g": 800.0,
        },
        "manufactured_parts": [],
        "electronics": {
            "power_supply": {"voltage_dc": 24.0, "max_current_a": 1.0},
            "components": [
                {
                    "component_id": "m1",
                    "type": "motor",
                    "assembly_part_ref": "missing_part",
                }
            ],
        },
        "totals": {
            "estimated_unit_cost_usd": 10.0,
            "estimated_weight_g": 100.0,
            "estimate_confidence": "high",
        },
    }

    content = yaml.dump(data)
    valid, result = validate_assembly_definition_yaml(content)
    assert valid is False
    assert "references unknown part 'missing_part'" in result[0]
