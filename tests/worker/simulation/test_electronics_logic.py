import pytest
import json
import numpy as np
from pathlib import Path
from build123d import Box, Compound

from worker.simulation.electronics import ElectronicsManager
from worker.simulation.builder import GenesisSimulationBuilder
from shared.models.schemas import (
    ElectronicsSection,
    ElectronicComponent,
    WireConfig,
    WireTerminal,
    PowerSupplyConfig
)
from shared.enums import ElectronicComponentType

def test_short_circuit_detection():
    # Setup a simple short circuit: battery pos -> wire -> battery neg
    electronics = ElectronicsSection(
        power_supply=PowerSupplyConfig(voltage_dc=12.0, max_current_a=10.0),
        components=[
            ElectronicComponent(component_id="bat1", type=ElectronicComponentType.POWER_SUPPLY)
        ],
        wiring=[
            WireConfig(
                wire_id="wire1",
                from_terminal=WireTerminal(component="bat1", terminal="pos"),
                to_terminal=WireTerminal(component="bat1", terminal="neg"),
                gauge_awg=18,
                length_mm=100.0
            )
        ]
    )

    manager = ElectronicsManager(electronics)
    manager.update()

    assert len(manager.errors) > 0
    assert "SHORT_CIRCUIT" in manager.errors[0]

def test_switch_logic():
    # battery -> switch -> motor
    electronics = ElectronicsSection(
        power_supply=PowerSupplyConfig(voltage_dc=12.0, max_current_a=10.0),
        components=[
            ElectronicComponent(component_id="bat1", type=ElectronicComponentType.POWER_SUPPLY),
            ElectronicComponent(component_id="sw1", type=ElectronicComponentType.SWITCH),
            ElectronicComponent(component_id="motor1", type=ElectronicComponentType.MOTOR, assembly_part_ref="actuator1")
        ],
        wiring=[
            WireConfig(
                wire_id="w1",
                from_terminal=WireTerminal(component="bat1", terminal="pos"),
                to_terminal=WireTerminal(component="sw1", terminal="in"),
                gauge_awg=18,
                length_mm=50.0
            ),
            WireConfig(
                wire_id="w2",
                from_terminal=WireTerminal(component="sw1", terminal="out"),
                to_terminal=WireTerminal(component="motor1", terminal="pos"),
                gauge_awg=18,
                length_mm=50.0
            )
        ]
    )

    manager = ElectronicsManager(electronics)

    # Switch OFF (default)
    manager.update()
    assert manager.is_powered_map.get("actuator1") is False

    # Switch ON
    manager.switch_states["sw1"] = True
    manager.update()
    assert manager.is_powered_map.get("actuator1") is True

    # Switch OFF again
    manager.switch_states["sw1"] = False
    manager.update()
    assert manager.is_powered_map.get("actuator1") is False

def test_no_short_circuit_with_motor():
    # battery -> motor -> battery (normal circuit)
    electronics = ElectronicsSection(
        power_supply=PowerSupplyConfig(voltage_dc=12.0, max_current_a=10.0),
        components=[
            ElectronicComponent(component_id="bat1", type=ElectronicComponentType.POWER_SUPPLY),
            ElectronicComponent(component_id="motor1", type=ElectronicComponentType.MOTOR)
        ],
        wiring=[
            WireConfig(
                wire_id="w1",
                from_terminal=WireTerminal(component="bat1", terminal="pos"),
                to_terminal=WireTerminal(component="motor1", terminal="t1"),
                gauge_awg=18,
                length_mm=50.0
            ),
            WireConfig(
                wire_id="w2",
                from_terminal=WireTerminal(component="motor1", terminal="t2"),
                to_terminal=WireTerminal(component="bat1", terminal="neg"),
                gauge_awg=18,
                length_mm=50.0
            )
        ]
    )

    manager = ElectronicsManager(electronics)
    manager.update()
    assert len(manager.errors) == 0

def test_genesis_builder_electronics(tmp_path):
    builder = GenesisSimulationBuilder(tmp_path)
    box = Box(1, 1, 1)

    electronics = ElectronicsSection(
        power_supply=PowerSupplyConfig(voltage_dc=12.0, max_current_a=10.0),
        components=[
            ElectronicComponent(component_id="bat1", type=ElectronicComponentType.POWER_SUPPLY)
        ],
        wiring=[
             WireConfig(
                wire_id="wire1",
                from_terminal=WireTerminal(component="bat1", terminal="pos"),
                to_terminal=WireTerminal(component="bat1", terminal="neg"),
                gauge_awg=18,
                length_mm=100.0
            )
        ]
    )

    scene_path = builder.build_from_assembly(box, electronics=electronics)

    with open(scene_path) as f:
        data = json.load(f)

    assert "electronics" in data
    assert data["electronics"] is not None
    assert data["electronics"]["power_supply"]["voltage_dc"] == 12.0
    assert len(data["electronics"]["wiring"]) == 1
    assert data["electronics"]["wiring"][0]["wire_id"] == "wire1"

def test_electronics_mapping_with_assembly_ref():
    electronics = ElectronicsSection(
        power_supply=PowerSupplyConfig(voltage_dc=12.0, max_current_a=10.0),
        components=[
            ElectronicComponent(component_id="bat1", type=ElectronicComponentType.POWER_SUPPLY),
            ElectronicComponent(component_id="m1", type=ElectronicComponentType.MOTOR, assembly_part_ref="motor_part_A")
        ],
        wiring=[
            WireConfig(
                wire_id="w1",
                from_terminal=WireTerminal(component="bat1", terminal="pos"),
                to_terminal=WireTerminal(component="m1", terminal="pos"),
                gauge_awg=18,
                length_mm=100.0
            )
        ]
    )

    manager = ElectronicsManager(electronics)
    manager.update()

    # Should be powered by component_id
    assert manager.is_powered_map.get("m1") is True
    # Should ALSO be powered by assembly_part_ref
    assert manager.is_powered_map.get("motor_part_A") is True
