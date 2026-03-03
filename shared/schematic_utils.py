"""
Utilities for generating schematics from electronic assembly definitions.
"""

from shared.models.schemas import (
    AssemblyDefinition,
    ElectronicComponentType,
    SchematicItem,
)


def get_schematic_pin_index(term: str) -> str:
    """
    Map a component terminal name to a schematic pin index.

    Standard mappings:
    - +, a, in, v+, supply_v+, 1 -> "1"
    - -, b, out, 0, gnd, 2 -> "2"
    """
    term = term.lower()

    if term in ["+", "a", "in", "v+", "supply_v+", "1"]:
        return "1"
    if term in ["-", "b", "out", "0", "gnd", "2"]:
        return "2"

    # Fallback: if it's a digit, use it directly
    if term.isdigit():
        return term

    # Default to "1" if unknown (better than failing, but risky)
    return "1"


def generate_schematic_soup(assembly: AssemblyDefinition) -> list[SchematicItem]:
    """
    Generate a tscircuit Soup JSON representation of the electronics assembly.
    """
    if not assembly.electronics:
        return []

    soup = []

    # 1. Collect required pins for each component from wiring
    comp_pins: dict[str, set[str]] = {}
    for comp in assembly.electronics.components:
        comp_pins[comp.component_id] = {"1", "2"}  # Ensure at least pins 1 and 2

    for wire in assembly.electronics.wiring:
        src_comp = wire.from_terminal.component
        if src_comp in comp_pins:
            comp_pins[src_comp].add(get_schematic_pin_index(wire.from_terminal.terminal))

        dst_comp = wire.to_terminal.component
        if dst_comp in comp_pins:
            comp_pins[dst_comp].add(get_schematic_pin_index(wire.to_terminal.terminal))

    # 2. Add components and their pins
    for i, comp in enumerate(assembly.electronics.components):
        # Determine symbol
        symbol_name = "generic_component"
        if comp.type == ElectronicComponentType.MOTOR:
            symbol_name = "resistor"  # tscircuit often uses resistor for generic load
        elif comp.type == ElectronicComponentType.SWITCH:
            symbol_name = "spst_switch"
        elif comp.type == ElectronicComponentType.RELAY:
            symbol_name = "spst_switch"  # Simplified
        elif comp.type == ElectronicComponentType.POWER_SUPPLY:
            symbol_name = "battery"
        elif comp.type == ElectronicComponentType.CONNECTOR:
            symbol_name = "header"

        comp_id = f"comp_{comp.component_id}"

        soup.append(
            SchematicItem(
                type="schematic_component",
                id=comp_id,
                name=comp.component_id,
                center={"x": 10 + i * 40, "y": 10},
                rotation=0,
                symbol_name=symbol_name,
            )
        )

        # Add collected pins
        pins = sorted(list(comp_pins.get(comp.component_id, ["1", "2"])))
        for pin_idx, pin_name in enumerate(pins):
            # Distribute pins horizontally around the component center
            offset_x = (pin_idx - (len(pins) - 1) / 2) * 10
            soup.append(
                SchematicItem(
                    type="schematic_pin",
                    id=f"{comp_id}_p{pin_name}",
                    component_id=comp_id,
                    name=pin_name,
                    center={"x": 10 + i * 40 + offset_x, "y": 10},
                )
            )

    # 3. Add traces
    for wire in assembly.electronics.wiring:
        # Resolve source pin
        src_pin_idx = get_schematic_pin_index(wire.from_terminal.terminal)
        src_pin_id = f"comp_{wire.from_terminal.component}_p{src_pin_idx}"

        # Resolve target pin
        dst_pin_idx = get_schematic_pin_index(wire.to_terminal.terminal)
        dst_pin_id = f"comp_{wire.to_terminal.component}_p{dst_pin_idx}"

        soup.append(
            SchematicItem(
                type="schematic_trace",
                id=f"trace_{wire.wire_id}",
                source=src_pin_id,
                target=dst_pin_id,
            )
        )

    return soup
