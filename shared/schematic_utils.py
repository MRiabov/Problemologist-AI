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
    - +, a, in, v+, supply_v+, 1, coil+ -> "1"
    - -, b, out, 0, gnd, 2, coil- -> "2"
    - com -> "3"
    - no -> "4"
    - nc -> "5"
    """
    term = term.lower()

    if term in ["+", "a", "in", "v+", "supply_v+", "1", "coil+"]:
        return "1"
    if term in ["-", "b", "out", "0", "gnd", "2", "coil-"]:
        return "2"

    # Common relay/switch terminals
    if term == "com":
        return "3"
    if term == "no":
        return "4"
    if term == "nc":
        return "5"

    # Fallback: if it's a digit, use it directly
    if term.isdigit():
        return term

    # If it looks like a valid alphanumeric identifier, use it
    import re
    clean_term = re.sub(r'[^a-zA-Z0-9]', '', term)
    if clean_term:
        return clean_term

    # Default to "1" if unknown (better than failing, but risky)
    return "1"


def generate_schematic_soup(assembly: AssemblyDefinition) -> list[SchematicItem]:
    """
    Generate a tscircuit Soup JSON representation of the electronics assembly.
    """
    if not assembly.electronics:
        return []

    soup = []

    # 1. Add components
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
        center_x = 10 + i * 40

        soup.append(
            SchematicItem(
                type="schematic_component",
                id=comp_id,
                name=comp.component_id,
                center={"x": center_x, "y": 10},
                rotation=0,
                symbol_name=symbol_name,
            )
        )

        # Collect pins from wiring
        used_pins = set()
        for wire in assembly.electronics.wiring:
            if wire.from_terminal.component == comp.component_id:
                used_pins.add(get_schematic_pin_index(wire.from_terminal.terminal))
            if wire.to_terminal.component == comp.component_id:
                used_pins.add(get_schematic_pin_index(wire.to_terminal.terminal))

        # Ensure at least pins 1 and 2 exist for standard components if no wiring implies otherwise
        if not used_pins:
            used_pins.add("1")
            used_pins.add("2")

        sorted_pins = sorted(list(used_pins))

        # Distribute pins horizontally centered on the component
        # Width available roughly 20 units (-10 to +10)
        num_pins = len(sorted_pins)
        for p_idx, pin_name in enumerate(sorted_pins):
            # Calculate offset
            if num_pins > 1:
                # Spread linearly from -10 to +10
                offset = (p_idx * 20.0 / (num_pins - 1)) - 10
            else:
                offset = 0

            soup.append(
                SchematicItem(
                    type="schematic_pin",
                    id=f"{comp_id}_p{pin_name}",
                    component_id=comp_id,
                    name=pin_name,
                    center={"x": center_x + offset, "y": 10},
                )
            )

    # 2. Add traces
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
