import logging

from PySpice.Spice.Netlist import Circuit
from PySpice.Unit import *

from shared.enums import ElectronicComponentType
from shared.models.schemas import ElectronicsSection

logger = logging.getLogger(__name__)


def resolve_node_name(comp_id: str, term: str) -> str:
    """
    Resolves a component ID and terminal to a standard circuit node name.
    """
    if term == "supply_v+" or (comp_id == "supply" and term == "v+"):
        return "supply_v+"
    if term == "0" or (comp_id == "supply" and term == "0"):
        return "0"

    # Normalization for motor terminals
    if term == "a":
        term = "+"
    if term == "b":
        term = "-"

    return f"{comp_id}_{term}"


def build_circuit_from_section(
    section: ElectronicsSection,
    switch_states: dict[str, bool] | None = None,
    add_shunts: bool = False,
) -> Circuit:
    """
    Build a PySpice Circuit from an ElectronicsSection definition.

    Args:
        section: The electronics definition.
        switch_states: Optional map of switch/relay states.
        add_shunts: If True, add high-impedance paths to ground for all nodes
                    to ensure simulation stability. Default is False for validation.
    """
    circuit = Circuit("AssemblyCircuit")
    switch_states = switch_states or {}

    # 1. Setup Power Supply
    # We use a standard naming for PSU nodes: 'supply_v+' and circuit.gnd (0)
    vcc_node = "supply_v+"
    gnd_node = circuit.gnd

    circuit.V("supply", vcc_node, gnd_node, section.power_supply.voltage_dc @ u_V)

    # 2. Add Components
    for comp in section.components:
        if comp.type == ElectronicComponentType.POWER_SUPPLY:
            # Add secondary power supply (e.g. battery)
            # Use rated_voltage if available, otherwise default to main supply voltage (fallback)
            voltage = comp.rated_voltage or section.power_supply.voltage_dc
            circuit.V(
                f"{comp.component_id}",
                f"{comp.component_id}_+",
                f"{comp.component_id}_-",
                voltage @ u_V,
            )
            continue

        if comp.type == ElectronicComponentType.MOTOR:
            # Model motor as a resistive load for DC analysis
            # Use provided specs or defaults if missing
            v_rated = comp.rated_voltage or section.power_supply.voltage_dc
            i_stall = comp.stall_current_a or 2.0
            resistance = v_rated / i_stall

            # Motor terminals are usually '+' and '-' or 'a' and 'b'
            # We'll support both common patterns by mapping them to the circuit nodes
            # Note: Wiring will connect to these nodes.
            circuit.R(
                f"m_{comp.component_id}",
                f"{comp.component_id}_+",
                f"{comp.component_id}_-",
                resistance @ u_Ohm,
            )

        elif (
            comp.type == ElectronicComponentType.SWITCH
            or comp.type == ElectronicComponentType.RELAY
        ):
            # Model as a very low resistance when closed, or high when open.
            # Default to CLOSED (0.01 Ohm) if not specified in switch_states.
            is_closed = switch_states.get(comp.component_id, True)
            resistance = 0.01 if is_closed else 100 @ u_MOhm

            # Identify connected terminals from wiring
            connected_terminals = set()
            for wire in section.wiring:
                if wire.from_terminal.component == comp.component_id:
                    connected_terminals.add(wire.from_terminal.terminal)
                if wire.to_terminal.component == comp.component_id:
                    connected_terminals.add(wire.to_terminal.terminal)

            # Sort for deterministic node ordering
            terminals = sorted(list(connected_terminals))

            node1 = f"{comp.component_id}_in"
            node2 = f"{comp.component_id}_out"

            if len(terminals) == 2:
                node1 = resolve_node_name(comp.component_id, terminals[0])
                node2 = resolve_node_name(comp.component_id, terminals[1])
            elif len(terminals) > 2:
                logger.warning(
                    "switch_has_more_than_two_terminals",
                    comp_id=comp.component_id,
                    terminals=terminals,
                )
                # Fallback: use first two
                node1 = resolve_node_name(comp.component_id, terminals[0])
                node2 = resolve_node_name(comp.component_id, terminals[1])
            elif len(terminals) < 2:
                logger.warning(
                    "switch_has_fewer_than_two_terminals",
                    comp_id=comp.component_id,
                    terminals=terminals,
                )
                # Fallback to default in/out if disconnected (effectively floating)

            circuit.R(
                f"sw_{comp.component_id}",
                node1,
                node2,
                resistance @ u_Ohm,
            )

        elif comp.type == ElectronicComponentType.CONNECTOR:
            # Connectors are treated as passive junction points (nodes).
            # No explicit resistance is modeled between pins unless we knew the internal schematic.
            # Wires connect to "connector_id_pin" nodes, which implicitly joins them.
            pass

    # 3. Add Wiring
    for wire in section.wiring:
        node_from = resolve_node_name(
            wire.from_terminal.component, wire.from_terminal.terminal
        )
        node_to = resolve_node_name(
            wire.to_terminal.component, wire.to_terminal.terminal
        )

        # Model wire as a small resistor
        resistance = 0.01  # Default
        if wire.gauge_awg and wire.length_mm:
            # R_per_m approx for copper: 0.01 * (1.26 ^ (AWG - 15))
            r_per_m = 0.01 * (1.26 ** (wire.gauge_awg - 15))
            resistance = r_per_m * (wire.length_mm / 1000.0)

        circuit.R(wire.wire_id, node_from, node_to, max(resistance, 0.0001) @ u_Ohm)

    # 4. Add high-impedance shunt to ground for all nodes to prevent singular matrix
    # This ensures every node has a DC path to ground.
    if add_shunts:
        for node in circuit.nodes:
            if node != circuit.gnd:
                circuit.R(f"shunt_{node}", node, circuit.gnd, 1000 @ u_MOhm)

    return circuit
