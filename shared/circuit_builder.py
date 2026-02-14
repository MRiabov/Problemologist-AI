import logging

from PySpice.Spice.Netlist import Circuit
from PySpice.Unit import *

from shared.models.schemas import ElectronicsSection

logger = logging.getLogger(__name__)


def build_circuit_from_section(section: ElectronicsSection) -> Circuit:
    """
    Build a PySpice Circuit from an ElectronicsSection definition.
    """
    circuit = Circuit("AssemblyCircuit")

    # Map component terminals to circuit nodes
    # We use a naming convention: {component_id}_{terminal}

    # 1. Setup Power Supply
    # The PSU is defined in section.power_supply
    # We assume there is a component with type 'power_supply' that refers to it.
    psu_comp = next((c for c in section.components if c.type == "power_supply"), None)
    psu_id = psu_comp.component_id if psu_comp else "psu"

    # Define PSU nodes
    vcc_node = f"{psu_id}_+"
    gnd_node = circuit.gnd  # Use SPICE 0 node for PSU '-'

    circuit.V("supply", vcc_node, gnd_node, section.power_supply.voltage_dc @ u_V)

    # 2. Add Components
    for comp in section.components:
        if comp.type == "power_supply":
            continue  # Already handled

        if comp.type == "motor":
            # Model motor as a resistive load for DC analysis
            # In a more advanced model, we could add inductance.
            if comp.rated_voltage and comp.stall_current_a:
                # R = V_rated / I_stall
                resistance = comp.rated_voltage / comp.stall_current_a
                # Motor terminals are usually '+' and '-'
                circuit.R(
                    f"m_{comp.component_id}",
                    f"{comp.component_id}_+",
                    f"{comp.component_id}_-",
                    resistance @ u_Ohm,
                )
            else:
                logger.warning(
                    f"Motor {comp.component_id} missing rated_voltage or stall_current_a, skipping."
                )

        elif comp.type == "switch" or comp.type == "relay":
            # Model as a very low resistance when closed, or high when open.
            # For validation, we might want to test both states.
            # Default to CLOSED for basic validation.
            circuit.R(
                f"sw_{comp.component_id}",
                f"{comp.component_id}_in",
                f"{comp.component_id}_out",
                0.01 @ u_Ohm,
            )

        # Add other types as needed

    # 3. Add Wiring
    for wire in section.wiring:
        # Wires connect terminals (nodes)
        node_from = f"{wire.from_terminal.component}_{wire.from_terminal.terminal}"
        node_to = f"{wire.to_terminal.component}_{wire.to_terminal.terminal}"

        # In SPICE, '-' terminal of PSU is GND (0)
        if node_from == f"{psu_id}_-":
            node_from = circuit.gnd
        if node_to == f"{psu_id}_-":
            node_to = circuit.gnd

        # Model wire as a small resistor (or ideally based on gauge and length)
        # R = rho * L / A
        # For now, use 0.01 Ohm or a simple model.
        # AWG 18 is approx 0.02 Ohm per meter.
        resistance = 0.01  # Default
        if wire.gauge_awg and wire.length_mm:
            # Very rough approximation for copper
            # R_per_km for AWG 18 is ~21 Ohm.
            # AWG 18: ~0.021 Ohm/m
            # AWG 16: ~0.013 Ohm/m
            # AWG 20: ~0.033 Ohm/m
            # R = (2^((AWG-10)/3) * 0.001) Ohm/m ? No.
            # Simple lookup or formula:
            r_per_m = 0.01 * (1.26 ** (wire.gauge_awg - 15))  # Rough
            resistance = r_per_m * (wire.length_mm / 1000.0)

        circuit.R(wire.wire_id, node_from, node_to, max(resistance, 0.001) @ u_Ohm)

    return circuit
