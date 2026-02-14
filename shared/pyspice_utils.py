import logging
import os
from typing import Any

from PySpice.Spice.NgSpice.Shared import NgSpiceShared

# Configure PySpice to find ngspice library
if not os.environ.get("PYSPICE_LIBRARY_PATH"):
    for path in [
        "/usr/lib/x86_64-linux-gnu/libngspice.so.0",
        "/usr/lib/libngspice.so.0",
        "/usr/local/lib/libngspice.so.0",
        "/usr/lib/x86_64-linux-gnu/libngspice.so",
    ]:
        if os.path.exists(path):
            NgSpiceShared.LIBRARY_PATH = path
            break

from PySpice.Spice.Netlist import Circuit
from PySpice.Unit import *

from shared.models.schemas import CircuitValidationResult, PowerSupplyConfig

logger = logging.getLogger(__name__)


def create_circuit(name: str) -> Circuit:
    """Create a new PySpice circuit."""
    return Circuit(name)


def build_circuit_from_section(section: Any) -> Circuit:
    """
    Build a PySpice Circuit from an ElectronicsSection schema.

    Args:
        section: ElectronicsSection instance.

    Returns:
        A PySpice Circuit instance.
    """
    circuit = Circuit("Assembly Electronics")

    # 1. Add PSU
    # Assume PSU is connected between 'supply_v+' and '0'
    circuit.V(
        "supply", "supply_v+", circuit.gnd, float(section.power_supply.voltage_dc) @ u_V
    )

    # 2. Add Components
    # We need to map component_id to spice models.
    # For now, we use a simple resistor model for motors (stall resistance)
    for comp in section.components:
        if comp.type == "motor":
            # Heuristic: resistance = rated_voltage / stall_current
            v = comp.rated_voltage or section.power_supply.voltage_dc
            i = comp.stall_current_a or 2.0
            r = v / i
            # Wires will connect to component_id.a and component_id.b
            circuit.R(
                comp.component_id,
                f"{comp.component_id}_a",
                f"{comp.component_id}_b",
                r @ u_Ohm,
            )

    # 3. Add Wiring
    # Wiring connects terminals. We treat each 'net' as a PySpice node.
    # We'll use a simple Union-Find or similar to group connected terminals into nodes.

    # For now, let's assume a simpler model where wires are direct connections (zero ohm)
    # until we need wire resistance.
    # We'll use the 'terminal' name as the node name if it's 'supply_v+' or '0'.
    # Otherwise, we'll assign node names based on connections.

    # Netlist mapping: (comp_id, terminal) -> node_name
    term_to_node = {}

    def get_node(comp_id, term_name):
        if term_name in ["supply_v+", "0"]:
            return term_name if term_name != "0" else circuit.gnd
        # Standardize node names
        # Motors have 'a' and 'b' terminals in our circuit.R above
        if comp_id and term_name:
            return f"{comp_id}_{term_name}"
        return f"{comp_id}"

    # Connect components to the netlist using the wires
    for wire in section.wiring:
        node_from = get_node(wire.from_terminal.component, wire.from_terminal.terminal)
        node_to = get_node(wire.to_terminal.component, wire.to_terminal.terminal)

        # In PySpice, a wire is a 0V voltage source or a very small resistor.
        # To avoid singular matrix, we use a tiny resistor.
        if node_from != node_to:
            circuit.R(f"wire_{wire.wire_id}", node_from, node_to, 1e-6 @ u_Ohm)

    return circuit


def validate_circuit(
    circuit: Circuit, psu_config: PowerSupplyConfig | None = None
) -> CircuitValidationResult:
    """
    Run DC operating point analysis on the circuit.
    Detects short circuits, overcurrent, and floating nodes.
    """
    max_current = psu_config.max_current_a if psu_config else 10.0

    try:
        simulator = circuit.simulator()
        analysis = simulator.operating_point()

        node_voltages = {str(node): float(node) for node in analysis.nodes.values()}
        branch_currents = {
            str(branch): float(branch) for branch in analysis.branches.values()
        }

        total_draw = 0.0
        errors = []
        warnings = []

        # In PySpice, current through a voltage source is positive if it flows from + to - terminal (passive sign convention).
        # A power supply supplying power will have NEGATIVE current in PySpice's Vsource.
        # total_draw from PSU should be the absolute value of current through the supply Vsource.

        for name, current in branch_currents.items():
            current_val = float(current)
            if (
                abs(current_val) > 1000.0
            ):  # 1kA is definitely a short for these small systems
                errors.append(
                    f"FAILED_SHORT_CIRCUIT detected in branch {name}: {current_val:.2f}A"
                )

            # Assume any voltage source starting with 'supply' is our PSU
            if name.lower().startswith("vsupply"):
                total_draw += abs(current_val)

        if total_draw > max_current:
            errors.append(
                f"FAILED_OVERCURRENT_SUPPLY: Total draw {total_draw:.2f}A exceeds PSU rating {max_current:.2f}A"
            )

        return CircuitValidationResult(
            valid=len(errors) == 0,
            node_voltages=node_voltages,
            branch_currents=branch_currents,
            total_draw_a=total_draw,
            errors=errors,
            warnings=warnings,
        )
    except Exception as e:
        error_msg = str(e)
        if "Singular matrix" in error_msg or "indefinite matrix" in error_msg:
            return CircuitValidationResult(
                valid=False,
                errors=[
                    "FAILED_OPEN_CIRCUIT: Floating nodes or unconnected circuit components detected."
                ],
                total_draw_a=0.0,
            )
        return CircuitValidationResult(
            valid=False,
            errors=[f"Circuit simulation failed: {error_msg}"],
            total_draw_a=0.0,
        )


def simulate_circuit_transient(
    circuit: Circuit, duration_s: float, step_s: float
) -> Any:
    """


    Run a transient simulation.


    Useful for seeing how voltages/currents change over time (e.g. motor start).


    """

    simulator = circuit.simulator()

    return simulator.transient(step_time=step_s, end_time=duration_s)


def calculate_power_budget(circuit: Circuit, psu_config: PowerSupplyConfig) -> dict:
    """


    Calculate the power budget for the circuit compared to PSU capacity.


    """

    res = validate_circuit(circuit, psu_config)

    total_draw = res.total_draw_a

    capacity = psu_config.max_current_a

    margin = capacity - total_draw

    margin_pct = (margin / capacity * 100.0) if capacity > 0 else 0.0

    return {
        "total_draw_a": round(total_draw, 3),
        "max_capacity_a": round(capacity, 3),
        "margin_a": round(margin, 3),
        "margin_pct": round(margin_pct, 1),
        "is_safe": res.valid and total_draw <= capacity,
        "errors": res.errors,
    }
