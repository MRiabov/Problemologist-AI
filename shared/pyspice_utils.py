import structlog
from typing import Any

from PySpice.Spice.Netlist import Circuit
from PySpice.Spice.NgSpice.Shared import NgSpiceShared
from PySpice.Unit import *

from shared.models.schemas import (
    CircuitValidationResult,
    ElectronicsSection,
    PowerSupplyConfig,
)

# NGSpice 42 compatibility monkeypatch
# NGSpice 42 sends "Using SPARSE 1.3" to stderr, which PySpice treats as an error.
_original_send_char = NgSpiceShared._send_char


@staticmethod
def _patched_send_char(message_c, ngspice_id, user_data):
    from PySpice.Spice.NgSpice.Shared import ffi, ffi_string_utf8

    self = ffi.from_handle(user_data)
    message = ffi_string_utf8(message_c)
    prefix, _, content = message.partition(" ")
    if prefix == "stderr" and (
        "SPARSE" in content
        or content.startswith("Note:")
        or "singular matrix" in content.lower()
    ):
        self._stdout.append(content)
        return 0
    return _original_send_char(message_c, ngspice_id, user_data)


NgSpiceShared._send_char = _patched_send_char

logger = structlog.get_logger(__name__)


def create_circuit(name: str) -> Circuit:
    """Create a new PySpice circuit."""
    return Circuit(name)


def validate_circuit(
    circuit: Circuit,
    psu_config: PowerSupplyConfig | None = None,
    section: ElectronicsSection | None = None,
) -> CircuitValidationResult:
    """
    Run DC operating point analysis on the circuit.
    Detects short circuits, overcurrent, and floating nodes.
    """
    max_current = psu_config.max_current_a if psu_config else 10.0

    try:
        simulator = circuit.simulator()

        # Clear previous output to avoid carry-over errors
        if hasattr(simulator, "_ngspice_shared") and hasattr(
            simulator._ngspice_shared, "_stdout"
        ):
            simulator._ngspice_shared._stdout = []

        analysis = simulator.operating_point()

        # Check for captured singular matrix warnings
        if hasattr(simulator, "_ngspice_shared") and hasattr(
            simulator._ngspice_shared, "_stdout"
        ):
            for line in simulator._ngspice_shared._stdout:
                if "singular matrix" in line.lower():
                    return CircuitValidationResult(
                        valid=False,
                        errors=[
                            f"FAILED_OPEN_CIRCUIT: Floating nodes detected (singular matrix): {line.strip()}"
                        ],
                        total_draw_a=0.0,
                    )

        import numpy as np

        node_voltages = {
            str(node): float(np.array(node).item()) for node in analysis.nodes.values()
        }
        branch_currents = {
            str(branch): float(np.array(branch).item())
            for branch in analysis.branches.values()
        }

        total_draw = 0.0
        errors = []
        warnings = []

        # Check for short circuit at PSU nodes
        vcc_node = "supply_v+"
        if vcc_node in node_voltages:
            v_vcc = node_voltages[vcc_node]
            if v_vcc < (psu_config.voltage_dc * 0.9 if psu_config else 0.1):
                # If PSU voltage is dragged down significantly, it might be a short
                # but ONLY if we are actually supplying power.
                pass

        # In PySpice, current through a voltage source is positive if it flows from + to - terminal (passive sign convention).
        # A power supply supplying power will have NEGATIVE current in PySpice's Vsource.
        # total_draw from PSU should be the absolute value of current through the supply Vsource.

        for name, current in branch_currents.items():
            current_val = float(current)

            # Detect extreme currents which indicate shorts
            if abs(current_val) > 1000.0:
                errors.append(
                    f"FAILED_SHORT_CIRCUIT: Extreme current in branch {name}: "
                    f"{current_val:.2f}A"
                )

            # Assume any voltage source starting with 'vsupply' is our PSU
            if name.lower().startswith("vsupply"):
                total_draw += abs(current_val)

        if total_draw > max_current:
            errors.append(
                f"FAILED_OVERCURRENT_SUPPLY: Total draw {total_draw:.2f}A "
                f"exceeds PSU rating {max_current:.2f}A"
            )

        # T007: Per-wire overcurrent check (INT-123)
        if section and section.wiring:
            from shared.wire_utils import get_awg_properties

            for wire in section.wiring:
                # Resolve nodes as in circuit_builder.py
                def resolve_node_name(comp_id, term):
                    if term == "supply_v+" or (comp_id == "supply" and term == "v+"):
                        return "supply_v+"
                    if term == "0" or (comp_id == "supply" and term == "0"):
                        return "0"
                    if term == "a":
                        term = "+"
                    if term == "b":
                        term = "-"
                    return f"{comp_id}_{term}"

                n_from = resolve_node_name(
                    wire.from_terminal.component, wire.from_terminal.terminal
                )
                n_to = resolve_node_name(
                    wire.to_terminal.component, wire.to_terminal.terminal
                )

                v1 = node_voltages.get(n_from, 0.0)
                v2 = node_voltages.get(n_to, 0.0)

                # Get resistance from circuit if possible, or recalculate
                # Actually, circuit elements are stored in circuit.elements
                # But it's easier to just recalculate for validation
                # or look it up: R_wire_id
                r_val = 0.01
                if f"R{wire.wire_id}" in circuit.element_names:
                    r_val = float(circuit[wire.wire_id].resistance)
                elif wire.gauge_awg and wire.length_mm:
                    props = get_awg_properties(wire.gauge_awg)
                    r_val = props["resistance_ohm_m"] * (wire.length_mm / 1000.0)

                current = abs(v1 - v2) / max(r_val, 0.0001)

                limit = get_awg_properties(wire.gauge_awg)["max_current_a"]
                if current > limit:
                    errors.append(
                        f"FAILED_OVERCURRENT_WIRE: Wire {wire.wire_id} (AWG {wire.gauge_awg}) "
                        f"carrying {current:.2f}A exceeds limit {limit:.2f}A"
                    )

        # Detect floating nodes: check if any node has near-zero conductance to everything else
        # PySpice usually throws an exception for this, but we can also check for extremely high voltages
        # or non-finite values if SPICE managed to return something.
        for node, voltage in node_voltages.items():
            if not np.isfinite(voltage) or abs(voltage) > 1e6:
                errors.append(
                    f"FAILED_FLOATING_NODE: Node {node} has unstable voltage: {voltage}"
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
        error_msg = str(e).strip()
        if "Singular matrix" in error_msg or "indefinite matrix" in error_msg:
            return CircuitValidationResult(
                valid=False,
                errors=[
                    "FAILED_OPEN_CIRCUIT: Floating nodes or unconnected "
                    "circuit components detected."
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


def calculate_static_power_budget(section: ElectronicsSection) -> dict:
    """
    Perform a simple static power budget calculation without running SPICE.
    Sums up rated currents and compares to PSU capacity.
    """
    total_rated_current = 0.0
    psu = section.power_supply

    for comp in section.components:
        if comp.type == "motor":
            # Use stall current as worst-case for budget
            total_rated_current += comp.stall_current_a or 0.0
        elif comp.type == "relay" or comp.type == "switch":
            # Switches/relays consume negligible power for budget purposes
            # (unless we model the relay coil, which we don't yet in high-level budget)
            pass

    margin = psu.max_current_a - total_rated_current

    return {
        "total_rated_current_a": round(total_rated_current, 3),
        "max_capacity_a": round(psu.max_current_a, 3),
        "margin_a": round(margin, 3),
        "is_safe": total_rated_current <= psu.max_current_a,
        "warnings": ["Budget exceeded"]
        if total_rated_current > psu.max_current_a
        else [],
    }


def calculate_power_budget(circuit: Circuit, psu_config: PowerSupplyConfig) -> dict:
    """
    Calculate the power budget for the circuit using SPICE simulation (dynamic check).
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
