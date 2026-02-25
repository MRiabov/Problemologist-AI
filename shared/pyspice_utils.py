import structlog
from typing import Any

import ctypes.util

from PySpice.Spice.Netlist import Circuit
from PySpice.Spice.NgSpice.Shared import NgSpiceShared
from PySpice.Unit import *

# Robustly locate ngspice library if not found by default
if NgSpiceShared.LIBRARY_PATH.startswith("libngspice"):
    # ctypes.util.find_library might find it in non-standard paths
    # or handle the .so.0 vs .so issue
    lib_path = ctypes.util.find_library("ngspice")
    if lib_path:
        NgSpiceShared.LIBRARY_PATH = lib_path

from shared.models.schemas import (
    CircuitValidationResult,
    ElectronicsSection,
    PowerSupplyConfig,
)
from shared.enums import FailureReason, ElectronicComponentType
from shared.models.simulation import SimulationFailure
from shared.circuit_builder import resolve_node_name

# NGSpice 42 compatibility monkeypatch
# NGSpice 42 sends "Using SPARSE 1.3" to stderr, which PySpice treats as an error.
if not hasattr(NgSpiceShared, "_patched_problemologist"):
    _original_send_char = NgSpiceShared._send_char

    @staticmethod
    def _patched_send_char(message_c, ngspice_id, user_data):
        from PySpice.Spice.NgSpice.Shared import ffi, ffi_string_utf8

        try:
            # Safely resolve self and message
            self = ffi.from_handle(user_data)
            message = ffi_string_utf8(message_c)
        except Exception:
            return 0

        prefix, _, content = message.partition(" ")
        if prefix == "stderr" and (
            "SPARSE" in content
            or content.startswith("Note:")
            or "singular matrix" in content.lower()
        ):
            if hasattr(self, "_stdout"):
                self._stdout.append(content)
            return 0
        return _original_send_char(message_c, ngspice_id, user_data)

    NgSpiceShared._send_char = _patched_send_char
    NgSpiceShared._patched_problemologist = True

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
    failures: list[SimulationFailure] = []
    warnings = []

    # Proactive open-circuit check (INT-124)
    if section:
        resolved_connected_nodes = set()
        for wire in section.wiring:
            resolved_connected_nodes.add(
                resolve_node_name(
                    wire.from_terminal.component, wire.from_terminal.terminal
                )
            )
            resolved_connected_nodes.add(
                resolve_node_name(wire.to_terminal.component, wire.to_terminal.terminal)
            )

        for comp in section.components:
            required = []
            if comp.type == ElectronicComponentType.MOTOR:
                required = ["+", "-"]
            elif comp.type in [ElectronicComponentType.SWITCH, ElectronicComponentType.RELAY]:
                required = ["in", "out"]
            elif comp.type == ElectronicComponentType.POWER_SUPPLY:
                required = ["+", "-"]

            for term in required:
                target_node = resolve_node_name(comp.component_id, term)
                if target_node not in resolved_connected_nodes:
                    failures.append(
                        SimulationFailure(
                            reason=FailureReason.OPEN_CIRCUIT,
                            detail=f"Component {comp.component_id} terminal {term} is not connected.",
                        )
                    )

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
                    f = SimulationFailure(
                        reason=FailureReason.OPEN_CIRCUIT,
                        detail=f"Floating nodes detected (singular matrix): {line.strip()}",
                    )
                    return CircuitValidationResult(
                        valid=False,
                        errors=[str(f)],
                        failures=[f],
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

        sources_draw = {}  # source_name -> current_draw

        for name, current in branch_currents.items():
            current_val = float(current)
            name_lower = name.lower()

            # Detect extreme currents which indicate shorts (INT-120)
            if abs(current_val) > 100.0:
                failures.append(
                    SimulationFailure(
                        reason=FailureReason.SHORT_CIRCUIT,
                        detail=f"Extreme current in branch {name}: {current_val:.2f}A",
                    )
                )

            if name_lower.startswith("v"):
                sources_draw[name_lower] = abs(current_val)

        if "vsupply" in sources_draw:
            total_draw = sources_draw["vsupply"]
            if total_draw > max_current:
                failures.append(
                    SimulationFailure(
                        reason=FailureReason.OVERCURRENT,
                        detail=f"Total draw {total_draw:.2f}A exceeds PSU rating {max_current:.2f}A",
                    )
                )

        if section:
            for comp in section.components:
                if comp.type == ElectronicComponentType.POWER_SUPPLY:
                    v_name = f"v{comp.component_id}".lower()
                    if v_name in sources_draw:
                        draw = sources_draw[v_name]
                        # Use stall_current_a as max_current for now
                        limit = comp.stall_current_a or 10.0
                        if draw > limit:
                            failures.append(
                                SimulationFailure(
                                    reason=FailureReason.OVERCURRENT,
                                    detail=f"Battery {comp.component_id} draw {draw:.2f}A exceeds limit {limit:.2f}A",
                                )
                            )

        # T007: Per-wire overcurrent check (INT-123)
        if section and section.wiring:
            from shared.wire_utils import get_awg_properties

            for wire in section.wiring:
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
                # or look it up: R_wire_id. PySpice prepends 'R' to resistors.
                r_val = 0.01
                element_name = f"R{wire.wire_id}"
                if element_name in circuit.element_names:
                    r_val = float(circuit[element_name].resistance)
                elif wire.gauge_awg and wire.length_mm:
                    props = get_awg_properties(wire.gauge_awg)
                    r_val = props["resistance_ohm_m"] * (wire.length_mm / 1000.0)

                current = abs(v1 - v2) / max(r_val, 0.0001)

                limit = get_awg_properties(wire.gauge_awg)["max_current_a"]
                if current > limit:
                    failures.append(
                        SimulationFailure(
                            reason=FailureReason.OVERCURRENT,
                            detail=f"Wire {wire.wire_id} (AWG {wire.gauge_awg}) carrying {current:.2f}A exceeds limit {limit:.2f}A",
                        )
                    )

        # Detect floating nodes: check if any node has near-zero conductance to everything else
        # PySpice usually throws an exception for this, but we can also check for extremely high voltages
        # or non-finite values if SPICE managed to return something.
        for node, voltage in node_voltages.items():
            if not np.isfinite(voltage) or abs(voltage) > 1e6:
                failures.append(
                    SimulationFailure(
                        reason=FailureReason.OPEN_CIRCUIT,
                        detail=f"Node {node} has unstable voltage: {voltage}",
                    )
                )

        return CircuitValidationResult(
            valid=len(failures) == 0,
            node_voltages=node_voltages,
            branch_currents=branch_currents,
            total_draw_a=total_draw,
            errors=[str(f) for f in failures],
            failures=failures,
            warnings=warnings,
        )
    except Exception as e:
        error_msg = str(e).strip()
        if "Singular matrix" in error_msg or "indefinite matrix" in error_msg:
            f = SimulationFailure(
                reason=FailureReason.OPEN_CIRCUIT,
                detail="Floating nodes or unconnected circuit components detected.",
            )
            return CircuitValidationResult(
                valid=False,
                errors=[str(f)],
                failures=[f],
                total_draw_a=0.0,
            )

        f = SimulationFailure(
            reason=FailureReason.VALIDATION_FAILED,
            detail=f"Circuit simulation failed: {error_msg}",
        )
        return CircuitValidationResult(
            valid=False,
            errors=[str(f)],
            failures=[f],
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
        if comp.type == ElectronicComponentType.MOTOR:
            # Use stall current as worst-case for budget
            total_rated_current += comp.stall_current_a or 0.0
        elif comp.type == ElectronicComponentType.RELAY or comp.type == ElectronicComponentType.SWITCH:
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
        "failures": res.failures,
    }
