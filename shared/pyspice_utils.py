import logging
from typing import Any
from PySpice.Spice.Netlist import Circuit
from PySpice.Unit import *
from shared.models.schemas import CircuitValidationResult, PowerSupplyConfig

logger = logging.getLogger(__name__)


def create_circuit(name: str) -> Circuit:
    """Create a new PySpice circuit."""
    return Circuit(name)


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
