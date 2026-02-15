---
work_package_id: WP02
title: Circuit Simulation & Validation
lane: "done"
dependencies: []
base_branch: main
base_commit: 0f015147c8010750ea277c85719f4e8cd6b83ccb
created_at: '2026-02-15T07:39:25.526010+00:00'
subtasks: [T005, T006, T007, T008, T009]
shell_pid: "38220"
agent: "gemini-cli"
reviewed_by: "MRiabov"
review_status: "approved"
---

# WP02: Circuit Simulation & Validation

## Objective
Implement a circuit validation gate using PySpice to detect electrical failures before physics simulation.

## Context
We use `PySpice` to run DC operating point analysis. This allows us to calculate currents and voltages across all components. If a short circuit or overcurrent is detected, we fail the simulation early.

## Detailed Guidance

### T005: Implement `shared/pyspice_utils.py`
**Purpose**: Setup the bridge to Ngspice.

**Steps**:
1. Create a wrapper for PySpice `Circuit` objects.
2. Implement a method to run `.op()` (operating point) analysis.
3. Handle common PySpice exceptions and map them to our internal failure types.

### T006: Implement `shared/circuit_builder.py`
**Purpose**: Convert our `ElectronicsSection` model into a PySpice netlist.

**Steps**:
1. Map `PowerSupplyConfig` to a Voltage Source (`V`).
2. Map `ElectronicComponent` (motors) to Resistors (`R`) based on stall current (`R = V/I`).
3. Handle serial/parallel connections if specified (initially assume simple parallel branches for motors).

### T007: Implement `validate_circuit` function
**Purpose**: Analyze simulation results for safety.

**Steps**:
1. Check `V_source.current`: if > `max_current_a`, trigger `OVERCURRENT_SUPPLY`.
2. Check for node voltages: if a node is at `0V` but should be at `Vcc`, check for shorts.
3. Detect "floating nodes" (nodes with no path to ground).

### T008: Add `calculate_power_budget` utility
**Purpose**: Simple static check before running SPICE.

**Steps**:
1. Sum up all `rated_current` values of components.
2. Compare with PSU `max_current_a`.

### T009: Add unit tests
**Purpose**: Ensure validation logic is robust.

**Steps**:
1. Test a "happy path" circuit (1 PSU, 2 motors).
2. Test a "short circuit" case (0.01 Ohm resistor across PSU).
3. Test an "overcurrent" case (too many motors).

## Definition of Done
- `validate_circuit` correctly identifies shorts and overcurrents.
- Netlists are successfully generated from `assembly_definition.yaml`.
- Tests pass.

## Risks
- PySpice/Ngspice not being correctly installed in the environment.
- Complex netlists causing convergence issues in SPICE.

## Activity Log

- 2026-02-15T07:39:25Z – gemini – shell_pid=351162 – lane=doing – Assigned agent via workflow command
- 2026-02-15T08:04:13Z – gemini – shell_pid=351162 – lane=for_review – Implemented circuit simulation and validation with PySpice.
- 2026-02-15T08:39:08Z – gemini-cli – shell_pid=38220 – lane=doing – Started review via workflow command
- 2026-02-15T08:40:54Z – gemini-cli – shell_pid=38220 – lane=done – Review passed: Implementation successfully integrates PySpice for circuit validation, provides accurate netlist generation from assembly models, and includes comprehensive unit and integration tests.
