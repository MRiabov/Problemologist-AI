# Desired Architecture: WP3 — Electromechanical Integration

## Objective

Enable the design and verification of **electromechanical systems** — mechanisms that need electrical power to function. The agent shall design not just the physical parts but also the electrical wiring, power distribution, and control logic that drives motors, pumps, and actuators.

### Goals

1. **Power distribution**: Supply mains power (wall supply) to motors, pumps, and actuators with realistic wiring.
2. **Wiring in 3D**: Route cables through the physical assembly, ensuring they don't snag, tear, or interfere with moving parts.
3. **Circuit simulation**: Validate electrical correctness via SPICE — detect short circuits, under-voltage, overcurrent — before running the physics simulation.
4. **Unified electromechanical simulation**: Circuit state (which motors are energised) drives the physics simulation on every timestep.

### Non-goals (for now)

1. **PCB design** — adds complexity with low value; this is essentially a second product. PCBs can be revisited in WP6 (Productionizing) if needed.
2. **Analog signal integrity** — we don't need oscilloscope-grade SPICE. We need "is this motor on or off and is the circuit valid."
3. **Firmware / embedded code** — no Arduino, no MCU programming.
4. **Wireless / RF** — out of scope.
5. **Sensors** — deferred to WP7.

---

## Architecture Overview

```
┌───────────────────────────────────────────────────┐
│                 Agent Workspace                    │
│                                                   │
│  ┌──────────┐   ┌──────────────┐   ┌───────────┐ │
│  │ Mech     │◄─►│ Electrical   │──►│ PySpice   │ │
│  │ Engineer │   │ Engineer     │   │ Validator  │ │
│  └────┬─────┘   └──────┬───────┘   └─────┬─────┘ │
│       │                │                  │       │
│       ▼                ▼                  ▼       │
│  ┌─────────────────────────────────────────────┐  │
│  │         assembly_definition.yaml            │  │
│  │  (parts + wiring + circuit netlist)         │  │
│  └──────────────────┬──────────────────────────┘  │
│                     │                             │
│                     ▼                             │
│  ┌─────────────────────────────────────────────┐  │
│  │     Unified Simulation (MuJoCo/Genesis)     │  │
│  │  Circuit state → Motor commands per step    │  │
│  └─────────────────────────────────────────────┘  │
└───────────────────────────────────────────────────┘
```

---

## Circuit Modelling

### Technology: PySpice

We use **PySpice** (Python bindings to Ngspice SPICE engine) for circuit simulation. We do not need high-fidelity analog simulation. The purpose is to:

1. Validate that the circuit is electrically sound (no short circuits, no floating nodes).
2. Determine which motors/actuators receive power and at what voltage/current.
3. Detect overcurrent, under-voltage, and short-circuit conditions.

PySpice is suitable because:

- **Short circuit detection**: If the agent wires a switch directly across the supply, SPICE calculates massive current → fail with "Electronics Burnt."
- **Voltage arithmetic**: Two 24V supplies in series = 48V; in parallel = 24V. No manual logic needed — the solver handles it.
- **Lightweight**: No GUI dependencies, runs headless, pip-installable.

### Power Supply Model

Systems draw power from **mains/wall supply** (not batteries). The standard model is:

```yaml
power_supply:
  type: "mains_ac_rectified"    # Wall power → PSU → DC
  voltage_dc: 24.0              # V (standard industrial)
  max_current_a: 10.0           # A (PSU rating)
  # Agents must ensure total draw ≤ max_current_a
```

The agent can specify different PSU ratings. If total motor draw exceeds `max_current_a`, the circuit simulation fails with `overcurrent_supply`.

### Circuit Definition (Netlist)

The agent defines circuits as Python code using PySpice's netlist API. This fits the CodeAct pattern — the agent writes a script, not a tool call.

```python
from pyspice_utils import create_circuit, validate_circuit

circuit = create_circuit("motor_control")

# Power supply: 24V DC from wall PSU
circuit.V("supply", "vcc", circuit.gnd, 24)

# Switch (controlled by objective logic)
circuit.R("switch_on", "vcc", "motor_node", 0.1)  # ~0 Ω when ON

# Motor: modelled as resistive load (stall current ≈ V/R)
circuit.R("motor1", "motor_node", circuit.gnd, 12)  # 2A at 24V

result = validate_circuit(circuit)
# → {valid: true, motor1_current_a: 2.0, total_draw_a: 2.0, warnings: []}
```

### Circuit Validation Tool

```python
def validate_circuit(circuit: PySpiceCircuit) -> CircuitValidationResult:
    """
    Run DC operating point analysis on the circuit.
    Returns:
      - valid: bool (no short circuits, no floating nodes)
      - node_voltages: dict[str, float]
      - branch_currents: dict[str, float]
      - total_draw_a: float
      - errors: list[str]  (e.g. "SHORT_CIRCUIT at node X", "OVERCURRENT_SUPPLY")
      - warnings: list[str] (e.g. "Motor M1 under-voltage: 18V < 24V rated")
    """
```

This is a pre-simulation gate. If the circuit fails validation, the physics simulation does not run.

### Integration with `assembly_definition.yaml`

<!-- Note: preliminary_cost_estimation.yaml is being renamed to assembly_definition.yaml -->

The assembly definition gains an `electronics` section:

```yaml
electronics:
  power_supply:
    type: "mains_ac_rectified"
    voltage_dc: 24.0
    max_current_a: 10.0

  wiring:
    - wire_id: "w1"
      from: {component: "psu", terminal: "+"}
      to: {component: "motor_a", terminal: "+"}
      gauge_awg: 18
      length_mm: 350        # auto-calculated from 3D routing if routed
      routed_in_3d: true     # if true, wire has physical geometry in sim

    - wire_id: "w2"
      from: {component: "motor_a", terminal: "-"}
      to: {component: "psu", terminal: "-"}
      gauge_awg: 18
      length_mm: 400
      routed_in_3d: true

  components:
    - component_id: "psu"
      type: "power_supply"
      # PSU is a COTS part — referenced from parts.db
      cots_part_id: "PSU-24V-10A-DIN"

    - component_id: "motor_a"
      type: "motor"
      # References the motor defined in final_assembly.parts
      assembly_part_ref: "feeder_motor_env_v1"
      rated_voltage: 24.0
      stall_current_a: 3.5
```

---

## Wiring in 3D Space

### Physical Wire Model

Wires are physical objects in the simulation. They can:

- Be pulled taut, sag under gravity, and snag on moving parts.
- Tear if stress exceeds the wire's rated tensile strength.
- Cause simulation failure if torn (electrical disconnection).

### Physics Implementation

- **MuJoCo**: Use `tendon` elements (site-to-site cables with length limits).
- **Genesis**: Use native tendon/cable primitives.

The wire's physical path is defined by waypoints (routing points through the assembly), and the simulation engine interpolates between them.

### Wire Routing

The agent routes wires by specifying waypoints in 3D space:

```python
from wire_utils import route_wire

route_wire(
    wire_id="w1",
    waypoints=[
        (0, 0, 50),      # PSU terminal
        (10, 0, 50),     # Along frame rail
        (10, 30, 50),    # Turn toward motor
        (15, 30, 45),    # Motor terminal
    ],
    gauge_awg=18,
    attach_to=["frame_rail"]  # Wire is clipped to this part
)
```

### Wire Validation

Before simulation:

1. **Clearance check**: Wire path doesn't intersect solid parts (except at attached points).
2. **Length check**: Total wire length is consistent with the netlist definition.
3. **Bend radius**: Wire doesn't bend tighter than the minimum bend radius for its gauge.

During simulation:

1. **Tension monitoring**: If wire tension exceeds rated tensile strength → `wire_torn` failure.
2. **Interference detection**: If a moving part contacts a wire and the wire can't deflect → warning or failure depending on severity.

---

## Unified Electromechanical Simulation

### Relationship to Existing Motor Controllers

The main spec defines motor behaviour via **abstract controller functions** (`sinusoidal`, `constant`, `square`, or user-defined) that directly command MuJoCo/Genesis actuators. These act as an "invisible controller" — they define *how* the motor moves.

WP3 does **not** replace this system. The circuit layer adds a **power gate** on top: it determines *whether* the motor can move. If a motor isn't wired to power, or if a wire tears mid-simulation, the motor stops — regardless of what the controller function says.

```
effective_torque = controller_function(t) × is_powered(motor_id, t)
```

Where:

- `controller_function(t)` — the existing MVP motor control (sinusoidal, constant, etc.) as defined in `assembly_definition.yaml`
- `is_powered(motor_id, t)` — a signal from the circuit simulation: `1.0` if fully powered, `0.0` if disconnected, or degraded (e.g. `0.75`) if under-voltage

This means the agent still defines `control.mode: sinusoidal` as before, but **also** needs to wire the motor to a power supply for it to function.

### Simulation Loop Integration

The circuit and physics simulations are coupled per-timestep:

```
For each simulation timestep t:
  1. Evaluate circuit state → which motors are powered (binary/degraded)
  2. For each motor: effective_torque = controller_fn(t) × is_powered(motor, t)
  3. Apply effective torques to MuJoCo/Genesis actuators
  4. Step physics simulation
  5. Check wire tensions and physical integrity
  6. Check for objective completion / failure
```

Consequences:

- A motor only spins if it is both wired to power **and** has a controller function.
- If a wire tears mid-simulation, `is_powered()` drops to 0, the motor stops.
- Pre-WP3 benchmarks (no `electronics` section) implicitly have `is_powered() = 1.0` for all motors — full backward compatibility.

---

## The Electronics Engineer Agent

### Purpose

A specialized sub-agent responsible for the electrical aspects of the design. It works alongside the Mechanical Engineer agent.

### Responsibilities

1. **Circuit design**: Define the electrical netlist connecting power supply → switches/relays → motors/actuators.
2. **Wire routing**: Route physical cables through the 3D assembly.
3. **Power budget validation**: Ensure total current draw doesn't exceed PSU rating.
4. **Circuit simulation**: Run PySpice validation before physics simulation.

### Interaction with Mechanical Engineer

The interaction is sequential with iteration:

1. **Mech Engineer** designs the assembly (parts, motors, actuators, positions).
2. **Mech Engineer** hands off to **Elec Engineer** with: assembly geometry, motor locations, power requirements.
3. **Elec Engineer** designs the circuit, routes wires, validates electrically.
4. **Elec Engineer** hands back the wired assembly.
5. If wire routing conflicts with the mechanism (e.g., wire passes through a moving arm's sweep), the Elec Engineer requests the Mech Engineer to modify. Loop until resolved.

### Handover Data

**Mech → Elec:**

- `assembly_definition.yaml` with `final_assembly` (parts, joints, motors with rated specs)
- 3D geometry (STEP/OBJ files) for wire clearance checks

**Elec → Mech (additions):**

- `electronics` section in `assembly_definition.yaml` (netlist, wiring, power supply)
- Wire geometry (waypoints) for inclusion in the physics simulation

### Communication Protocol

Both agents share the same workspace filesystem. The Elec Engineer writes the `electronics` section of `assembly_definition.yaml` and wire routing scripts. The Mech Engineer can read but not modify the electronics section (enforced by file validation).

This is implemented as additional LangGraph nodes in the engineering workflow:

```
Planner → Mech CAD Engineer → Elec Engineer → Reviewer
                ↑__________________________|  (if conflicts)
```

---

## Agent Tools

### New tools for the Electrical Engineer

```python
# Circuit tools
def validate_circuit(circuit: PySpiceCircuit) -> CircuitValidationResult: ...
def simulate_circuit_transient(circuit, duration_s, timestep_s) -> TransientResult: ...

# Wiring tools
def route_wire(wire_id, waypoints, gauge_awg, attach_to) -> WireRouteResult: ...
def check_wire_clearance(wire_routes, assembly) -> ClearanceResult: ...

# Power budget
def calculate_power_budget(circuit, motors) -> PowerBudgetResult: ...
```

### Modified existing tools

- `simulate(Compound)`: Now also requires a valid circuit definition. Rejects simulation if `validate_circuit()` hasn't passed.
- `validate_and_price(Compound)`: Now includes wire and COTS electrical component costs.

---

## COTS Electrical Components

Electrical components (PSUs, relays, connectors, switches) are added to the **existing** `parts.db` COTS catalog — not a separate database.

New component categories:

- `power_supply` — DIN-rail PSUs, bench supplies
- `relay` — electromechanical relays for motor switching
- `connector` — terminal blocks, wire connectors
- `wire` — by gauge, priced per meter

The existing COTS search subagent handles queries for electrical components in the same way it handles mechanical ones.

---

## Failure Modes

### Electrical failures (pre-simulation, from PySpice)

| Failure | Detection | Result |
|---------|-----------|--------|
| Short circuit | Current > 100× expected | `FAILED_SHORT_CIRCUIT` |
| Overcurrent (supply) | Total draw > PSU rating | `FAILED_OVERCURRENT_SUPPLY` |
| Overcurrent (wire) | Wire current > gauge rating | `FAILED_OVERCURRENT_WIRE` |
| Under-voltage | Motor receives < 80% rated voltage | Warning (soft) |
| Floating node | Unconnected circuit node | `FAILED_OPEN_CIRCUIT` |

### Physical failures (during simulation)

| Failure | Detection | Result |
|---------|-----------|--------|
| Wire torn | Tendon tension > rated tensile | `FAILED_WIRE_TORN` |
| Wire interference | Wire-body collision with no slack | Warning → fail if persistent |
| Motor without power | Circuit disconnection mid-sim | Motor stops (may cause objective failure) |

---

## Evaluation Criteria

Following the main spec's evaluation pattern:

### Fast evals

1. Circuit netlist parses without errors in 95% of cases.
2. Wire routing scripts execute without exceptions in 90% of cases.
3. Power budget calculations are numerically correct in 99% of cases.

### Medium evals

1. Given a mechanism with motors, the Elec Engineer produces a valid circuit in 80% of cases (first attempt), 95% after one retry.
2. Wire routes don't intersect solid parts in 85% of cases.
3. Power budget is correctly sized (not over-provisioned by >200%, not under-provisioned) in 90% of cases.
4. The agent uses the COTS catalog for PSU and connector selection in 70% of cases (vs. hallucinating specs).

### Slow evals (end-to-end)

1. Given a benchmark requiring powered motors, the combined Mech+Elec agents produce a working electromechanical system in 60% of cases within 40 tool calls.
2. Wire routing survives runtime jitter (no tears across 5 seeds) in 70% of successful solutions.
3. Circuit state correctly gates motor behaviour in 95% of simulations — motors don't spin without power.

---

## Observability Events

New domain events added to the event tracking system:

1. `circuit_validation` — result (pass/fail), error type, branch currents
2. `wire_routing` — wire count, total length, clearance pass/fail
3. `power_budget_check` — total draw vs. PSU capacity, margin %
4. `electrical_failure` — failure type (short circuit, wire torn, etc.)
5. `elec_agent_handover` — Mech→Elec and Elec→Mech handover events with iteration count
6. `circuit_simulation` — transient sim duration, motor states over time

---

## Delivery / Frontend

1. **Schematic view**: Display the circuit netlist as a simple schematic diagram (auto-generated from PySpice netlist).
2. **Wire view**: Visualize wire routes in the 3D viewer (coloured lines/tubes following waypoints).
3. **Script view**: Existing code artifact viewer shows the circuit definition scripts.
4. (Extra) **Circuit timeline**: Show which motors were powered at which simulation timestep — a simple ON/OFF timeline chart.

---

## Future Work

1. **PCB design** — if production-ready outputs are needed (WP6), integrate KiCad/SKiDL for PCB layout generation.
2. **Sensors** — buttons, limit switches, encoders (WP7).
3. **Firmware** — running logic on simulated MCUs.
4. **Thermal simulation** — heat from high-current wires and motors.

---

## `objectives.yaml` Schema Extension

Electromechanical benchmarks extend `objectives.yaml` with:

```yaml
# Additional fields for electromechanical benchmarks
electronics_requirements:
  power_supply_available:
    type: "mains_ac_rectified"
    voltage_dc: 24.0
    max_current_a: 10.0
    location: [0, -30, 0]          # Where the PSU is in the env

  wiring_constraints:
    max_total_wire_length_mm: 2000  # Budget for total wiring
    restricted_zones:               # Where wires cannot pass
      - name: "high_heat_zone"
        min: [10, 10, 0]
        max: [20, 20, 30]

  # Circuit must be valid for simulation to proceed
  circuit_validation_required: true
```
