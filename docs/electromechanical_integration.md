# Electromechanical Integration

The Problemologist AI system supports integrated electromechanical design, allowing agents to not only create mechanical assemblies but also design the circuits and routing required to power and control them.

## Key Components

### 1. Electronics Engineer Agent
The `electronics_engineer` agent is specialized in:
- Defining electrical netlists using PySpice.
- Selecting appropriate COTS components (PSUs, motors, sensors).
- Routing physical wires in 3D space.
- Validating power requirements against supply limits.

### 2. PySpice Integration
The system uses PySpice to simulate and validate circuits. Agents define the netlist, and the worker executes the simulation to check for:
- Node voltages.
- Branch currents.
- Total power consumption.

### 3. Wire Routing
Physical wire routing is handled through the `wire_utils.py` and specific builder tools.
- `route_wire(from, to, waypoints)`: Defines the physical path of a wire.
- `check_wire_clearance()`: Ensures wires do not intersect with moving parts or static geometry.

## Data Schema

Electronics information is stored in the `electronics` section of `assembly_definition.yaml`:

```yaml
electronics:
  power_supply:
    voltage_dc: 12.0
    max_current_a: 10.0
  wiring:
    - wire_id: "motor_power"
      from: { component: "psu", terminal: "v+" }
      to: { component: "motor_1", terminal: "vcc" }
      gauge_awg: 18
      routed_in_3d: true
  components:
    - component_id: "motor_1"
      type: "motor"
      stall_current_a: 2.5
```

## Evaluation & Benchmarking

### 1. Electromechanical Benchmarks
The system includes a suite of 10 electromechanical benchmarks in `evals/datasets/electronics_engineer.json`. These focus on DC power distribution, motor connectivity, and 3D wire routing under various geometric constraints.

### 2. Running Evaluations
To measure the performance of the system on electromechanical tasks, use the `evals/run_evals.py` script:

```bash
python evals/run_evals.py --agent electronics_engineer
```

This script generates a report with the following metrics:
- **Electrical Validity Rate**: Percentage of designs passing schematic and simulation checks (no shorts, valid DC path).
- **Wire Integrity Rate**: Success rate of physical routing and clearance validation against mechanical geometry.
- **Power Efficiency Score**: Measures how well the selected power supply matches the estimated motor load.

## Best Practices for Agents
- **Common Ground**: Always explicitly define a `GND` node in the PySpice netlist for all return paths.
- **Gauge Selection**: Use appropriate AWG ratings for high-current motors to avoid simulation failures.
- **Clearance Checking**: Always call `check_wire_clearance()` after routing to ensure wires do not intersect with moving parts or rigid obstacles.
