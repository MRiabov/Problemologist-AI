# Tutorial: Designing Electromechanical Systems

This tutorial walks you through creating a simple electromechanical assembly: a powered fan.

## Step 1: Define the Mechanical Context
The Mechanical Engineer first creates the fan blades, housing, and motor mount.

```python
from build123d import *
# ... (mechanical code)
```

## Step 2: Design the Circuit
The Electronics Engineer defines the power connections.

```python
import PySpice.Logging.Logging as logging
from PySpice.Spice.Netlist import Circuit

circuit = Circuit('Fan Power')
circuit.V('input', 'vcc', 'gnd', 12)
# Connect motor terminals to VCC and GND
# ...
```

## Step 3: Route the Wires
Once the circuit is defined, we must place the wires in the 3D model.

```python
from shared.wire_utils import route_wire

# Route from PSU to Motor
route_wire(
    from_pos=(0, 0, 0),
    to_pos=(10, 50, 5),
    waypoints=[(5, 25, 10)],
    label="vcc_to_motor"
)
```

## Step 4: Validate
The system automatically checks:
1. **Simulation**: Does the circuit work? (No shorts, valid DC loop)
2. **Clearance**: Are wires hitting the fan blades?
3. **Power**: Is the 12V supply enough for the motor?

## Complete Assembly Definition Example

The following `assembly_definition.yaml` snippet shows a fully defined electromechanical system:

```yaml
electronics:
  power_supply:
    part_id: "batt-li-3s-2200"
    voltage_dc: 11.1
    max_current_a: 30.0
  wiring:
    - wire_id: "m1_pwr"
      from: { component: "psu", terminal: "pos" }
      to: { component: "motor_1", terminal: "pos" }
      gauge_awg: 18
      routed_in_3d: true
      waypoints: [[10, 5, 2], [15, 20, 2]]
  components:
    - component_id: "motor_1"
      part_id: "n20-12v-300rpm"
      type: "motor"
      stall_current_a: 0.8
```

## Prompt Engineering for Electromechanical Tasks

When prompting the agent for electromechanical designs, follow these tips:
- **Mention Voltage Rails**: Explicitly state the required voltage for the motors (e.g., 12V or 24V).
- **Define Power Constraints**: Clearly state the expected load if multiple motors are used.
- **Enforce Safety**: Ask the agent to ensure wires avoid heat-generating components or high-friction areas.

## Common Pitfalls
- **Forgetting Common Ground**: All components MUST share a GND node in the netlist.
- **Undersized Wires**: High-current paths require thicker wires (lower AWG).
- **Routing through Rotors**: Always check 3D clearance against moving mechanical parts.
