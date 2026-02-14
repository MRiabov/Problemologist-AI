# Specification: Electromechanical Integration (WP3)

## Summary

Enable the design and verification of electromechanical systems within the Agentic CAD environment. This feature introduces electrical circuit design (schematics), circuit validation (safety and correctness), physical wire routing in 3D space, and a dedicated Electrical Engineer sub-agent to handle these specialized tasks. The system will bridge the gap between mechanical mechanisms and the power systems required to drive them.

## Motivation

Modern engineering involves complex integration of mechanics and electronics. By enabling the AI to design and validate electrical systems alongside mechanical ones, we move closer to "full-stack" engineering simulation. This prevents unrealistic designs where motors spin without power or wires are routed through moving parts, leading to high-fidelity, manufacturable designs.

## Success Criteria

1. **Electrically Valid Design**: AI-designed circuits must be electrically sound (no shorts, valid node connections) in at least 80% of first attempts.
2. **Power Gating Enforcement**: Wired motors MUST operate if and only if they are properly powered. Power gating correctness must be 100%.
3. **Physical Integrity**: Simulation must detect and report wire tears if tension exceeds rated strength, or clearance violations if wires intersect solid volumes.
4. **Frontend Visualization**: Users can switch to an "Electronics View" in the dashboard, where the electrical net is rendered using `tscircuit`, and physical wires are visualized in the 3D viewer.

## User Scenarios

### Scenario 1: Designing a Powered Conveyor

A user prompts the agent to "design a conveyor belt that moves objects at 0.5m/s using a 24V motor."

1. The Mechanical Engineer agent designs the conveyor structure and selects a suitable motor.
2. The handoff occurs to the Electrical Engineer agent.
3. The Electrical Engineer agent designs a circuit connecting a 24V PSU to the motor via a switch/relay.
4. The Electrical Engineer agent routes the wires through the conveyor frame, ensuring avoidence of the belt's path.
5. The system validates the circuit and starts the simulation.
6. The user views the schematic in the dashboard and sees the motor spinning only when the circuit is energized.

### Scenario 2: Detecting Electrical Failure

An agent incorrectly routes a wire through a moving shear mechanism.

1. During simulation, the shear moves and intercepts the wire path.
2. The simulation engine detects the intersection and lack of slack, resulting in a `FAILED_WIRE_TORN` event.
3. The Electrical Engineer agent is notified of the failure and must re-route the wire.

## Functional Requirements

### FR-1: Circuit Modelling & Validation

- Support circuit definition using SKiDL/PySpice paradigms.
- Validate netlists for:
  - Short circuits (current > 100x rating).
  - Overcurrent (total draw > PSU rating).
  - Floating nodes/open circuits.
- The validation result must serve as a "gate" preventing physics simulation if failures are found.

### FR-2: Power Gating Logic

- Map circuit states to motor actuators.
- Implement an `is_powered(motor_id, t)` signal that modulates motor torque.
- Ensure full backward compatibility: mechanisms without defined electronics are "powered by default."

### FR-3: 3D Wire Routing

- Define wires as 3D splines/arcs defined by waypoints.
- Attach wires to specific part surfaces.
- Enforce clearance checks: wires must not penetrate solid parts except at attachment points.
- Implement tension monitoring and breakage detection based on material properties (tensile strength).

### FR-4: Electrical Engineer Sub-Agent

- Introduce a specialized agent node using LangGraph.
- Implement a handover protocol: Mech Engineer (Assembly + Geometry) -> Elec Engineer (Circuit + Wiring).
- Support iterative conflict resolution when wiring issues occur.

### FR-5: Frontend Integration

- Export electrical nets in a format compatible with `tscircuit`.
- Provide an "Electronics Viewing Mode" in the dashboard.
- Render physical wires as 3D tubes/lines in the CAD viewer.

## Key Entities

- **Circuit**: The logical netlist of components and connections.
- **Power Supply (PSU)**: The source of electrical energy (Mains AC -> DC).
- **Wire**: Physical connector with attributes (gauge, length, tensile strength, physical path).
- **Motor/Actuator**: Electromechanical component gated by the circuit.
- **Circuit State**: Real-time voltage/current values for each node/branch.

## Assumptions

- We focus on DC power systems for motors and actuators (Mains supply is rectified).
- Initial complexity is limited: no complex logic (firmware/MCU) or sensors (WP7).
- PySpice/Ngspice is available in the worker environment.
- Wiring materials are standard (e.g., copper gauge based on AWG).
