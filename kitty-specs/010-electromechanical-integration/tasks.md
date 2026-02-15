# Tasks: Electromechanical Integration (WP3)

This feature enables the design and verification of electromechanical systems, bridging circuit simulation with 3D physics.

## Phase 1: Foundation & Schemas
### WP01: Foundation & Schemas
- **Goal**: Define the data structures and persistence layer for electromechanical integration.
- **Priority**: High
- **Success Criteria**: `assembly_definition.yaml` supports electronics; `parts.db` contains electrical components.
- **Subtasks**:
    - [x] T001: [P] Update `shared/models/schemas.py` with `ElectronicsSection` (PSU, wiring, components) and update `ObjectivesYaml`.
    - [x] T002: [P] Create `shared/enums.py` extensions for electrical components and failure types (SHORT_CIRCUIT, WIRE_TORN).
    - [x] T003: [P] Update `assembly_definition.yaml` validation logic in `worker/utils/file_validation.py`.
    - [x] T004: [P] Add migration to `parts.db` for electrical component categories (power_supply, relay, connector, wire).
- **Implementation Sketch**: Start with Pydantic models, then update enums, then validation logic, and finally the DB migration.
- **Dependencies**: None
- **Estimated Prompt Size**: ~250 lines

## Phase 2: Simulation Core
### WP02: Circuit Simulation & Validation
- **Goal**: Implement the electrical validation gate using PySpice.
- **Priority**: High
- **Success Criteria**: Netlists can be generated and validated for shorts/overcurrent.
- **Subtasks**:
    - [x] T005: [P] Implement `shared/pyspice_utils.py`: setup Ngspice, basic simulation wrapper, and result parsing.
    - [x] T006: [P] Implement `shared/circuit_builder.py`: netlist generation from `ElectronicsSection`.
    - [x] T007: [P] Implement `validate_circuit` function: detect shorts, overcurrent, floating nodes.
    - [x] T008: [P] Add `calculate_power_budget` utility.
    - [x] T009: [P] Add unit tests for circuit validation logic.
- **Implementation Sketch**: Setup PySpice first, then build the netlist generator, then the validation logic, and finish with tests.
- **Dependencies**: WP01
- **Estimated Prompt Size**: ~350 lines

### WP03: 3D Wire Routing Infrastructure
- **Goal**: Enable the AI to define physical paths for wires in 3D space.
- **Priority**: Medium
- **Success Criteria**: Spline-based wire paths can be generated and checked for solid penetration.
- **Subtasks**:
    - [x] T010: [P] Implement `shared/wire_utils.py`: spline interpolation for waypoints.
    - [x] T011: [P] Implement `check_wire_clearance` using `build123d`/`trimesh` to prevent volume intersection.
    - [x] T012: [P] Implement `route_wire` helper for agents to define splines and attachments.
    - [x] T013: [P] Add wire gauge property lookup (resistance per meter, tensile strength).
- **Implementation Sketch**: Focus on the spline math first, then the collision/clearance logic using trimesh/build123d.
- **Dependencies**: WP01
- **Estimated Prompt Size**: ~300 lines

### WP04: Physics Simulation Integration
- **Goal**: Couple the circuit state with the physics simulation.
- **Priority**: High
- **Success Criteria**: Motors only spin when powered; wires can tear under tension.
- **Subtasks**:
    - [x] T014: [P] Update `worker/simulation/builder.py`: inject `tendon` and `site` elements into MJCF for 3D wires.
    - [x] T015: [P] Update `worker/simulation/loop.py`: implement `is_powered` signal logic per actuator.
    - [x] T016: [P] Implement tension monitoring in `SimulationLoop`: check tendon force vs tensile strength.
    - [x] T017: [P] Implement mid-simulation failure handling for `electrical_failure` events.
    - [x] T018: [P] Add simulation tests for power gating (motor stops when wire "breaks").
- **Implementation Sketch**: Inject tendons into MJCF first, then modify the simulation loop to modulate torque based on circuit state.
- **Dependencies**: WP02, WP03
- **Estimated Prompt Size**: ~400 lines

## Phase 3: Agentic Intelligence
### WP05: Electrical Engineer Agent & Workflow
- **Goal**: Introduce the specialized sub-agent and its handover protocol.
- **Priority**: High
- **Success Criteria**: The agent can design circuits and route wires autonomously in the graph.
- **Subtasks**:
    - [ ] T019: [P] Create `controller/agent/nodes/electronics_engineer.py` node.
    - [ ] T020: [P] Define Electrical Engineer system prompt in `config/prompts.yaml`.
    - [ ] T021: [P] Implement tools for Electrical Engineer: `validate_circuit`, `route_wire`, `search_electrical_cots`.
    - [ ] T022: [P] Update `controller/agent/graph.py` with handover logic (Mech -> Elec -> Reviewer).
    - [ ] T023: [P] Implement `todo.md` checklist generation for electrical tasks.
    - [ ] T024: [P] Add integration tests for the Mech+Elec agent loop.
- **Implementation Sketch**: Define the node and prompt first, then the tools, and finally wire them into the LangGraph.
- **Dependencies**: WP02, WP03, WP04
- **Estimated Prompt Size**: ~450 lines

## Phase 4: Delivery
### WP06: Observability & Frontend
- **Goal**: Provide visual feedback and telemetry for electromechanical systems.
- **Priority**: Medium
- **Success Criteria**: Users can see schematics and 3D wires in the dashboard.
- **Subtasks**:
    - [ ] T025: [P] Update `shared/observability/schemas.py` with electrical events.
    - [ ] T026: [P] Update `controller/api/` to export circuit data for `tscircuit`.
    - [ ] T027: [P] Implement 3D wire rendering in `frontend/src/viewer/`.
    - [ ] T028: [P] Add "Electronics View" toggle in the dashboard.
    - [ ] T029: [P] Integrate `tscircuit` component for schematic visualization.
- **Implementation Sketch**: Add observability events first, then the API endpoints, and finally the frontend visualization components.
- **Dependencies**: WP05
- **Estimated Prompt Size**: ~350 lines

### WP07: Evals & Documentation
- **Goal**: Validate the feature with benchmarks and provide documentation.
- **Priority**: Medium
- **Success Criteria**: 10+ electromechanical benchmarks pass; documentation is up-to-date.
- **Subtasks**:
    - [ ] T030: [P] Generate 10+ electromechanical benchmarks in `evals/datasets/`.
    - [ ] T031: [P] Implement `run_evals.py` extensions for electrical metrics.
    - [ ] T032: [P] Perform prompt tuning for the Electrical Engineer node.
    - [ ] T033: [P] Documentation: update `docs/electromechanical_integration.md`.
- **Implementation Sketch**: Generate benchmarks, run them, tune prompts based on results, and then write final docs.
- **Dependencies**: WP05, WP06
- **Estimated Prompt Size**: ~300 lines
