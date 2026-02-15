---
work_package_id: WP05
title: Electrical Engineer Agent & Workflow
lane: "doing"
dependencies: []
base_branch: main
base_commit: 1472e7bc3d8d7a12f56ca01e975a236920a5b7d5
created_at: '2026-02-15T10:17:49.427911+00:00'
subtasks: [T019, T020, T021, T022, T023, T024]
shell_pid: "121941"
agent: "gemini-cli"
---

# WP05: Electrical Engineer Agent & Workflow

## Objective
Implement the specialized Electrical Engineer sub-agent and integrate it into the engineering LangGraph workflow.

## Context
The Engineering workflow currently has a Mechanical Engineer (who handles geometry and assembly). We are adding an Electrical Engineer node that takes the mechanical design and adds the electronics.

## Detailed Guidance

### T019: Create `controller/agent/nodes/electronics_engineer.py`
**Purpose**: Define the LangGraph node.

**Steps**:
1. Create a new node function `electronics_engineer_node`.
2. Implement the state transitions (Receive Mech Assembly -> Design Electronics -> Send for Review).

### T020: Define Electrical Engineer system prompt
**Purpose**: Instruct the agent on how to do electrical engineering.

**Steps**:
1. Add `electronics_engineer` section to `config/prompts.yaml`.
2. Include instructions for:
    - Designing safe circuits (parallel branches, correct PSU sizing).
    - Routing wires away from moving mechanisms.
    - Using the `validate_circuit` tool before finishing.

### T021: Implement tools for Electrical Engineer
**Purpose**: Provide the agent with the necessary capabilities.

**Steps**:
1. Register `validate_circuit`, `route_wire`, and `search_electrical_cots` as tools.
2. Ensure they are accessible to the `electronics_engineer` node.

### T022: Update `controller/agent/graph.py` (Handover)
**Purpose**: Integrate the new node into the main graph.

**Steps**:
1. Update `define_graph` to include the `electronics_engineer` node.
2. Add an edge from `mechanical_engineer` to `electronics_engineer`.
3. Add an edge from `electronics_engineer` to `reviewer`.

### T023: Implement `todo.md` checklist generation
**Purpose**: Guide the agent's internal progress.

**Steps**:
1. Update the planner to generate electrical subtasks in the shared `todo.md`.

### T024: Add integration tests
**Purpose**: Verify the full Mech+Elec loop.

**Steps**:
1. Prompt the agent to "Build a powered conveyor."
2. Verify it calls both Mech and Elec tools.
3. Check that the final output contains both geometry and electronics.

## Definition of Done
- Electrical Engineer node correctly receives mechanical context.
- Agent successfully designs and validates a circuit in the graph.
- Full graph execution (Mech -> Elec -> Reviewer) works for powered parts.

## Risks
- Loops between Mech and Elec agents if wiring always fails clearance.
- Token overhead of two engineers communicating.

## Activity Log

- 2026-02-15T10:17:49Z – gemini-cli – shell_pid=121941 – lane=doing – Assigned agent via workflow command
