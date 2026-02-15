---
work_package_id: WP01
title: Foundation & Schemas
lane: "doing"
dependencies: []
base_branch: main
base_commit: b0b8a776ea84e7a6a780c69c31bdcfaaa7ce8261
created_at: '2026-02-15T07:28:28.680431+00:00'
subtasks: [T001, T002, T003, T004]
shell_pid: "347260"
agent: "gemini"
---

# WP01: Foundation & Schemas

## Objective
Establish the data models and persistence layer required to support electromechanical integration in the assembly and benchmark definitions.

## Context
We need to extend the existing `assembly_definition.yaml` (formerly `preliminary_cost_calculation.yaml`) and `objectives.yaml` to include an `electronics` section. This section will store the circuit netlist, power supply specifications, and wiring paths. We also need to add electronic component categories to our COTS database.

## Detailed Guidance

### T001: Update `shared/models/schemas.py`
**Purpose**: Define the Pydantic models for the electronics section.

**Steps**:
1. Add `PowerSupplyConfig`: `type`, `voltage_dc`, `max_current_a`.
2. Add `ElectronicComponent`: `component_id`, `type` (motor, relay, etc.), `cots_part_id` (optional), `assembly_part_ref` (optional).
3. Add `WireConfig`: `wire_id`, `from_node`, `to_node`, `gauge_awg`, `length_mm`, `waypoints` (list of 3D coordinates).
4. Add `ElectronicsSection`: Contains `power_supply`, `components`, and `wiring`.
5. Integrate `ElectronicsSection` into `AssemblyDefinition` and `ObjectivesYaml`.

**Validation**:
- [ ] Models can be instantiated with valid YAML.
- [ ] Schema validation catches missing required fields.

### T002: Create `shared/enums.py` extensions
**Purpose**: Define enums for electrical failures and component types.

**Steps**:
1. Add `ElectricalFailureType`: `SHORT_CIRCUIT`, `OVERCURRENT`, `WIRE_TORN`, `OPEN_CIRCUIT`.
2. Add `ElectronicComponentType`: `MOTOR`, `POWER_SUPPLY`, `RELAY`, `SWITCH`.

### T003: Update `assembly_definition.yaml` validation
**Purpose**: Ensure the new electronics section is validated during file operations.

**Steps**:
1. Update `worker/utils/file_validation.py` to include `ElectronicsSection` in the validation logic.
2. Ensure `assembly_part_ref` correctly references existing parts in the assembly.

### T004: Add migration to `parts.db`
**Purpose**: Seed the COTS database with electrical categories.

**Steps**:
1. Create a migration or script to add categories: `power_supply`, `relay`, `connector`, `wire`.
2. Add a few sample parts to `config/manufacturing_config.yaml` or directly to the DB for testing.

## Definition of Done
- All Pydantic models implemented and tested.
- Enums added.
- `assembly_definition.yaml` validation passes with electronic data.
- `parts.db` contains new categories.

## Risks
- Breaking backward compatibility with existing `assembly_definition.yaml` files. Ensure the `electronics` section is optional.

## Activity Log

- 2026-02-15T07:28:29Z – gemini – shell_pid=339790 – lane=doing – Assigned agent via workflow command
- 2026-02-15T07:32:49Z – gemini – shell_pid=339790 – lane=for_review – Electronics data models, enums, cross-reference validation, and COTS database seeding implemented.
- 2026-02-15T07:36:18Z – gemini – shell_pid=347260 – lane=doing – Started review via workflow command
