---
work_package_id: "WP04"
title: "Agent Integration & Facade"
lane: "planned"
dependencies: ["WP02", "WP03"]
subtasks: ["T015", "T016"]
---

# WP04: Agent Integration & Facade

## Goal

Expose the workbench functionality to the Agent via a unified, simple API.

## Context

Agents in the Worker container need a simple way to validate their designs. We provide a single function `validate_and_price`.

## Subtasks

### T015: Implement Facade Function

**Objective**: Dispatch logic for validation.
**Files**: `src/worker/utils/dfm.py`
**Instructions**:

1. Create `src/worker/utils/dfm.py`.
2. Import `CNCWorkbench`, `InjectionMoldingWorkbench`.
3. Import `ManufacturingMethod`.
4. Implement `validate_and_price(part: Part | Compound, quantity: int = 1) -> dict`:
   - Check `part.metadata["manufacturing_method"]`.
   - Instantiate correct Workbench (`cnc` or `im`).
   - Call `analyze(part, quantity)`.
   - Return `.model_dump()` of the result (Agent prefers dict/json usually, or strict object? Rule says Pydantic is preferred for exchange, but usually tool outputs specifically might need serialization. Let's return the Pydantic model directly if possible, or dict if it's the terminal output).
   - **Correction**: Return `WorkbenchResult` (Pydantic model) as per "Use Pydantic Models" rule.

### T016: Create Integration Verification

**Objective**: Ensure the facade works end-to-end.
**Files**: `tests/workbenches/test_facade.py`
**Instructions**:

1. Create `tests/workbenches/test_facade.py`.
2. Test calling `validate_and_price` with a CNC part. verify it calls CNC logic.
3. Test calling with IM part.
4. Test calling with missing metadata (Should raise clear error).

## Verification

- Run `pytest tests/workbenches/test_facade.py`.

## Definition of Done

- `validate_and_price` is callable.
- Dispatches correctly.
- Returns clear errors on misuse.
