---
work_package_id: WP07
title: Evals & Documentation
lane: planned
dependencies: []
subtasks: [T030, T031, T032, T033]
---

# WP07: Evals & Documentation

## Objective
Finalize the feature by validating it against benchmarks and providing comprehensive documentation.

## Context
We need to ensure the system is reliable across various electromechanical scenarios and that users know how to use it.

## Detailed Guidance

### T030: Generate electromechanical benchmarks
**Purpose**: Create a test suite for the new capability.

**Steps**:
1. Create 10+ new benchmark prompts in `evals/datasets/`.
2. Examples: "Fan with speed control", "Two-axis solar tracker", "Conveyor with emergency stop".

### T031: Implement `run_evals.py` extensions
**Purpose**: Measure electrical success.

**Steps**:
1. Add metrics: `electrical_validity_rate`, `wire_integrity_rate`, `power_efficiency_score`.
2. Generate reports comparing Mech-only vs Electromechanical performance.

### T032: Perform prompt tuning
**Purpose**: Optimize agent behavior.

**Steps**:
1. Run evals and analyze failures.
2. Refine the Electrical Engineer system prompt to address common mistakes (e.g., forgetting ground connections).

### T033: Update documentation
**Purpose**: Help users and developers.

**Steps**:
1. Update `docs/electromechanical_integration.md`.
2. Add a tutorial in `docs/tutorials/electronics.md`.

## Definition of Done
- 10+ benchmarks run and scored.
- Reports show >80% success rate on electrical validity.
- Documentation is complete and published.

## Risks
- Benchmarks being too difficult for the current LLM state.
- Evaluation metrics not accurately reflecting real-world engineering quality.
