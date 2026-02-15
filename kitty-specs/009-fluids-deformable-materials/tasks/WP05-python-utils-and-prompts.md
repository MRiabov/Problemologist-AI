---
work_package_id: WP05
title: Python Utils & Prompt Updates
lane: "for_review"
dependencies: []
base_branch: main
base_commit: b11638b77884f77afaa837eab3919085e67aece8
created_at: '2026-02-15T12:31:20.264728+00:00'
subtasks: [T018, T019, T020, T021]
shell_pid: "83837"
agent: "gemini"
---

# WP05: Python Utils & Prompt Updates

## Objective

Provide agents with the necessary Python utility functions to analyze physics results and update their internal reasoning to prioritize safety factors and fluid containment.

## Context

The agents currently only understand rigid-body failure. We need to teach them about stress concentrations and fluid handling using direct Python function calls, which are more reliable and faster for the agents to execute than external tool calls.

## Guidance

### T018: `get_stress_report` Python Utility

- Implement `get_stress_report(part_label: str) -> dict` in `worker/utils/validation.py` (or `simulation_utils.py`).
- Returns the `StressSummary` for a specific part from the last simulation.
- Include diagnostic advice (e.g. "Safety factor too low - add material").
- **Note**: This function is already partially implemented in `worker/utils/validation.py`. Ensure it is robust and documented for agent use.

### T019: `define_fluid` Python Utility

- Implement `define_fluid(...)` in `worker/utils/validation.py`.
- Function to add fluid emitters/volumes to the assembly definition (modifies `objectives.yaml` or similar).
- Input: location, volume, fluid properties.
- **Note**: This function is already partially implemented in `worker/utils/validation.py`.

### T020: `preview_stress` Heatmap Utility

- Implement `preview_stress(component, ...)` in `worker/utils/validation.py`.
- Generates and returns a list of paths to stress heatmap images.
- Allows the agent (via VLM) to "see" where the stress is high.
- **Note**: This function is already partially implemented in `worker/utils/validation.py`.

### T021: Agent Prompt Updates

- Update Engineer and Planner prompts:
  - Add "Stress Considerations" section to the planning phase.
  - Instruct the agent to import and use these specific Python functions from `worker.utils.validation`.
  - Instruct the agent to target safety factors between 1.5 and 5.0.
  - Explain how to use the new stress and fluid utilities.

## Definition of Done

- [ ] Agent can successfully import and call `get_stress_report` to analyze part failue.
- [ ] Agent can define fluids in the simulation environment using `define_fluid`.
- [ ] Prompts correctly guide the agent to avoid part breakage.
- [ ] Agent solution quality (SC-005, SC-006) meets success criteria.

## Risks

- Agents hallucinating arguments or import paths.
- Circular dependencies if utilities import heavy backend modules at top level (ensure lazy imports where necessary).

## Activity Log

- 2026-02-15T12:31:20Z – gemini – shell_pid=83837 – lane=doing – Assigned agent via workflow command
- 2026-02-15T19:04:28Z – gemini – shell_pid=83837 – lane=for_review – Implemented get_stress_report, define_fluid, preview_stress, and set_soft_mesh. Updated agent prompts to use these utilities for stress and fluid reasoning. Added unit tests.
