---
work_package_id: WP05
title: Agent Tools & Prompt Updates
lane: planned
dependencies: []
subtasks: [T018, T019, T020, T021]
---

# WP05: Agent Tools & Prompt Updates

## Objective
Provide agents with the necessary tools to analyze physics results and update their internal reasoning to prioritize safety factors and fluid containment.

## Context
The agents currently only understand rigid-body failure. We need to teach them about stress concentrations and fluid handling.

## Guidance

### T018: `get_stress_report` Tool
- Controller tool that returns the `StressSummary` for a specific part from the last simulation.
- Include diagnostic advice (e.g. "Safety factor too low - add material").

### T019: `define_fluid` Tool
- Agent tool to add fluid emitters/volumes to the assembly definition.
- Input: location, volume, fluid properties.

### T020: `preview_stress` Heatmap Tool
- Tool to generate and return a URL/path to a stress heatmap image.
- Allows the agent (via VLM) to "see" where the stress is high.

### T021: Agent Prompt Updates
- Update Engineer and Planner prompts:
  - Add "Stress Considerations" section to the planning phase.
  - Instruct the agent to target safety factors between 1.5 and 5.0.
  - Explain how to use the new stress and fluid tools.

## Definition of Done
- [ ] Agent can successfully call `get_stress_report` and use the data to modify CAD.
- [ ] Agent can define fluids in the simulation environment.
- [ ] Prompts correctly guide the agent to avoid part breakage.
- [ ] Agent solution quality (SC-005, SC-006) meets success criteria.

## Risks
- Agents ignoring stress data and continuing with "rigid-only" thinking.
- Tool call explosion (agents calling `get_stress_report` for every part).
