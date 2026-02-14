---
work_package_id: "WP05"
title: "Agent Tools & Reasoning"
lane: "planned"
dependencies: "[]"
subtasks: ["T016", "T017", "T018", "T019", "T020"]
---

# WP05: Agent Tools & Reasoning

## Objective
Equip agents with tools to define fluids, analyze stress, and optimize designs based on physical feedback. Update agent prompts to incorporate stress-aware planning and reviewing.

## Context
Physical simulation data is only useful if the agent can understand and act upon it. We need to expose stress reports and fluid definitions as tools and update the "brain" (prompts) to use them.

## Detailed Guidance

### T016: `define_fluid` tool
- Create a new tool in `worker/agent/tools.py`.
- Signature: `define_fluid(name: str, viscosity: float, density: float, color: tuple)`.
- This tool updates the internal simulation state/config for the upcoming simulation run.

### T017: `get_stress_report` tool
- Create a new tool in `worker/agent/tools.py`.
- Signature: `get_stress_report(part_label: str) -> StressSummary`.
- Retrieves the latest stress summary from the previous simulation result.
- Provides safety factor and utilization % to the agent.

### T018: `preview_stress` tool
- Implement a tool that renders a von Mises stress heatmap on a part.
- This likely calls a helper in the asset pipeline (see WP06) and returns the image path.

### T019: Update `simulate` tool
- Update the existing `simulate` tool to accept `fem_enabled` and `particle_budget` overrides.
- Ensure the tool returns the extended `SimulationResult` with stress and fluid data.

### T020: Update Agent Prompts
- Update `controller/prompts.py` (or relevant prompt files):
  - **Planner**: Must consider load paths and material strength in risk assessments.
  - **CAD Engineer**: Must call `get_stress_report` and iterate on design (add material for safety < 1.5, remove for safety > 5.0).
  - **Reviewer**: Must check stress utilization and fluid metric passes.

## Test Strategy
- **Tool Validation**: Verify each tool returns expected data structures.
- **Agent Loop**: Run a "too-weak-part" benchmark and verify the CAD agent identifies the high stress and proposes a thicker part.
- **Fluid Loop**: Run a fluid task and verify the agent uses `define_fluid`.

## Definition of Done
- [ ] `define_fluid`, `get_stress_report`, and `preview_stress` tools implemented.
- [ ] `simulate` tool returns FEM/Fluid metrics.
- [ ] Agent prompts updated with stress-aware logic.
- [ ] Agent successfully improves a failing load-bearing design.

## Risks
- Agent getting stuck in "over-engineering" loops (adding too much material).
- Prompt bloating causing context window issues.
- Agent misinterpreting stress units (Pa vs MPa).
