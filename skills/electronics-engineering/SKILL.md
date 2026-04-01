---
name: electronics-engineering
description: Electromechanical planning and runtime guidance for the current Problemologist electronics stack. Use only when a plan or implementation explicitly includes an `electronics` section, benchmark `electronics_requirements`, circuit validation, or electronics review.
---

# Electronics Engineering Skill

Use this skill for current repo behavior, not legacy prompt lore.
It is for electromechanical reasoning and handoff alignment, not for importing runtime internals into authored `solution_script.py`.

## Load The Right Reference

- `references/api_reference.md`
  Use when you need the current schema or helper surface.
- `references/circuit_patterns.md`
  Use when composing an `ElectronicsSection` or reviewing a simple circuit topology.
- `references/routing_best_practices.md`
  Use when routing 3D wires or reviewing clearance/tension risks.
- `../cots-parts/SKILL.md`
  Use when the electronics plan includes catalog-backed motors, relays, connectors, wires, or other off-the-shelf components that need part identity and provenance preserved.

## Core Rules

1. Treat `assembly_definition.yaml` as the source of truth for planner-authored electronics.
2. Validate the logical circuit before physics simulation.
   Shorts, open circuits, and overcurrent are pre-gate failures.
3. Keep logical connectivity and physical routing separate but consistent.
   A valid netlist is not enough if the routed wire path collides with geometry or tears under motion.
4. Use current schemas and helpers, not stale placeholder APIs.
5. Backward compatibility still exists:
   if there is no `electronics` section and no explicit circuit-validation task, do not load this skill just because the design has motors or wires. Do not invent electronics unless the task requires it.

## Practical Workflow

1. Read the `electronics` section or explicit circuit-validation intent.
2. Load the current API reference.
3. Define components and wires with explicit terminals.
4. Run circuit validation and power-budget checks.
5. If wires are physical, route them with waypoints and check clearance against the assembly.

## Contract Notes

- For authored CAD scripts, continue to use the canonical `utils.*` import surface unless the task is specifically editing repo internals.
- For repo/runtime work, the current implementation surface is centered on `worker_heavy/utils/electronics.py`, `shared/circuit_builder.py`, `shared/pyspice_utils.py`, `shared/wire_utils.py`, and `shared/models/schemas.py`.
- Do not treat those runtime/internal helper modules as part of the authored submission-script import contract. `solution_script.py` should stay on the canonical `utils.submission` / `utils.metadata` surface and use direct build123d geometry for any physical implementation details.
