---
name: electromechanics-syntax
description: Local authoring syntax for motorized moving parts and future electromechanics sections. Use when editing `assembly_definition.yaml`, `final_assembly`, `moving_parts`, `AssemblyPartConfig.control`, or related motion syntax in planner and solution handoffs.
---

# Electromechanics Syntax

## Overview

This skill covers the solution-facing syntax for motorized moving parts. It is
the contract layer between authored handoffs and runtime controller mapping.
Use it before editing `assembly_definition.yaml` or solution scripts that
declare moving parts, motor controls, or future explicit electronics.

For explicit circuit, power, or wire-routing work, still load
`electronics-engineering`.

## Read First

- `specs/architecture/agents/handover-contracts.md`
- `shared/models/schemas.py`
- `../cots-parts/SKILL.md`
- `../electronics-engineering/SKILL.md` when the handoff includes an explicit
  `electronics` section or circuit-validation task

## Core Rules

1. `AssemblyDefinition.final_assembly` is the authored source of truth.
   `moving_parts` is derived from it.
2. Use `PartConfig.name` as the stable moving-part identifier.
3. Use `AssemblyPartConfig.dofs` to declare motion. `dofs: []` means static.
4. Use `AssemblyPartConfig.control` only for motorized parts. The canonical
   control modes are `CONSTANT`, `SINUSOIDAL`, and `ON_OFF`. The parser accepts
   case-insensitive spellings, but new handoffs should use uppercase canonical
   values.
5. Keep `speed` populated for motorized parts. Use `frequency` only when the
   selected control mode needs it.
6. Keep COTS motor identity explicit in `assembly_definition.yaml.cots_parts`.
   Motion syntax does not replace provenance.
7. Motors alone do not imply an `electronics` section. Add power or wiring only
   when the handoff explicitly requires it.
8. Do not invent backend-specific motor fields, hidden controller names, or
   alternate syntax layers.

## Authoring Pattern

1. Identify every moving part in the approved handoff.
2. Bind the part name, DOFs, and control mode to the same stable label.
3. If the part is COTS-backed, keep the catalog `part_id` aligned with the
   motion contract.
4. If the handoff includes explicit power or wire routing, switch to
   `electronics-engineering` for that part of the contract.
5. Keep the solution script and assembly names synchronized so the runtime can
   build controller maps without guessing.

## Example

```yaml
cots_parts:
  - part_id: ServoMotor_DS3218
    manufacturer: Generic
    unit_cost_usd: 18.0
    quantity: 1
    source: catalog

final_assembly:
  - name: drive_motor
    config:
      dofs: ["rotate_z"]
      control:
        mode: CONSTANT
        speed: 1.0
  - name: frame
    config:
      dofs: []
```

## Future Expansion

- This skill will absorb explicit electronics syntax later.
- Until that lands, explicit circuit and wiring work should still use
  `electronics-engineering`.
- For geometry and COTS motor bodies, keep using
  `build123d-cad-drafting-skill` and `cots-parts`.
