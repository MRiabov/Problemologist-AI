---
title: Goal-Zone Overlap YAML Allowance Contract
status: migration
agents_affected:
  - benchmark_planner
  - benchmark_plan_reviewer
  - benchmark_coder
  - benchmark_reviewer
  - engineer_planner
  - engineer_plan_reviewer
  - engineer_coder
  - engineer_execution_reviewer
added_at: '2026-04-09T18:46:00Z'
---

# Goal-Zone Overlap YAML Allowance Contract

<!-- Migration tracker. Check items conservatively as implementation lands. -->

## Purpose

This migration removes markdown substring matching from the goal-zone overlap
gate and replaces it with a typed YAML contract in the planner drafting
section.

The target contract is:

1. overlap allowance is declared in YAML, not inferred from `plan.md`,
2. the drafting section remains the machine-readable home for the allowance,
3. the validator fails closed when a geometry body intersects the goal zone
   but no matching YAML allowance entry exists,
4. benchmark and engineering drafting contracts use the same typed shape,
5. `plan.md` remains human-readable context only and is not part of the
   authorization path.

This is a narrow contract change, not a drafting overhaul. The repo already
has a typed `drafting` section; this migration adds one more typed field so the
allowed overlap is machine-readable instead of prose-sniffed.

## Problem Statement

The current overlap gate treats markdown wording as the source of truth for
goal-zone overlap permissions.

1. `worker_heavy/utils/file_validation.py` inspects `plan.md` and looks for
   nearby `capture`, `occupy`, or `reference` wording before it allows a
   solid to overlap the goal zone.
2. That makes the validator depend on narrative phrasing instead of on the
   typed assembly contract that already exists in YAML.
3. The same behavior leaks into seeded benchmark rows, where overlap
   allowance is currently described in prose for human readability.
4. A prose gate is brittle: it can be satisfied by wording drift, and it can
   fail even when the geometry and the authored contract are otherwise
   consistent.

The correct contract is to store the overlap allowance next to the drafting
metadata that already describes the assembly geometry.

## Current-State Inventory

| Area | Current behavior | Why it must change |
| -- | -- | -- |
| `worker_heavy/utils/file_validation.py` | Uses `_plan_explicitly_allows_goal_zone_overlap()` to scan `plan.md` for proximity between goal-zone text and allowance words. | The gate should read a typed YAML field instead of doing markdown substring matching. |
| `shared/models/schemas.py` | `DraftingSheet` models `views`, `dimensions`, `callouts`, `notes`, and `layout`, but no dedicated overlap-allowance field. | The drafting schema needs a machine-readable place to store the allowed goal-zone overlap. |
| `specs/architecture/agents/engineering-planner-technical-drawings.md` | Says overlap is allowed when the plan explicitly states capture, occupy, or reference language. | The architecture should move that allowance to the YAML drafting contract. |
| `shared/assets/template_repos/benchmark_generator/benchmark_assembly_definition.yaml` | Provides a drafting section, but no typed overlap-allowance block. | New benchmark workspaces should start with the canonical field already present. |
| `shared/assets/template_repos/engineer/assembly_definition.yaml` | Provides a drafting section, but no typed overlap-allowance block. | The mirrored engineering contract should use the same typed shape. |
| `dataset/data/seed/artifacts/benchmark_coder/bc-012-drawing-full/benchmark_assembly_definition.yaml` | Describes the raised shelf in drafting prose, but the overlap allowance still depends on narrative wording. | The seeded overlap allowance should be explicit in YAML. |
| `dataset/data/seed/artifacts/benchmark_coder/bc-013-drawing-full/benchmark_assembly_definition.yaml` | Describes the catch bin in drafting prose, but the overlap allowance still depends on narrative wording. | The seeded overlap allowance should be explicit in YAML. |

## Proposed Target State

1. `DraftingSheet` gains a typed `goal_zone_overlap_intents` field.
2. The field is a strict list of zone/target pairs, not a freeform string or
   prose note.
3. Each entry names the target geometry and the goal zone it is allowed to
   overlap.
4. The validator reads only the YAML list when deciding whether goal zone
   overlap is allowed.
5. `plan.md` may still mention the overlap for human readers, but the
   validation gate does not parse or inspect markdown prose for
   authorization.
6. Missing, malformed, or absent YAML allowance entries fail closed when the
   drafted geometry overlaps the goal zone.
7. The same field exists in both benchmark and engineering drafting
   contracts so the mirrored graphs stay aligned.

Example shape:

```yaml
drafting:
  goal_zone_overlap_intents:
    - zone_name: goal_zone
      target: raised_goal_shelf
```

## Required Work

### 1. Add the typed overlap-allowance list

- Extend `shared/models/schemas.py` with a strict drafting submodel for
  goal-zone overlap allowance entries.
- Keep the field typed and finite; do not model it as a freeform string map
  or prose-driven note.
- Require each entry to name the goal zone and the target geometry that is
  allowed to overlap it.
- Keep the field inside the existing `drafting` contract so it remains part of
  the assembly YAML, not a separate metadata file.

### 2. Switch validation to YAML

- Update `worker_heavy/utils/file_validation.py` so goal-zone overlap
  allowance is read from the drafting YAML list.
- Remove the markdown substring gate from the overlap decision path.
- Keep forbid-zone checks unchanged and fail closed.
- Treat `plan.md` as explanatory context only.

### 3. Update the architecture wording

- Update `specs/architecture/agents/engineering-planner-technical-drawings.md`
  so the overlap allowance is described as a YAML drafting contract, not as a
  prose parser rule.
- Keep the human-readable plan free to explain the overlap, but do not make the
  prose the machine-readable authority.

### 4. Refresh templates and seeded examples

- Add the typed overlap-allowance list to the benchmark and engineering starter
  drafting templates.
- Populate `bc-012-drawing-full` with an explicit YAML allowance for the raised
  goal shelf.
- Populate `bc-013-drawing-full` with an explicit YAML allowance for the catch
  bin.
- Keep the seed prose consistent with the YAML, but do not rely on the prose
  for validation.

### 5. Add a negative regression

- Add one regression case where `plan.md` says the fixture may overlap the goal
  zone, but the YAML allowance entry is missing, and assert that validation fails
  closed.
- Add one regression case where the YAML allowance entry exists and the same overlap
  is accepted.

## Non-Goals

- Do not broaden this migration to forbid-zone allowance.
- Do not keep markdown parsing as a fallback.
- Do not add an open-ended notes field as a substitute for the typed list.
- Do not change the existing geometry overlap math beyond the source of the
  allowance decision.

## Sequencing

The safe order is:

1. Add the typed drafting field to the schema.
2. Move the overlap gate to the YAML field and remove the markdown heuristic.
3. Update the architecture wording to match the new authority.
4. Refresh the starter templates and the seeded benchmark rows.
5. Add the regression coverage that proves markdown wording no longer grants
   overlap permission by itself.

## Acceptance Criteria

1. Goal-zone overlap is allowed only when the drafting YAML declares the
   matching zone/target entry.
2. `worker_heavy/utils/file_validation.py` no longer parses `plan.md` to
   authorize overlap.
3. The benchmark and engineering drafting schemas both expose the same
   typed overlap-allowance list.
4. Seeded cases such as `bc-012` and `bc-013` express overlap allowance in YAML
   rather than relying on prose.
5. A markdown-only overlap claim fails closed.
