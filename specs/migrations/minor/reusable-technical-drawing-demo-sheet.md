---
title: Reusable Technical Drawing Demo Sheet
status: investigation
agents_affected:
  - benchmark_planner
  - benchmark_plan_reviewer
  - benchmark_coder
  - benchmark_reviewer
  - engineer_planner
  - engineer_plan_reviewer
  - engineer_coder
  - engineer_execution_reviewer
added_at: '2026-04-08T17:29:58Z'
---

# Reusable Technical Drawing Demo Sheet

<!-- Follow-on investigation for a reusable demo-grade drafting sheet. -->

## Purpose

This migration investigates the next step after
[Engineering Planner Technical Drawings Migration](engineering-planner-technical-drawings-migration.md):
turn the drafting layer into a reusable review sheet that is useful on every
drafting-enabled run, not just structurally valid on paper.

The target output is a demo-grade technical drawing package that can show:

1. a default orthographic trio,
2. optional exploded review presentation,
3. datum tags,
4. binding dimensions,
5. callouts,
6. section or detail views when needed, and
7. notes that explain what can and should be done with the model.

The contract remains planner-authored and read-only downstream. The goal is to
make the draft legible and repeatable, not to turn the planner into a release
CAD author.

## Problem Statement

The current drafting path proves the contract exists, but it does not yet make
the sheet pleasant or consistently useful for demos and review.

1. The renderer currently projects edges and emits vector sidecars, but it does
   not yet present a polished sheet with annotations and review semantics.
2. The technical-drawing scripts structurally construct `TechnicalDrawing`,
   but the seeded examples are mostly minimal stubs rather than reusable sheets.
3. The schema models views, dimensions, callouts, and notes, but it does not
   yet carry a first-class display/layout concept for exploded review
   presentation.
4. Minimal drafting mode can therefore satisfy validation without producing a
   sheet that helps a human understand the model quickly.
5. If each benchmark or engineering row invents its own presentation trick, the
   output drifts into one-off styling instead of a shared drafting contract.

## Current-State Inventory

| Area | Current behavior | Why it matters |
| -- | -- | -- |
| `shared/models/schemas.py` | Models `DraftingSheet`, `DraftingView`, `DraftingDimension`, `DraftingCallout`, and `DraftingNote`, but no explicit display-only explode/layout block. | The contract can describe intent, but not yet a reusable presentation strategy. |
| `worker_renderer/utils/technical_drawing.py` | Loads `assembly_definition.yaml.drafting`, projects edges, and writes PNG/SVG/DXF bundles. | It produces preview output, but only the bare linework is rendered today. |
| `shared/assets/template_repos/benchmark_generator/drafting/benchmark_plan_technical_drawing_script.py` | Calls `TechnicalDrawing(...)` and returns the geometry compound. | This is enough to satisfy a structural gate, not enough to produce a demo-grade sheet. |
| `shared/assets/template_repos/engineer/drafting/solution_plan_technical_drawing_script.py` | Mirrors the same minimal drafting pattern for the engineering graph. | The behavior is symmetric, but still skeletal. |
| `config/prompts.yaml` and prompt assembly | Tell drafting-enabled roles to use `render_technical_drawing()`. | The prompt gate exists, but the output contract is still too thin for a polished demo. |
| `specs/architecture/agents/engineering-planner-technical-drawings.md` | Requires the drafting layer and allows exploded/layout presentation in the technical-drawing companion. | The architecture already permits the next step; the runtime and templates just do not fully realize it yet. |

## Proposed Target State

1. The drafting layer keeps `DraftingSheet` and `DraftingView` as the source
   of truth, but it gains a typed presentation/layout submodel for
   display-only exploded or staggered review presentation.
2. The layout submodel does not change inventory, joints, motion, or target
   geometry. It only changes how the sheet is presented for review.
3. When a drafting-enabled run does not specify richer presentation, the
   runtime materializes a default orthographic trio: front, top, and side.
   This is the reusable starter sheet for the common case.
4. If the drafting package asks for exploded review presentation, the
   renderer offsets the presentation layer, not the authored evidence
   geometry.
5. Datum identifiers, binding dimensions, callouts, and notes are rendered as
   visible review annotations and remain tied to real authored targets.
6. Section and detail views remain part of the same reusable path and are used
   when the base views would otherwise hide the binding interface.
7. `render_technical_drawing()` produces a review bundle that is more legible
   than a raw edge projection while still preserving the vector sidecars and
   bundle identity contract.
8. Benchmark and engineering planner graphs share the same drafting
   vocabulary so the demo behavior is consistent across every drafting-enabled
   run.

## Required Work

### 1. Extend the drafting contract

- Add a typed presentation/layout block to the drafting schema.
- Make the block explicit enough to express exploded review layout without
  mutating the source geometry.
- Keep the block narrow and machine-checkable.
- Do not let the layout block introduce new parts, joints, motions, or
  inventory labels.
- Preserve the existing `datums`, `dimensions`, `callouts`, `notes`,
  `section_marker`, and `detail_target` semantics.

### 2. Render the full review sheet

- Update `worker_renderer/utils/technical_drawing.py` so it renders the
  review semantics, not just projected edges.
- Render datum markers, dimension leaders and text, callout labels, section
  markers, detail targets, and the default orthographic trio.
- Render exploded or staggered review presentation as display-only offsets
  applied to the sheet layer.
- Preserve the current PNG, SVG, and DXF sidecar bundle structure.
- Keep the renderer fail-closed when the drafting data is missing or malformed.

### 3. Make the starter templates useful

- Update the benchmark and engineer drafting templates so the common case
  yields a readable orthographic trio instead of a mostly empty stub.
- Keep the starter reusable across runs instead of tailoring it to one seeded
  benchmark.
- Preserve the rule that the evidence script remains the compact geometry
  companion and the technical-drawing script carries presentation-only layout
  when that is needed.

### 4. Refresh prompts and review expectations

- Update drafting prompts so agents understand that the sheet should be
  demo-useful, not merely structurally valid.
- Make the reviewer guidance reject technically valid but unreadable or
  non-actionable sheets.
- Keep the prompt policy aligned across benchmark and engineering planner
  paths.

### 5. Add integration coverage

- Add one narrow benchmark drafting case that proves the reusable trio and the
  presentation layer.
- Add one narrow engineering drafting case that proves the same behavior on
  the solution side.
- Add a mode-off case that proves the drafting contract stays prompt-gated.
- Assert the render bundle contents, not just that `TechnicalDrawing(...)` was
  constructed.

## Non-Goals

- No full GD&T.
- No release-grade manufacturing drawing set.
- No separate CAD authoring language.
- No automatic dimension extraction for arbitrary geometry.
- No change to `render_cad()`.
- No benchmark-specific bespoke illustration style.

## Sequencing

The safe order is:

1. Add the typed presentation/layout contract.
2. Teach the renderer to draw the shared review sheet.
3. Update the starter templates to provide a useful baseline.
4. Refresh drafting prompts and reviewer expectations.
5. Add integration coverage for benchmark and engineering runs.

## Acceptance Criteria

1. A drafting-enabled run with only minimal drafting data still produces a
   readable default orthographic trio.
2. A drafting-enabled run can request exploded review presentation without
   changing the underlying inventory or motion contract.
3. Datum tags, binding dimensions, and callouts appear in the preview bundle
   and are tied to real authored targets.
4. The benchmark and engineering drafting paths produce the same basic review
   behavior from the shared drafting contract.
5. The evidence script remains inventory-faithful and does not absorb layout-
   only presentation concerns.
6. The integration suite proves the happy path and the mode-off path.

## Open Questions

1. Should the layout block live on `DraftingView` or on `DraftingSheet`?
2. Should the orthographic trio be the fallback only, or the default starter
   even when the draft specifies a smaller view set?
3. Should the renderer treat exploded presentation as a named layout mode or as
   a generic display transform family?

## File-Level Change Set

The implementation should touch the smallest set of files that actually land
the reusable sheet contract:

- `shared/models/schemas.py`
- `worker_renderer/utils/technical_drawing.py`
- `shared/assets/template_repos/benchmark_generator/drafting/benchmark_plan_technical_drawing_script.py`
- `shared/assets/template_repos/engineer/drafting/solution_plan_technical_drawing_script.py`
- `shared/assets/template_repos/benchmark_generator/drafting/benchmark_plan_evidence_script.py`
- `shared/assets/template_repos/engineer/drafting/solution_plan_evidence_script.py`
- `config/prompts.yaml`
- `controller/agent/prompt_manager.py`
- `tests/integration/**`
- `specs/architecture/agents/engineering-planner-technical-drawings.md`
