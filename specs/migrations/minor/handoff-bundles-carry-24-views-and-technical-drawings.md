---
title: Handoff Bundles Carry 24 Views and Technical Drawings
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
added_at: '2026-04-11T08:22:02Z'
---

# Handoff Bundles Carry 24 Views and Technical Drawings

<!-- Migration tracker. No behavior change yet. -->

## Purpose

This migration defines the next step after the render-bucket split: each
published stage handoff bundle must carry both the 24-view static preview set
and the technical drawing package for the same revision.

The target state applies to all three persistent render buckets:

1. `renders/benchmark_renders/`
2. `renders/engineer_plan_renders/`
3. `renders/final_solution_submission_renders/`

The intent is to make the published bundle self-sufficient for downstream
agents and human reviewers. They should be able to inspect the visual model
from all sides and the orthographic drawing package from the same stage-owned
bundle without reconstructing evidence from the scratch tree.

The current architecture already separates scratch preview evidence from
persistent handoff bundles. This migration keeps that separation intact while
making the persistent bundle contents stricter and more complete.

The target contract is described in:

- [CAD and other infrastructure](../../architecture/CAD-and-other-infra.md)
- [Simulation and Rendering](../../architecture/simulation-and-rendering.md)
- [Engineering Planner Technical Drawings](../../architecture/agents/engineering-planner-technical-drawings.md)
- [Agent tools](../../architecture/agents/tools.md)
- [Agent artifacts and filesystem](../../architecture/agents/artifacts-and-filesystem.md)
- [renders acceptance criteria](../../architecture/agents/agent-artifacts/renders_acceptance_criteria.md)
- [Engineering Planner](../../architecture/agents/roles-detailed/engineer-planner.md)
- [Benchmark Planner](../../architecture/agents/roles-detailed/benchmark-planner.md)

## Problem Statement

The repository already knows about static handoff bundles and it already knows
about technical drawing previews, but the contract is still split across two
surfaces:

1. `render_cad()` and `render_technical_drawing()` are the obvious public
   inspection helpers, but they are both scratch-oriented and write to
   `renders/current-episode/`.
2. The persistent stage buckets are currently described as 24-view handoff
   bundles, but the contract does not yet say that the published bundle also
   carries the technical drawing package.
3. Reviewers and downstream agents therefore have to infer whether a stage
   bundle contains the drawing companion or whether they must look elsewhere.
4. The final solution submission bucket,
   `renders/final_solution_submission_renders/`, must follow the same rule as
   the benchmark and engineer-plan buckets. It should not become a weaker or
   differently shaped artifact family.
5. If the static bundle path continues to depend on the manual scratch helpers,
   the repo will keep conflating ephemeral inspection with durable handoff
   evidence.

The migration closes that gap by making the published bundle explicitly
composite: 24 visual views plus the drawing package, all owned by the same
stage bucket.

## Current-State Inventory

| Area | Current behavior | Why it must change |
| -- | -- | -- |
| `specs/architecture/CAD-and-other-infra.md` | Names the persistent stage buckets and describes them as 24-view handoff bundles, while `render_cad(...)` and `render_technical_drawing(...)` remain scratch helpers. | The contract does not yet say that the published bundle also includes the technical drawing package. |
| `specs/architecture/simulation-and-rendering.md` | Separates preview evidence from persistent handoff bundles and keeps the scratch tree and the persistent buckets distinct. | The static handoff bundle needs a stronger content contract, not just a stronger location contract. |
| `specs/architecture/agents/tools.md` | Documents `render_cad(...)` and `render_technical_drawing(...)` as on-demand preview helpers that persist into `renders/current-episode/`. | The helper contract is correct for scratch evidence, but it does not define how the published bundle carries both modalities. |
| `specs/architecture/agents/engineering-planner-technical-drawings.md` | Defines the technical drawing package and the `render_technical_drawing()` usage gate. | The drawing contract stops at inspection and validation; it does not yet state that the persisted stage bundle must include the drawing package alongside the 24-view preview set. |
| `specs/architecture/agents/agent-artifacts/renders_acceptance_criteria.md` | Requires bundle-local manifests and sidecar consistency for render bundles. | The file family needs to state that the persistent bundle is composite, not just manifest-valid. |
| Role sheets and review sheets for benchmark and engineer planning/reviewing | Mention the relevant persistent buckets, but do not yet explain that the same bundle should expose both preview views and drawings. | Reviewers need to know that both artifact families belong in the stage bundle they are inspecting. |
| `renders/current-episode/` | Serves as the only scratch render tree for active-stage inspection. | Scratch evidence should remain ephemeral and should not be the only place where the drawing package exists. |

## Phased Migration Shape

This migration is a contract change plus a publication-path change.

1. Contract tranche: update the architecture docs so the persistent stage
   bundles are explicitly composite artifacts.
2. Publication tranche: update the bundle-generation path so the published
   bundle carries both the 24-view render set and the technical drawing set.
3. Review tranche: update the role sheets, prompt text, and artifact contracts
   so downstream agents know both evidence families are available in the same
   bundle.
4. Verification tranche: refresh the narrow render-related integration slice
   so the new bundle shape is exercised through the real workflow.

## Proposed Target State

1. `renders/benchmark_renders/`, `renders/engineer_plan_renders/`, and
   `renders/final_solution_submission_renders/` each publish an immutable
   stage handoff bundle for the latest revision.
2. Each published bundle contains both of the following evidence families for
   the same revision:
   - the 24-view static preview render set
   - the technical drawing package, including raster inspection images and any
     vector sidecars required by the drawing contract
3. `render_cad()` and `render_technical_drawing()` remain ephemeral
   inspection helpers and continue to write only to `renders/current-episode/`.
4. The persistent bundle publication path may reuse the same rendering
   primitives, but it must not depend on the scratch helpers as the public
   contract for durable bundle creation.
5. The bundle-local manifest for each persistent bundle describes the complete
   published package, not just the 24-view render subset.
6. Reviewers and downstream agents can inspect both the view set and the
   drawing package from the same stage-owned bundle.
7. The final solution submission bucket follows the same content rule as the
   benchmark and engineer-plan buckets.

## Required Work

### 1. Tighten the bundle contract

- Update the architecture docs so they explicitly say that a published stage
  bundle carries both the 24-view preview set and the technical drawing
  package.
- Update the render artifact contract so bundle-local manifests and sidecars
  describe the complete composite bundle.
- Keep the scratch-tree contract unchanged: `renders/current-episode/` stays
  the only writable render tree for active-stage inspection.

### 2. Separate scratch helpers from bundle publication

- Keep `render_cad()` and `render_technical_drawing()` as ephemeral manual
  inspection helpers.
- Ensure the persistent bundle generation path does not require calling those
  helpers as its public API.
- Make the durable publication path assemble the 24-view output and the
  technical drawing output into the same stage-owned bundle.
- Keep the final solution submission bucket on the same bundle shape as the
  benchmark and engineer-plan buckets.

### 3. Update review-facing guidance

- Update the benchmark and engineering role sheets so they state that both
  the 24-view render set and the drawing package are available in the
  published bundle.
- Update reviewer guidance so media inspection covers both evidence families
  when they exist.
- Keep `inspect_media(...)` as the only agent-facing path for visual review.

### 4. Refresh tests and fixtures

- Update the narrow render-related integration slice so it proves the
  published bundle contains both evidence families.
- Refresh any mock manifests or seeded bundles that still treat technical
  drawings as scratch-only evidence.
- Keep the existing render manifest and sidecar checks fail-closed.

## Non-Goals

- Do not rename `renders/benchmark_renders/`, `renders/engineer_plan_renders/`,
  or `renders/final_solution_submission_renders/`.
- Do not change `render_cad()` or `render_technical_drawing()` into durable
  bundle writers.
- Do not add a fourth persistent render bucket.
- Do not change simulation ownership or simulation-video provenance.
- Do not change the render-manifest schema unless a reader truly requires it.
- Do not change the technical drawing contract itself beyond how it is
  packaged into the published stage bundle.

## Sequencing

The safe order is:

1. Update the architecture and artifact docs so the bundle contract is
   explicit.
2. Update the bundle publication path so it assembles the composite stage
   bundle.
3. Update role guidance and prompt text so downstream consumers expect the
   same bundle shape.
4. Refresh integration coverage and seeded fixtures.

## Acceptance Criteria

1. The repo documents the persistent render buckets as composite stage
   bundles, not as 24-view-only bundles.
2. Each published bundle under
   `renders/benchmark_renders/`,
   `renders/engineer_plan_renders/`, and
   `renders/final_solution_submission_renders/` contains both the 24-view
   preview set and the technical drawing package for the same revision.
3. `render_cad()` and `render_technical_drawing()` still write only to
   `renders/current-episode/`.
4. Reviewers can inspect both evidence families from the published bundle
   through the existing media-inspection path.
5. The narrow render-related integration slice passes through
   `./scripts/run_integration_tests.sh`.

## Migration Checklist

### Bundle contract

- [ ] Update the architecture docs to describe persistent handoff bundles as
  composite artifacts that contain both the 24-view preview set and the
  technical drawing package.
- [ ] Update the render artifact contract so bundle-local manifests describe
  the complete published bundle.

### Publication path

- [ ] Update the persistent bundle generation path so it publishes both
  evidence families into the same stage-owned bucket.
- [ ] Keep `render_cad()` and `render_technical_drawing()` restricted to
  `renders/current-episode/`.
- [ ] Keep `renders/final_solution_submission_renders/` aligned with the same
  content rule as the benchmark and engineer-plan buckets.

### Review guidance

- [ ] Update benchmark and engineering role sheets so reviewers expect both
  the view set and the drawing package inside the persistent bundle.
- [ ] Update prompt or tool guidance if it still implies the drawing package
  lives only in the scratch tree.

### Tests and fixtures

- [ ] Refresh the narrow render-related integration slice to assert both
  evidence families exist in the published bundle.
- [ ] Update any seeded render fixtures or mock manifests that still assume
  technical drawings are scratch-only artifacts.

## File-Level Change Set

The implementation should touch the smallest set of files that actually
enforce the new contract:

- `specs/architecture/CAD-and-other-infra.md`
- `specs/architecture/simulation-and-rendering.md`
- `specs/architecture/agents/tools.md`
- `specs/architecture/agents/engineering-planner-technical-drawings.md`
- `specs/architecture/agents/agent-artifacts/renders_acceptance_criteria.md`
- `specs/architecture/agents/roles-detailed/benchmark-planner.md`
- `specs/architecture/agents/roles-detailed/engineer-planner.md`
- `specs/architecture/agents/roles-detailed/benchmark-plan-reviewer.md`
- `specs/architecture/agents/roles-detailed/engineer-plan-reviewer.md`
- `specs/architecture/agents/roles-detailed/benchmark-reviewer.md`
- `specs/architecture/agents/roles-detailed/benchmark-coder.md`
- `specs/architecture/agents/roles-detailed/engineer-coder.md`
- `specs/architecture/agents/roles-detailed/engineer-execution-reviewer.md`
- `worker_renderer/api/routes.py`
- `worker_renderer/utils/technical_drawing.py`
- `worker_heavy/utils/handover.py`
- `worker_heavy/activities/heavy_tasks.py`
- `shared/rendering/renderer_client.py`
- `config/prompts.yaml`
- `controller/agent/prompt_manager.py`
- `specs/integration-test-list.md`
- `tests/integration/**`
