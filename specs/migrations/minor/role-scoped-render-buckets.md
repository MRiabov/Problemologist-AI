---
title: Role-Scoped Render Buckets
status: investigation
agents_affected:
  - benchmark_reviewer
  - engineer_coder
  - engineer_execution_reviewer
added_at: '2026-03-31T20:20:26Z'
---

# Role-Scoped Render Buckets

<!-- Investigation doc. No behavior change yet. -->

## Purpose

This document investigates a render-path refactor that keeps benchmark preview evidence,
engineer inspection previews, and final preview bundles in separate
workspace buckets while preserving the existing manifest and review contracts.

The architecture already names those buckets. This note records the runtime
plumbing and test follow-up required to keep them distinct and readable by the
right roles:

- `renders/benchmark_renders/`
- `renders/engineer_renders/`
- `renders/final_preview_renders/`

The benchmark execution reviewer (`benchmark_reviewer`) needs the benchmark
bucket because it is part of the review record. Engineer roles need the
benchmark bucket as read-only context in addition to their own preview buckets.

## Problem Statement

The path split is mostly in place, but the plumbing is still partly implicit:

1. Render bucket selection is still inferred from workspace shape in a few
   places instead of from an explicit stage contract.
2. Some collectors recurse `renders/**`, while at least one submission-artifact
   collector still scans only the top level of `renders/`, which can drop
   bucketed image files from the handoff payload.
3. Agent and reviewer prompts already mention the buckets, so runtime helpers
   and docs need to stay aligned with that wording.
4. If the split is not kept explicit, future renderer or handover refactors can
   collapse the buckets back into a generic `renders/` surface and reintroduce
   reviewer ambiguity.

## Current-State Inventory

| Area | Current behavior | Why it matters |
| -- | -- | -- |
| `worker_heavy/utils/rendering.py` | Chooses `benchmark_renders`, `engineer_renders`, or `final_preview_renders` by checking whether `assembly_definition.yaml` exists in the workspace. | The bucket names are correct, but the rule is still implicit and can drift when new entrypoints appear. |
| `worker_renderer/api/routes.py` | Materializes render blobs into the selected bucket path and writes the manifest in the same workspace. | This is the right persistence shape and should stay aligned with the role buckets. |
| `worker_heavy/api/routes.py` | Recurses through `renders/**` when collecting render blobs for several live HTTP paths. | This is the model the submission path should match. |
| `worker_heavy/utils/handover.py` | Validates render-manifest consistency against the rendered bundle paths. | It must continue to accept bucketed paths without flattening them. |
| `worker_heavy/activities/heavy_tasks.py` | Collects submission artifacts and must recurse through `renders/**` so bucketed files survive the returned payload. | Bucketed files under `renders/benchmark_renders/`, `renders/engineer_renders/`, and `renders/final_preview_renders/` remain attached to the handoff. |
| `config/prompts.yaml` | Already names the three render buckets in benchmark and engineer prompts. | The runtime path split should keep matching those prompt instructions instead of inventing a fourth naming scheme. |
| `tests/integration/mock_responses/_render_manifest.py` | Rebuilds render manifests from `renders/**` recursively. | This is the correct artifact-shape reference for bucket-aware collection. |

## Jobs To Be Done

1. Keep benchmark review evidence in `renders/benchmark_renders/` and make it
   visible to `benchmark_reviewer` as the canonical benchmark-review bucket.
2. Keep engineer single-view inspection previews in `renders/engineer_renders/`
   so `engineer_coder` can inspect its own part previews without confusing them
   with benchmark input evidence.
3. Keep final engineer preview bundles in `renders/final_preview_renders/`
   so `engineer_execution_reviewer` can review the final assembly bundle as a
   separate evidence surface.
4. Preserve benchmark-owned static renders as read-only input for engineer
   roles while still allowing those roles to inspect them.
5. Make every handoff or submission collector recurse bucket directories so
   nested render files and `render_manifest.json` survive the payload assembly
   step.
6. Keep `inspect_media(...)` as the only media-review path; the bucket split
   should not change visual-inspection policy or media attachment behavior.
7. Update any tests or narrative docs that still imply a single generic render
   surface instead of three role-scoped buckets.

## Proposed Target State

1. Benchmark preview writes explicit preview evidence to
   `renders/benchmark_renders/`.
2. Engineer single-part previews write to `renders/engineer_renders/`.
3. Final engineer preview writes to `renders/final_preview_renders/`.
4. Benchmark and engineer review paths can inspect benchmark renders without
   reclassifying those files as engineer-owned output.
5. Render-manifest generation remains one manifest per bundle, and the manifest
   points at the files in the same bucket that produced them.
6. The bucket paths are explicit enough that a future refactor does not need to
   infer role intent from incidental workspace files.

## Required Work

### 1. Keep bucket selection explicit in runtime helpers

- Keep `worker_heavy/utils/rendering.py` and `worker_renderer/api/routes.py`
  aligned on the same bucket names.
- Treat the current workspace-shape inference as the compatibility rule, not as
  the long-term contract.
- If a future entrypoint needs a different bucket, add an explicit stage signal
  instead of reusing the wrong folder name.

### 2. Fix bucket-aware artifact collection

- Update `worker_heavy/activities/heavy_tasks.py` so submission artifacts recurse
  through `renders/**` instead of scanning only the top level of `renders/`.
- Audit `worker_heavy/utils/handover.py` and any related collector so bucketed
  render files and `render_manifest.json` remain attached to the review payload.
- Keep the path normalization rules workspace-relative so the manifest stays
  stable across controller, worker, and replay paths.

### 3. Keep role prompts and docs aligned

- Verify `config/prompts.yaml` still names the bucket that matches each role's
  inspection context.
- Update narrative docs only if they still describe a generic render directory
  instead of the three bucketed surfaces.
- Keep the existing architecture docs as the canonical source for the bucket
  contract rather than duplicating the same rule in a fourth place.

### 4. Update tests and catalog rows

- Revisit render-related integration coverage so the bucket split is exercised
  through the live workflow, not just by file existence checks.
- The most likely rows to retune are `INT-188`, `INT-189`, `INT-204`,
  `INT-207`, `INT-208`, `INT-210`, and `INT-211`.
- Add or revise assertions so benchmark evidence, engineer preview evidence,
  and final preview evidence are checked in the correct bucket.

## Non-Goals

- Do not change the `RenderManifest` schema.
- Do not rename review decision or review comments files.
- Do not move rendering off `worker-renderer`.
- Do not flatten all bucketed renders back into the root `renders/` directory.
- Do not change `inspect_media(...)` semantics or the visual-inspection policy.

## Sequencing

This refactor is adjacent to the renderer-shim cleanup in
`remove-worker-renderer-local-render-shim.md`.

The safe order is:

1. Keep the renderer service path stable.
2. Make the artifact collectors bucket-aware.
3. Update prompts, docs, and integration assertions to match the bucket split.
4. Remove any implicit compatibility path only after the bucketed evidence flow
   is verified end-to-end.

## Acceptance Criteria

1. Benchmark review evidence is surfaced from `renders/benchmark_renders/`.
2. Engineer inspection previews are surfaced from `renders/engineer_renders/`.
3. Final preview bundles are surfaced from `renders/final_preview_renders/`.
4. `benchmark_reviewer` and `engineer_execution_reviewer` can inspect
   benchmark renders without the files being reclassified as engineer-owned
   output.
5. Submission and review collectors preserve nested render paths and
   `render_manifest.json` in the handoff payload.
6. Existing render schemas, `inspect_media(...)`, and visual-inspection policy
   remain unchanged.
7. The integration suite can prove the bucket split through the live workflow
   rather than through mocked file paths alone.

## File-Level Change Set

The implementation should touch the smallest set of files that actually enforce
the new contract:

- `worker_heavy/utils/rendering.py`
- `worker_renderer/api/routes.py`
- `worker_heavy/api/routes.py`
- `worker_heavy/activities/heavy_tasks.py`
- `worker_heavy/utils/handover.py`
- `config/prompts.yaml`
- `specs/architecture/agents/tools.md`
- `specs/architecture/agents/handover-contracts.md`
- `specs/integration-test-list.md`
- `tests/integration/**`
- optionally `docs/backend-reference.md` and `docs/architecture.md` if the
  narrative docs still describe a single generic render surface
