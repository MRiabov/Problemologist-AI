---
title: 'Rendering Refactor: Scratch Buckets and Backend Selection'
status: migration
agents_affected:
  - benchmark_coder
  - benchmark_reviewer
  - engineer_coder
  - engineer_execution_reviewer
added_at: '2026-04-04T18:08:23Z'
---

# Rendering Refactor: Scratch Buckets and Backend Selection

<!-- Migration tracker. No behavior change yet. -->

## Purpose

This migration finishes the render-contract cleanup by making `renders/tmp/`
the only writable scratch render tree, renaming the final handoff bucket to
`renders/final_solution_submission_renders/`, and removing the last runtime and
prompt assumptions that preview rendering is tied to a specific authoring
library.

The target state is defined in:

- [CAD and other infrastructure](../../architecture/CAD-and-other-infra.md)
- [Simulation and Rendering](../../architecture/simulation-and-rendering.md)
- [Agent artifacts and filesystem](../../architecture/agents/artifacts-and-filesystem.md)
- [Agent tools](../../architecture/agents/tools.md)
- [Agent handover contracts](../../architecture/agents/handover-contracts.md)
- [Agent roles](../../architecture/agents/roles.md)

This migration supersedes the older render-bucket note in
`role-scoped-render-buckets.md` and follows the renderer-shim cleanup in
`remove-worker-renderer-local-render-shim.md`.

## Problem Statement

The repo now has the desired architecture contract, but the runtime, prompt,
and collector surfaces still need to converge on it.

1. Manual preview generation still has to be separated from persistent handoff
   evidence so live inspection does not leak into published bundles.
2. The old bucket names (`engineer_renders` and `final_preview_renders`) still
   appear in runtime readers, workspace seeding, prompt text, and helper code.
3. Persistent render paths are still too writable from agent-facing surfaces
   in a few places, which breaks the read-only bundle contract.
4. The render pipeline still has a final-bundle composition bug: the final
   engineer submission renders can omit benchmark-owned context, which makes
   engineer-side seed renders miss benchmark parts or objective overlays.
5. Prompt and helper wording still implies preview rendering is a
   build123d-specific render backend instead of a renderer-worker contract
   whose backend is selected inside `worker-renderer`.
6. The RGB handoff payload-path overlay still lacks a dedicated render-policy
   toggle, so disabling it would require code changes instead of a config
   change under `render:` in `config/agents_config.yaml`.
7. The public preview helper still uses the older `motion_forecast` flag name
   in some helper and doc surfaces, even though the intended user-facing name
   is `payload_path: bool`.

## Current-State Inventory

| Area | Current behavior | Why it must change |
| -- | -- | -- |
| `worker_light/api/routes.py` and `worker_heavy/utils/preview.py` | Live preview requests still materialize render evidence through legacy bucket assumptions instead of treating `renders/tmp/` as the only scratch tree. | Manual preview should be ephemeral and stage-local, not published as a persistent handoff bundle. |
| `worker_heavy/utils/rendering.py` and `worker_heavy/api/routes.py` | Bucket selection and handoff rendering still know about the old role-scoped naming and the old preview wording. | The backend must write the renamed buckets and keep preview/backend selection opaque to callers. |
| `shared/rendering/renderer_client.py` and `worker_renderer/api/routes.py` | Preview materialization still needs to be kept aligned with the renderer-owned backend selection and the new bundle split. | The renderer worker remains the owner of image generation, but not of legacy bucket naming. |
| `shared/script_contracts.py`, `controller/agent/initialization.py`, `controller/agent/handover_constants.py`, `controller/agent/nodes/base.py` | Manifest readers, workspace seeding, and discovery paths still enumerate old preview bucket names. | New bundles must resolve through `engineer_plan_renders` and `final_solution_submission_renders`. |
| `worker_heavy/utils/handover.py`, `worker_heavy/activities/heavy_tasks.py`, `controller/api/routes/datasets.py`, `controller/api/tasks.py`, `controller/middleware/remote_fs.py` | Collectors and readers still treat the old bucket paths as first-class surfaces. | Handoff assembly must recurse the renamed persistent buckets and exclude `renders/tmp/` from promotion. |
| `config/agents_config.yaml` and `config/prompts.yaml` | Path permissions and prompt text still encode the old bucket names and still imply the old render wording in a few places. | The agent-facing contract must match the architecture docs exactly. |
| Render-related integration tests and seed fixtures | Existing coverage still validates the old path split and the old bundle names. | The test suite must prove the new scratch/persistent split and the final-bundle composition rule. |

## Proposed Target State

1. `preview(...)` and `preview_drawing(...)` write live inspection evidence
   into `renders/tmp/` only.
2. `renders/tmp/` is deleted at handoff and never enters the persisted bundle
   index or handoff payload as a durable artifact.
3. Persistent 24-view handoff bundles are published only under:
   - `renders/benchmark_renders/`
   - `renders/engineer_plan_renders/`
   - `renders/final_solution_submission_renders/`
4. Those persistent buckets are read-only to agent roles; only backend/runtime
   utilities write them.
5. The renderer worker remains the owner of preview image generation, but the
   contract is backend-selected inside `worker-renderer`, not a hardcoded
   build123d render backend.
6. Final solution submission bundles include benchmark-owned context plus the
   approved engineer solution so benchmark fixtures and objective overlays are
   present by default.
7. RGB handoff bundles display the payload-path overlay specified by the
   relevant benchmark or engineer motion contract by default, but the
   overlay can be disabled via `render.handoff_rgb_payload_path_overlay.enabled`
   in `config/agents_config.yaml`.
8. Existing readers may continue to resolve bundle-local render manifests
   during the transition, but the canonical bucket names and prompt text use
   the renamed paths above.

## Required Work

### 1. Rewrite preview materialization

- Update the live preview entrypoints so manual preview requests materialize
  into `renders/tmp/` only.
- Keep the renderer worker as the image-generation service, but remove any
  wording or branching that treats build123d as the renderer backend.
- Ensure the preview helper and the renderer routes keep the same manifest
  shape while changing where live scratch files are written.
- Rename the public preview overlay switch from `motion_forecast` to
  `payload_path` in the helper-facing contract, request plumbing, and prompt
  examples, while preserving the same overlay behavior.

### 2. Rename the persistent render buckets end-to-end

- Replace `engineer_renders` with `engineer_plan_renders` wherever runtime code
  resolves preview bundles, manifest paths, or workspace seed files.
- Replace `final_preview_renders` with
  `final_solution_submission_renders` wherever runtime code resolves final
  handoff bundles, manifest paths, or dataset/task collectors.
- Keep benchmark evidence under `renders/benchmark_renders/` unchanged.

### 3. Enforce the scratch-only rule

- Make `renders/tmp/` the only agent-writable render tree in
  `config/agents_config.yaml`.
- Remove write permissions to the persistent render buckets from every agent
  role.
- Ensure the handoff path purges or replaces `renders/tmp/` at stage
  transition so scratch renders do not become durable outputs by accident.

### 4. Fix final submission composition

- Update the final solution submission render composition so benchmark-owned
  fixtures, objective overlays, and the approved solution geometry are all
  present before the 24-view bundle is generated.
- Remove the bug that lets the final submission bundle omit benchmark parts by
  default.
- Add a render-policy toggle in `config/agents_config.yaml` for the RGB
  handoff payload-path overlay, default it to enabled, and wire the renderer
  path to respect it without changing the handoff bundle shape.

### 5. Update prompt-facing and helper-facing text

- Replace lingering references to the old bucket names in `config/prompts.yaml`
  and any runtime help text.
- Replace wording that implies preview rendering is a build123d-specific
  backend with wording that says `worker-renderer` uses its selected preview
  backend.
- Replace any public preview examples that still say `motion_forecast=True`
  with `payload_path=True`.

### 6. Update tests and seed fixtures

- Update integration tests to prove live preview goes to `renders/tmp/`,
  persistent bundles use the renamed directories, and the persistent buckets
  remain read-only to agents.
- Add or refresh coverage for the final solution submission bundle so it
  contains benchmark-owned context by default.
- Update any render seed fixtures or mock manifests that still encode the old
  bucket names.
- Add coverage for the render-policy toggle that disables the RGB handoff
  payload-path overlay while keeping the default enabled.

## Non-Goals

- Do not replace `worker-renderer`.
- Do not change simulation ownership or the simulation-video contract.
- Do not change the render-manifest schema unless a specific reader requires
  it.
- Do not flatten bucketed renders back into a single root-level `renders/`
  directory.
- Do not change `inspect_media(...)` semantics or the visual-inspection policy.
- Do not add new render modalities.

## Sequencing

The safe order is:

1. Update preview materialization and bucket naming in runtime code.
2. Update access control, manifest readers, and collectors.
3. Fix final solution submission bundle composition.
4. Update prompts, docs, and helper text.
5. Refresh integration tests and seed fixtures.

## Acceptance Criteria

1. A repo-wide search across runtime code, prompt files, and workspace seeding
   finds no live references to `engineer_renders` or `final_preview_renders`
   outside historical migration docs.
2. Manual preview requests materialize into `renders/tmp/` only.
3. Published persistent 24-view bundles appear only under
   `renders/benchmark_renders/`, `renders/engineer_plan_renders/`, and
   `renders/final_solution_submission_renders/`.
4. Agent roles cannot write the persistent render buckets.
5. Final solution submission renders include benchmark-owned context by
   default and no longer drop benchmark parts or objective overlays.
6. RGB handoff bundles display the payload-path overlay defined by the
   relevant benchmark or engineer motion contract by default, and the overlay
   can be disabled via `config/agents_config.yaml` without changing code.
7. Public preview helper and prompt surfaces use `payload_path` as the overlay
   toggle name; any remaining `motion_forecast` references are limited to
   planner-motion contracts and historical docs.
8. The narrow render-related integration slice passes through
   `./scripts/run_integration_tests.sh`.

## Migration Checklist

### Preview materialization

- [x] Move live preview helpers to `renders/tmp/`.
- [x] Remove any remaining build123d-specific render wording or branch checks
  from runtime helpers.

### Bucket renaming

- [x] Replace old engineer/final bucket path constants with the renamed
  `engineer_plan_renders` and `final_solution_submission_renders` paths.
- [x] Update manifest readers, workspace seed files, and collectors to use
  the renamed persistent buckets.

### Access control

- [x] Make persistent render buckets read-only for agents.
- [x] Keep `renders/tmp/` writable only during the active stage.
- [x] Purge or replace `renders/tmp/` at handoff so scratch renders do not
  survive as durable bundle content.

### Final bundle composition

- [ ] Seed final solution submission renders with benchmark-owned geometry and
  objective overlays.
- [ ] Verify the final submission bundle no longer omits benchmark parts by
  default.
- [ ] Make the RGB handoff bundle display the payload-path overlay defined by
  the relevant stage motion contract by default, and make the config toggle
  able to disable it without changing the bundle shape.

### Prompts and tests

- [x] Update prompt templates and runtime help text that still mention the old
  bucket names or the wrong render-backend wording.
- [x] Refresh render-related integration tests and fixtures for the renamed
  buckets and scratch-only preview path.

## File-Level Change Set

The implementation should touch the smallest set of files that actually
enforce the new contract:

- `worker_light/api/routes.py`
- `worker_heavy/utils/preview.py`
- `worker_heavy/utils/rendering.py`
- `worker_heavy/api/routes.py`
- `worker_renderer/api/routes.py`
- `shared/workers/schema.py`
- `shared/agents/config.py`
- `shared/rendering/renderer_client.py`
- `shared/utils/agent/__init__.py`
- `shared/script_contracts.py`
- `controller/agent/initialization.py`
- `controller/agent/handover_constants.py`
- `controller/agent/nodes/base.py`
- `controller/middleware/remote_fs.py`
- `controller/api/routes/datasets.py`
- `controller/api/tasks.py`
- `worker_heavy/utils/handover.py`
- `worker_heavy/activities/heavy_tasks.py`
- `config/agents_config.yaml`
- `config/prompts.yaml`
- render-related integration tests and render seed fixtures
