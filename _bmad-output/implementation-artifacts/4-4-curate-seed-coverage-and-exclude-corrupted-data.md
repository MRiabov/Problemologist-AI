# Story 4.4: Curate Seed Coverage and Exclude Corrupted Data

Status: ready-for-dev

## Story

As a dataset operator, I want to manage coverage by seed and problem family and exclude known-corrupted rows so that exported datasets stay balanced, representative, reproducible, and free of integration-test pollution.

## Acceptance Criteria

1. Given repeated seed variants or underrepresented families, when export prioritization runs, then the system tracks coverage through `seed_id`, `seed_dataset`, `seed_match_method`, `generation_kind`, and `parent_seed_id`, and uses those persisted lineage fields to order work deterministically instead of relying on incidental iteration order.
1. Given integration-test runs or episodes inside known-corrupted windows, including the historical pre-2026-03-03 00:00 Europe/Dublin cutoff documented in `specs/dataset-generation.md`, when curation or export runs, then those rows are excluded fail-closed with stable machine-readable reason codes and no dataset row is produced for them.
1. Given multiple seeds for the same benchmark family, when I inspect the curated export manifest, then it includes family-level coverage counts, accepted-vs-rejected counts, and dropped-lineage provenance so I can reproduce why a row was kept or excluded.
1. Given the same seed inputs and persisted metadata, when curation runs twice without source changes, then the keep/drop decisions, family coverage counts, and output ordering are deterministic and do not depend on live workspace state, row materialization order, or UI-normalized read models.
1. Given a row is excluded, when I inspect the curation metadata later, then the source episode and seed identifiers plus the exclusion reason list are preserved in a typed manifest rather than being inferred from missing output files.

## Tasks / Subtasks

- [ ] Define typed curation and coverage-report models. (AC: 1, 3, 5)
  - [ ] Reuse the existing `EpisodeMetadata` lineage fields and the `GenerationKind` / `SeedMatchMethod` enums instead of inventing a second seed taxonomy.
  - [ ] Add a strict model for curation output that carries `bucket_counts`, `counts`, `dropped_lineage`, `rejected`, and coverage-by-family summaries.
  - [ ] Keep the schema fail-closed: unknown fields, ambiguous lineage, or malformed reason payloads must invalidate the curation output.
- [ ] Implement coverage-aware selection and exclusion in the dataset pipeline. (AC: 1-5)
  - [ ] Prioritize underrepresented seeds and problem families before saturated ones using persisted lineage metadata, not live output order or UI projections.
  - [ ] Exclude `is_integration_test=true` / `generation_kind=INTEGRATION_TEST` rows and known-corrupted windows before export materialization.
  - [ ] Preserve dropped-lineage provenance and stable reason ordering so reruns produce the same manifest.
  - [ ] Keep the filtering logic fail-closed when lineage is missing or too ambiguous to place a row in a family bucket.
- [ ] Extend integration coverage for curation, exclusion, and determinism. (AC: 1-5)
  - [ ] Add or refresh integration tests for underrepresented-family prioritization, repeated-seed dedupe, integration-test exclusion, historical corruption-cutoff exclusion, and deterministic reruns.
  - [ ] Assert against manifest output, persisted lineage metadata, and generated row counts rather than helper calls or in-memory heuristics.
  - [ ] Include at least one fail-closed case where malformed lineage or a corrupted row is rejected with an explicit reason list.

## Dev Notes

- Relevant architecture patterns and constraints:
  - Epic 4, Story 4.4 is the source of truth for seed coverage curation and corrupted-row filtering.
  - Story 4.2 already established that export must read raw persisted artifacts and metadata rather than normalized list projections. Apply the same rule here: use persisted lineage and episode records, not UI convenience views.
  - Story 4.3 already established that replay and diagnosis must come from persisted metadata and artifact evidence only. Keep curation decisions equally auditable from stored manifests and lineage fields.
  - The dataset-generation spec already mandates hard filtering of integration-test runs and known-corrupted data; the historical cutoff is pre-2026-03-03 00:00 Europe/Dublin.
  - Do not silently backfill, rewrite, or auto-heal excluded rows. Every drop must stay explainable through the manifest.
  - Seed-family balancing should be derived from stable episode metadata. Prefer the benchmark family key already persisted on the episode when it exists; otherwise fall back only to legacy dataset provenance fields.
  - Keep curation keyed by stable seed and episode identifiers. Do not guess from mutable workspace state or from `list_episodes()` / normalized read models.
  - If a row has insufficient lineage to place it in a family bucket, exclude it rather than guessing.
  - Preserve the existing manifest shape and naming style (`bucket_counts`, `counts`, `dropped_lineage`, `rejected`) so downstream consumers and historical comparisons remain stable.
  - If the batch pipeline needs a richer summary structure, add a typed model instead of widening raw dicts.

### Previous Story Intelligence

- Story 4.2 showed that dataset export must consume persisted raw artifacts rather than UI-normalized projections. This story should keep the same source-of-truth discipline for coverage selection.
- Story 4.3 showed that deterministic replay depends on explicit persisted reason codes and lineage. Use the same pattern here so a curator can later reconstruct why a row was dropped.

### Source tree components to touch

- `evals/logic/runner.py`
- `evals/logic/models.py`
- `evals/logic/workspace.py`
- `dataset/evals/run_evals.py`
- `dataset/evals/materialize_seed_workspace.py`
- `scripts/validate_eval_seed.py`
- `shared/models/schemas.py`
- `shared/enums.py`
- `controller/persistence/models.py` only if a new persisted curation summary is required
- `dataset/data/generated/*/manifest.json` or the generator that writes those manifests
- `tests/integration/evals_p2/test_evals_p2.py`
- `tests/integration/architecture_p0/test_int_071_to_073.py`
- `tests/integration/architecture_p0/test_codex_runner_mode.py`
- `tests/integration/architecture_p1/test_dataset_export.py` if the curation summary is surfaced through the export path

### Testing standards summary

- Use integration tests only; do not add unit-test-only coverage for this story.
- Assert against manifest output, persisted lineage metadata, and generated row counts.
- Keep exclusions fail-closed and deterministic.
- Prefer real batch-pipeline entrypoints or HTTP surfaces that exercise the live pipeline; do not test this through mocked helper calls.

### Project Structure Notes

- Keep seed curation in the batch/eval pipeline, not in a separate controller-side data store.
- Preserve stable manifest naming and output shape so downstream consumers can diff runs.
- Treat integration-test and corrupted-row exclusions as first-class filtering, not post-hoc cleanup.
- Use typed models for any new curation summary or report structure.

### References

- [Source: _bmad-output/planning-artifacts/epics.md, Epic 4: Dataset Export & Replay and Story 4.4: Curate Seed Coverage and Exclude Corrupted Data]
- [Source: _bmad-output/planning-artifacts/prd.md, Journey 3 and Journey 4, dataset readiness, reproducibility, and corruption filtering]
- [Source: specs/desired_architecture.md, architecture index for agents, evaluation gates, simulation, and observability contracts]
- [Source: specs/architecture/primary-system-objectives.md, product-level benchmark and training-dataset goals]
- [Source: specs/architecture/agents/overview.md, benchmark/engineer workflow split]
- [Source: specs/architecture/agents/roles.md, planner/reviewer responsibilities and handoff artifacts]
- [Source: specs/architecture/agents/handover-contracts.md, strict handoff and reviewer-contract rules]
- [Source: specs/architecture/agents/artifacts-and-filesystem.md, read-only mounts, latest-revision artifacts, and dataset-friendly filesystem contract]
- [Source: specs/architecture/agents/tools.md, planner/reviewer submission gates and validation utilities]
- [Source: specs/architecture/distributed-execution.md, controller/worker split and fail-closed heavy-path routing]
- [Source: specs/architecture/CAD-and-other-infra.md, benchmark/solution artifact boundaries and preview evidence assumptions]
- [Source: specs/architecture/evals-and-gates.md, fail-closed gating, dataset readiness, and schema/lineage expectations]
- [Source: specs/architecture/simulation-and-dod.md, latest-revision validation preview and reproducibility contract]
- [Source: specs/architecture/observability.md, joinable IDs, lineage, terminal reasons, seed tracking, and dataset-export observability requirements]
- [Source: specs/dataset-generation.md, S3 archive generation, metadata-only DB storage contract, and the historical corrupted-data cutoff]
- [Source: shared/models/schemas.py, `EpisodeMetadata`, `TraceMetadata`, and lineage-bearing schema fields]
- [Source: shared/enums.py, `GenerationKind` and `SeedMatchMethod` provenance enums]
- [Source: evals/logic/runner.py, seed lineage capture, hard-check aggregation, and manifest generation]
- [Source: evals/logic/workspace.py, seeded workspace materialization and entry-contract preflight]
- [Source: scripts/validate_eval_seed.py, seeded workspace validation and seed preflight]
- [Source: dataset/data/generated/workflow/v0.0.1/manifest.json, manifest shape and rejection provenance example]
- [Source: dataset/data/generated/component_seeded/v0.0.1/manifest.json, manifest shape and dedup/drop provenance example]
- [Source: tests/integration/evals_p2/test_evals_p2.py, dataset-readiness checks]
- [Source: tests/integration/architecture_p0/test_int_071_to_073.py, lineage observability linkage]
- [Source: tests/integration/architecture_p0/test_codex_runner_mode.py, seeded workspace materialization and contract checks]

## Dev Agent Record

### Agent Model Used

TBD

### Debug Log References

### Completion Notes List

### File List
