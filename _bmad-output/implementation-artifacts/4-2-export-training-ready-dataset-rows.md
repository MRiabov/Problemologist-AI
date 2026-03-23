# Story 4.2: Export Training-Ready Dataset Rows

Status: ready-for-dev

## Story

As a dataset operator, I want completed benchmark and solution episodes exported as training-ready dataset rows so that I can use them for supervised training or RL and later recover the source benchmark, solution, review artifacts, and lineage from persisted metadata alone.

## Acceptance Criteria

1. Given a completed benchmark or solution episode that passes schema, completeness, and lineage checks, when export runs, then the system writes a strict dataset-row archive with the source episode ID, benchmark ID, user session ID, episode type, revision hash, artifact hash, seed lineage fields, and latest-revision artifact references only.
2. Given a row missing required metadata or with invalid lineage, when export runs, then the row is excluded and no dataset-row archive is produced for it.
3. Given an exported benchmark or solution row, when I inspect the persisted export metadata later, then I can recover the source benchmark, the solution, the review artifacts, and the joinable session and episode identifiers without needing live worker state.
4. Given persisted benchmark-owned or engineer-owned assets exist, when export builds the row, then it reads the stored assets and metadata from the episode and benchmark persistence surfaces, not from live workspace files or the UI-normalized list projection.
5. Given render or manifest artifacts exist, when export runs, then it preserves the exact latest-revision media and manifest paths rather than substituting stale prior-revision references or text-only placeholders.
6. Given a completed exported row, when I open the archive contents, then I see the persisted artifact families for that episode type, including plans, TODOs, journals, implementation files, review YAMLs, and render manifests, rather than a partial UI projection.

## Tasks / Subtasks

- [ ] Define the dataset export schema and controller response models. (AC: 1, 3, 5)
  - [ ] Add strict Pydantic models for dataset-row metadata, artifact references, and export response payloads.
  - [ ] Carry `episode_type`, `benchmark_id`, `seed_id`, `seed_dataset`, `seed_match_method`, `generation_kind`, `parent_seed_id`, `is_integration_test`, `integration_test_id`, and joinable trace IDs (`simulation_run_id`, `cots_query_id`, `review_id`) plus source IDs and revision/artifact hashes from the persisted episode metadata.
  - [ ] Reuse the existing `EpisodeMetadata`, `TraceMetadata`, and asset reference patterns instead of inventing a second provenance model.
  - [ ] Add a typed persistence model and migration for dataset-row archive metadata and object-storage pointers rather than storing the row payload in a second relational blob.
- [ ] Implement export materialization from persisted benchmark and episode records. (AC: 1, 2, 3, 4, 5)
  - [ ] Add a dedicated controller export route/service in `controller/api/routes/datasets.py` and register it in `controller/api/main.py`.
  - [ ] Read raw persisted assets, traces, benchmark assets, and review manifests from DB and object storage.
  - [ ] Exclude any row that lacks the required schema/completeness/lineage fields instead of backfilling missing data.
  - [ ] Keep the exported row payload archive in object storage/S3 with DB metadata only if persistence is added; do not create a second relational payload store.
  - [ ] Do not source export content from `list_episodes()` normalization or any other UI convenience projection.
- [ ] Preserve benchmark-vs-solution bundle boundaries and latest-revision-only selection. (AC: 1, 3, 5)
  - [ ] Ensure benchmark rows retain benchmark-owned bundle references and reviewer manifests.
  - [ ] Ensure solution rows retain solution-owned bundle references, validation/simulation proof, and reviewer manifests.
  - [ ] Reject stale or cross-revision artifacts rather than silently falling back to older files.
  - [ ] Preserve the exact latest-revision media and manifest paths in the exported archive.
- [ ] Add integration coverage for export, exclusion, and round-trip inspectability. (AC: 1-5)
  - [ ] Seed the export path with the smallest real completed benchmark and solution episodes available, reusing the live completion flows from `test_handover.py` and `test_engineering_loop.py` instead of mocking episode state.
  - [ ] Add a dedicated `tests/integration/architecture_p1/test_dataset_export.py` slice, or extend `test_observability_extended.py` only if the new route intentionally lives there.
  - [ ] Assert the exported row can be traced back to the source benchmark, solution, review artifacts, and joinable IDs from persisted metadata alone.
  - [ ] Cover at least one fail-closed exclusion case for missing metadata or invalid lineage.

## Dev Notes

- Relevant architecture patterns and constraints:
  - Story 4.1 is the prerequisite bundle/persistence story. It establishes immutable latest-revision run/release bundles and inspectable preview assets; this story must consume that bundle and not create a second export-specific provenance system.
  - Dataset rows are S3/object-storage archives with DB metadata only. The controller should persist metadata and lineage pointers, not duplicate the full row payload in relational storage.
  - Export must distinguish benchmark-owned context from engineer-owned solution output. Keep `episode_type` or an equivalent row-kind field explicit so consumers do not blur the two bundle families.
  - Use strict schema validation at the export boundary; missing artifacts, stale manifests, invalid lineage, or schema drift must exclude the row fail-closed.
  - Do not use `list_episodes()` or any UI-normalized plan/journal projection as the source of truth. Export must read raw persisted artifacts and metadata from the DB/object-storage surfaces.
  - Story 4.4 owns seed coverage prioritization and corrupted/integration-test row filtering. This story should enforce only the hard validity and lineage gates.
  - If benchmark export needs proof that a row is the latest revision, use the persisted benchmark asset record and reviewer manifest, not live worker filesystem reads.
  - Prefer `EpisodeType`, `GenerationKind`, `SeedMatchMethod`, `EpisodeMetadata`, and `TraceMetadata` over ad hoc dicts when carrying provenance through models.
  - Do not use the `_normalize_plan_markdown()` read-model convenience in `controller/api/routes/episodes.py` as export source material.
  - Preserve the persisted artifact families already present on the episode, including plans, TODOs, journals, implementation files, review YAMLs, and render manifests, rather than reconstructing them from the UI or list endpoints.
- Source tree components to touch:
  - `controller/api/schemas.py`
  - `controller/api/main.py`
  - `controller/api/routes/datasets.py` (new)
  - `controller/persistence/models.py`
  - `controller/migrations/versions/*`
  - `shared/models/schemas.py`
  - `controller/api/routes/episodes.py`
  - `controller/api/routes/benchmark.py`
  - `tests/integration/architecture_p1/test_dataset_export.py`
  - `tests/integration/architecture_p1/test_handover.py`
  - `tests/integration/architecture_p1/test_engineering_loop.py`
  - `tests/integration/architecture_p1/test_reviewer_evidence.py`
  - `tests/integration/architecture_p1/test_observability_extended.py`
  - `tests/integration/evals_p2/test_evals_p2.py`
- Testing standards summary:
  - Use integration tests only. Drive the export over HTTP against the live controller and persist to real DB/S3-backed artifacts.
  - Assert on returned payloads, DB row metadata, object storage objects, and episode asset/trace records.
  - No unit-test-only coverage or direct import of persistence helpers.

### Project Structure Notes

- Add the export surface alongside the existing controller API modules rather than building a separate dataset subsystem.
- Keep row serialization narrow and typed. Do not store ad hoc JSON blobs with loose keys or inferred fallback content.
- Preserve the benchmark/solution bundle split. Benchmark rows must not be forced into engineer-shaped fields, and solution rows must not lose benchmark provenance.
- If later work needs replay reconstruction, that belongs to Story 4.3; do not backdoor replay assembly into export code.

### References

- [Source: \_bmad-output/planning-artifacts/epics.md, Epic 4: Dataset Export & Replay and Story 4.2: Export Training-Ready Dataset Rows]
- [Source: \_bmad-output/planning-artifacts/prd.md, Journey 3 and Journey 4, dataset readiness, reproducibility, and replay requirements]
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
- [Source: specs/architecture/observability.md, joinable IDs, lineage, terminal reasons, and dataset-export observability requirements]
- [Source: specs/dataset-generation.md, S3 archive generation and metadata-only DB storage contract]
- \[Source: controller/api/schemas.py, `EpisodeResponse`, `EpisodeListItem`, `AssetResponse`, and response model patterns\]
- [Source: controller/api/main.py, controller router registration pattern]
- \[Source: controller/api/routes/episodes.py, persisted episode/assets read model and `_normalize_plan_markdown()` fallback to avoid for export\]
- [Source: controller/api/routes/benchmark.py, benchmark session read model and benchmark asset source surface]
- [Source: controller/persistence/models.py, episode, trace, asset, and benchmark asset persistence models]
- \[Source: shared/models/schemas.py, `EpisodeMetadata`, `TraceMetadata`, and lineage-bearing schema fields\]
- \[Source: shared/enums.py, `EpisodeType`, `GenerationKind`, and `SeedMatchMethod` provenance enums\]
- [Source: shared/workers/schema.py, review-manifest and validation-record schema patterns]
- [Source: shared/observability/schemas.py, persisted event model patterns and lineage fields]
- [Source: tests/integration/architecture_p1/test_handover.py, benchmark bundle persistence assertions]
- [Source: tests/integration/architecture_p1/test_engineering_loop.py, solution bundle persistence assertions]
- [Source: tests/integration/architecture_p1/test_reviewer_evidence.py, latest-revision review manifest and media linkage assertions]
- [Source: tests/integration/architecture_p1/test_observability_extended.py, cross-system persistence/correlation patterns]
- [Source: tests/integration/evals_p2/test_evals_p2.py, dataset-readiness completeness checks]
- [Source: specs/integration-tests.md, INT-050, INT-055, INT-058, and INT-073]

## Dev Agent Record

### Agent Model Used

TBD

### Debug Log References

### Completion Notes List

### File List
