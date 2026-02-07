---
work_package_id: WP05
title: Persistence & Integration
lane: "doing"
dependencies: [WP04]
base_branch: 005-benchmark-scenario-generator-WP04
base_commit: 945ec66510a2e3541b30e87040f26f0e831e12c0
created_at: '2026-02-07T08:55:48.149728+00:00'
subtasks: [T017, T018, T019, T020]
shell_pid: "292387"
agent: "gemini-cli"
---

# WP05: Persistence & Integration

**Goal**: Save the results to S3/DB and provide a CLI for easy usage.

## Subtasks

### T017: Asset Persistence

**Purpose**: Upload generated artifacts to the cloud.

**Instructions**:

1. Implement `src/generators/benchmark/storage.py`.
2. `save_asset(script: str, images: List[bytes], mjcf: str, metadata: dict) -> BenchmarkAsset`:
   - Upload `script.py` to S3 (bucket: `benchmarks-source`).
   - Upload MJCF to S3.
   - Upload Images (bundled zip) to S3.
   - Create `BenchmarkAsset` record in DB.
   - Return the object.

### T018: Session Checkpointing

**Purpose**: Update session status in DB during execution.

**Instructions**:

1. Update `graph.py` or create a `Checkpointer` callback.
2. When graph nodes complete, update `GenerationSession` row in DB with new status/logs.
3. Allows real-time monitoring of long-running generation tasks.

### T019: CLI Interface

**Purpose**: Developer tool to run this.

**Instructions**:

1. Create `src/cli/benchmark.py` (or integrated into main entry point).
2. Commands:
   - `python -m src.cli.benchmark generate "A stack of 3 blocks"`
   - `python -m src.cli.benchmark batch --file prompts.txt`
3. Output: Print Session ID and final Asset URL.

### T020: End-to-End Integration Test

**Purpose**: The final proof.

**Instructions**:

1. Create `tests/e2e/test_benchmark_generation.py`.
2. Mock S3 and DB (or use test containers).
3. Run a full session with a simple prompt.
4. assert that:
   - Graph completes.
   - Script is generated and valid.
   - Assets "uploaded" (mock call verified).
   - DB record created.

## Verification

- [ ] CLI command triggers the graph.
- [ ] Assets are correctly routed to storage mock.
- [ ] Full pipeline runs from Prompt -> Asset.

## Activity Log

- 2026-02-07T08:55:48Z – Gemini – shell_pid=285937 – lane=doing – Assigned agent via workflow command
- 2026-02-07T09:05:08Z – Gemini – shell_pid=285937 – lane=for_review – Ready for review: Implemented persistence and CLI
- 2026-02-07T09:06:57Z – gemini-cli – shell_pid=292387 – lane=doing – Started review via workflow command
