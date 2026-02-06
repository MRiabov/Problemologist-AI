# Tasks: 008-observability-system

**Feature Directory**: `kitty-specs/008-observability-system`

## Status

- [ ] WP01: Foundation - Tracing & Persistence <!-- id: WP01 -->
- [ ] WP02: Storage System (S3) <!-- id: WP02 -->
- [ ] WP03: Validation & Contracts <!-- id: WP03 -->
- [ ] WP04: Operations - Backups <!-- id: WP04 -->

## Work Packages

### WP01: Foundation - Tracing & Persistence

**Setup the core observability infrastructure: LangFuse for tracing and Postgres for LangGraph persistence.**

- [x] T001: Implement `LangFuseCallbackHandler` and initialization logic in `src/observability/tracing.py`. <!-- id: T001 -->
- [x] T002: Implement `PostgresSaver` customization for LangGraph in `src/observability/persistence.py`. <!-- id: T002 -->
- [x] T003: Write unit tests for tracing and persistence configuration. <!-- id: T003 -->

**Implementation Notes**:

- Use `langfuse` SDK.
- Configure `PostgresSaver` to use the main controller database.
- Ensure tracing is async-compatible.

**Dependencies**: None
**Prompt Size**: ~300 lines

---

### WP02: Storage System (S3)

**Implement the S3 interface for handling artifact storage (videos, large files).**

- [x] T004: Create `S3Client` wrapper in `src/observability/storage.py` using `boto3`. <!-- id: T004 -->
- [ ] T005: Implement `upload_file`, `download_file`, `list_files` methods. <!-- id: T005 -->
- [ ] T006: Implement `get_presigned_url` method for secure access. <!-- id: T006 -->
- [ ] T007: Write unit tests for S3 wrapper using `moto` or mocks. <!-- id: T007 -->

**Implementation Notes**:

- Support S3-compatible backends (Railway, MinIO).
- Handle connection errors gracefully.

**Dependencies**: None (Parallelizable with WP01)
**Prompt Size**: ~400 lines

---

### WP03: Validation & Contracts

**Enforce strict data contracts and validation using Pydantic and Schemathesis.**

- [ ] T008: Define Pydantic models for `TraceEvent`, `AssetRecord` in `src/models/observability.py`. <!-- id: T008 -->
- [ ] T009: Verify and align `data-model.md` with implemented models. <!-- id: T009 -->
- [ ] T010: Set up `schemathesis` in `tests/contracts/test_observability.py` to fuzz endpoints. <!-- id: T010 -->

**Implementation Notes**:

- Ensure strict typing.
- Schemathesis should run against the OpenAPI spec generated from Fast(Lite)API or similar.

**Dependencies**: None (Parallelizable, but verifies other components)
**Prompt Size**: ~300 lines

---

### WP04: Operations - Backups

**Implement automated backup mechanisms for Database and S3 artifacts.**

- [ ] T011: Implement `src/ops/backup.py` with `backup_postgres()` and `backup_s3_files()` functions. <!-- id: T011 -->
- [ ] T012: Create `POST /ops/backup` endpoint in controller API to trigger the backup logic. <!-- id: T012 -->
- [ ] T013: Write integration test for backup workflow (checking artifacts created). <!-- id: T013 -->

**Implementation Notes**:

- Use `pg_dump` for Postgres.
- Sync S3 buckets to backup location or create zip.
- Endpoint should be secured.

**Dependencies**: WP01 (Database), WP02 (S3)
**Prompt Size**: ~350 lines
