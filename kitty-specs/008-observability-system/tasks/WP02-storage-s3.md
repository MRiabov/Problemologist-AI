---
work_package_id: "WP02"
title: "Storage System (S3)"
lane: "planned"
dependencies: []
subtasks: ["T004", "T005", "T006", "T007"]
---

# Work Package: Storage System (S3)

## Objective

Implement a robust S3 storage interface for managing large artifacts (videos, simulation logs, etc.). The system must support Railway's S3-compatible object storage (or MinIO for local dev/worker).

## Context

Agents produce heavy artifacts like videos and large log files that should not be stored in the Postgres database. These assets are uploaded to object storage (S3), and only their URLs/Keys are referenced in the database. This work package builds the client wrapper to handle these operations securely and reliability.

## Implementation Guide

### Subtask T004: Create `S3Client` wrapper

**Purpose**: Encapsulate `boto3` logic and configuration.

**Steps**:

1. Create `src/observability/storage.py`.
2. Define `S3Client` class.
3. Initialize `boto3.client('s3', ...)` in `__init__`.
4. Load config (Endpoint, Access Key, Secret Key, Bucket Name) from environment.

**Files**:

- `src/observability/storage.py` (New)

---

### Subtask T005: Implement file operations

**Purpose**: Provide simple methods for agents/workers to save and retrieve files.

**Steps**:

1. Add `upload_file(local_path: str, object_key: str) -> str` (returns public/internal URL if applicable, or just key).
2. Add `download_file(object_key: str, local_path: str)`.
3. Add `list_files(prefix: str) -> list[str]`.
4. Include error handling for `botocore.exceptions.ClientError`.
5. (Optional) Add async versions (`aupload_file`) if `aioboto3` is available/desired, otherwise run in executor. *Stick to sync `boto3` run in threadpool for now if async not strictly required by stack, or use `aioboto3` if consistent with `deepagents` style.*

**Implementation Note**:

- Ensure proper MIME type detection (using `mimetypes` module) during upload.

---

### Subtask T006: Implement `get_presigned_url`

**Purpose**: Allow secure, temporary access to private artifacts (e.g., for the frontend).

**Steps**:

1. Add `get_presigned_url(object_key: str, expiration: int = 3600) -> str`.
2. Use `s3_client.generate_presigned_url(...)`.

---

### Subtask T007: Write unit tests for S3 wrapper

**Purpose**: Verify storage logic without hitting real S3.

**Steps**:

1. Create `tests/observability/test_storage.py`.
2. Use `moto` library to mock S3 service.
3. Test:
    - Initialization with env vars.
    - Upload puts file in mock bucket.
    - Download retrieves file.
    - Presigned URL generation (check format).
    - Error handling (e.g., bucket not found).

**Files**:

- `tests/observability/test_storage.py` (New)

**Validation**:

- Run `pytest tests/observability/test_storage.py`.

## Definition of Done

- [ ] `src/observability/storage.py` implements `S3Client` with required methods.
- [ ] Configuration via environment variables works.
- [ ] Unit tests using `moto` pass.
- [ ] Type hints (Pydantic models or standard types) used for arguments.

## Reviewer Guidance

- Check for hardcoded region or endpoint defaults that might break on Railway.
- Verify that `moto` is a dev-dependency only.
