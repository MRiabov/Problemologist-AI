---
work_package_id: WP04
title: Operations - Backups
lane: planned
dependencies: []
subtasks: [T011, T014, T012, T013]
---

# Work Package: Operations - Backups

## Objective

Implement automated backup procedures for the core data stores: Postgres (Controller DB) and S3 (Asset Store). This ensures disaster recovery capability.

## Context

We need a daily backup of our state. Since we are using Railway/Podman, we will trigger backups via an API endpoint (callable by an external Cron job or Temporal workflow) that dumps the database and syncs the S3 bucket to a backup location (or creates a snapshot archive).

## Implementation Guide

### Subtask T011: Implement backup logic

**Purpose**: Scripts/Functions to perform the actual backup.

**Steps**:

1. Create `src/ops/backup.py`.
2. Implement `backup_postgres(db_url: str, output_path: str)`:
    - Use `subprocess` to call `pg_dump` (ensure `pg_dump` is installed in the container image).
    - Compress the output (gzip).
    - Upload to S3 "backup" bucket or folder.
3. Implement `backup_s3_files(source_bucket: str, backup_dest: str)`:
    - Use `aws s3 sync` CLI or `boto3` equivalent code to copy new files to a cold storage bucket (or a `backups/YYYY-MM-DD/` folder).

**Files**:

- `src/ops/backup.py` (New)

---

### Subtask T014: Implement Temporal Backup Workflow

**Purpose**: Wrap backup logic in a durable workflow to handle timeouts and retries.

**Steps**:

1. Create `src/ops/workflows.py`.
2. Define `BackupWorkflow` class with `@workflow.defn`.
3. Define `run_backup` activity with `@activity.defn` (wrapping functions from T011).
4. Configure retry policies (e.g., retry on S3 network errors).

**Files**:

- `src/ops/workflows.py` (New)

---

### Subtask T012: Create `POST /ops/backup` endpoint

**Purpose**: Trigger the backup workflow via HTTP.

**Steps**:

1. Add route `POST /ops/backup` to the controller API.
2. Protect it with a secret header.
3. In the handler, use `temporal_client.start_workflow(BackupWorkflow, ...)` or `execute_workflow`.
4. Return 202 Accepted with the Workflow ID.

**Files**:

- `src/api/ops.py` (Update/New)

---

### Subtask T013: Integration test for backup workflow

**Purpose**: Verify the backup actually produces files.

**Steps**:

1. Create `tests/ops/test_backup.py`.
2. Mock `subprocess.run` (for pg_dump) and `boto3` (for s3 upload).
3. Call the endpoint or function.
4. Assert that `pg_dump` was called with correct args.
5. Assert that S3 upload was initiated.

**Files**:

- `tests/ops/test_backup.py` (New)

**Validation**:

- Run `pytest tests/ops/test_backup.py`.

## Definition of Done

- [ ] Backup logic implemented for both DB and Files.
- [ ] Temporal Workflow defined for backups.
- [ ] Secure API endpoint triggers the workflow.
- [ ] Integration tests verify the flow.
- [ ] `Dockerfile` (or `apt-get` steps) updated to include `postgresql-client` (for `pg_dump`) if not already present.

## Reviewer Guidance

- Ensure the backup process doesn't block the main event loop (use async or threadpool).
- Check security of the endpoint (must require auth).
