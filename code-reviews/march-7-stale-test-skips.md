# Code Review: Stale Integration Test Skips

Date: March 7, 2026
Status: Review

## Overview

Identification of integration tests that are currently skipping implemented features due to stale "endpoint not found" logic or environment-specific routing issues. These skips are "genuinely invalid" as they prevent verification of features that are documented and implemented in the codebase.

## Stale Integration Skips

### 1. COTS Search Contract and Read-Only Behavior

- **File:** `tests/integration/architecture_p0/test_cots_reviewer.py`
- **Test Name:** `test_int_012_013_cots_search_contract_and_readonly`
- **Issue:** The test skips if `/cots/search` returns a 404, assuming the endpoint is not implemented.
- **Evidence of Implementation:** The endpoint is fully defined in `controller/api/routes/cots.py` and included in `controller/api/main.py`.
- **Recommendation:** Investigate why the test runner is receiving a 404. This may be due to the `CONTROLLER_URL` being misconfigured in the test environment or a missing route prefix in the test call.

### 2. Infrastructure Backup Operations

- **File:** `tests/integration/architecture_p1/test_infrastructure.py`
- **Test Name:** `test_operations_workflows_int_042`
- **Issue:** The test skips if `/ops/backup` returns a 404 with the message `"/ops/backup endpoint not found"`.
- **Evidence of Implementation:** The endpoint is implemented in `controller/api/ops.py` and is properly registered in the FastAPI app.
- **Recommendation:** Verify the `BACKUP_SECRET` and routing. The skip logic is currently masking a potential routing or deployment issue in the integration environment.

## Contextual Note

These skips differ from valid environment-based skips (e.g., skipping CUDA-dependent tests on CPU) because they target core API endpoints that are expected to be present in any standard deployment of the Controller service.
