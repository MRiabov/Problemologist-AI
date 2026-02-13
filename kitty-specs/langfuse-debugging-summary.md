# Langfuse v3 Debugging Summary

## 1. Problem Description

Traces are not appearing in the Langfuse UI.
The `langfuse-web` container frequently reports: `Failed to instantiate otel ingestion queue`.
The `controller` container reports: `Failed to export span batch code: 500, reason: {"message":"Internal Server Error","error":"An unknown error occurred"}`.

## 2. Investigated Components

### Environment Variables (.env)

- **Status:** Verified and Cleaned.
- **Findings:** Initially contained quotes and spaces (e.g., `LANGFUSE_SECRET_KEY = "..."`). These were removed to ensure standard parsing.
- **Keys:** `LANGFUSE_PUBLIC_KEY`, `LANGFUSE_SECRET_KEY`, and `LANGFUSE_HOST` are correctly set.

### Networking

- **Status:** Verified.
- **Findings:** `langfuse-web` can ping `redis` and `clickhouse`.
- **Port Mapping:** Port 13000 on host maps to 3000 in `langfuse-web`. Port 9000 is used for MinIO API, 19001 for MinIO Console.

### Clickhouse

- **Status:** Healthy but unstable.
- **Findings:**
  - Clickhouse server is running and healthy.
  - Tables (`traces`, `observations`, etc.) exist.
  - Port conflict warnings were found in logs (`Address already in use: 0.0.0.0:9009`), suggesting multiple startup attempts or leftover processes.
  - **Read-only issue:** Clickhouse logs showed tables sometimes being in `readonly mode` due to Zookeeper connection failures.
- **Fixes applied:**
  - Added explicit credentials (`default:langfuse`) to `CLICKHOUSE_URL` and `CLICKHOUSE_MIGRATION_URL`.
  - Restored `CLICKHOUSE_USER`, `CLICKHOUSE_PASSWORD`, and `CLICKHOUSE_DB` env vars which are required for migrations.

### Redis

- **Status:** Healthy.
- **Findings:** Running with password protection (`langfuse`). Authentication verified via `redis-cli`.

### SDK / Code (Controller)

- **Status:** Needs update to v3 paradigms.
- **Findings:**
  - Using `langfuse.decorators.observe` is the recommended v3 approach.
  - The `SafeCallbackHandler` might be hiding errors that could help debugging.
  - `DatabaseCallbackHandler` is currently responsible for dual-writing to local DB and forwarding to Langfuse.

## 3. Attempted Fixes

1. **Environment Sanitization:** Removed quotes/spaces from `.env`.
2. **Langfuse Config Tuning:**
   - Added `LANGFUSE_DIRECT_INGESTION: "true"`.
   - Added `LANGFUSE_ENABLE_OTEL_INGESTION: "true"`.
   - Added `LANGFUSE_AUTO_MIGRATION: "true"`.
3. **Clickhouse Auth:** Explicitly provided credentials in URLs to ensure the ingestion queue can connect.
4. **Service Restarts:** Performed multiple restarts of the Langfuse stack (`web`, `worker`, `clickhouse`, `redis`).

## 4. Current Blockers / Hypotheses

1. **OTel Ingestion Queue:** Even with `LANGFUSE_DIRECT_INGESTION`, the web container still struggles to instantiate the OTel queue. This is the root of the 500 errors when the controller tries to push spans.
2. **Clickhouse/Zookeeper Stability:** Clickhouse reporting `readonly mode` periodically suggests that Zookeeper (used for Clickhouse coordination) might be hitting resource limits or networking flakes, causing Clickhouse to reject writes.
3. **SDK Version mismatch:** While `langfuse` SDK is v3.14.1, some code patterns in `controller` might still be following v2 logic (e.g. `langfuse.trace()`).

## 5. Next Steps Recommended

1. **Verify Clickhouse Permissions:** Ensure the `default` user has full permissions to create and write to all tables in the `default` database.
2. **Increase Timeouts:** Increase the timeout for the OTLP exporter in the Python SDK to handle potential network congestion.
3. **Isolate OTel:** Try sending traces via standard OTel exporter to a generic sink to see if the issue is in the Controller's exporter or Langfuse's receiver.
