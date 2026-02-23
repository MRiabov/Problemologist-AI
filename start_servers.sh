#!/bin/bash
export IS_INTEGRATION_TEST=true
export LOG_LEVEL=INFO
export PYTHONUNBUFFERED=1
export POSTGRES_URL="postgresql+asyncpg://postgres:postgres@127.0.0.1:15432/postgres"
export TEMPORAL_URL="127.0.0.1:17233"
export S3_ENDPOINT="http://127.0.0.1:19000"
export S3_ACCESS_KEY=minioadmin
export S3_SECRET_KEY=minioadmin
export AWS_ACCESS_KEY_ID=minioadmin
export AWS_SECRET_ACCESS_KEY=minioadmin
export WORKER_URL="http://127.0.0.1:18001"
export WORKER_HEAVY_URL="http://127.0.0.1:18002"
export ASSET_S3_BUCKET="problemologist"
export WORKER_SESSIONS_DIR=$(mktemp -d -t pb-sessions-XXXXXX)

cd "$(dirname "$0")"

uv run uvicorn worker_light.app:app --host 0.0.0.0 --port 18001 > worker_light.log 2>&1 &
uv run uvicorn worker_heavy.app:app --host 0.0.0.0 --port 18002 > worker_heavy.log 2>&1 &
uv run uvicorn controller.api.main:app --host 0.0.0.0 --port 18000 > controller.log 2>&1 &
uv run python -m controller.temporal_worker > temporal_worker.log 2>&1 &

sleep 10
