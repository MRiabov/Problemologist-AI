#!/bin/bash
# scripts/env_up.sh
# Sets up the Problemologist-AI development environment using local infra and FastAPI servers.
# Use this for local debugging, optimization, or manual testing.

set -e

# Ensure we are in project root
cd "$(dirname "$0")/.."

# Stop any existing environment first to ensure a clean start
./scripts/env_down.sh

# Networking for local services (infra is still in Docker but exposed on host)
export IS_INTEGRATION_TEST=true
export LOG_LEVEL=${LOG_LEVEL:-INFO}

# DB points to the exposed port in docker-compose.test.yaml
export POSTGRES_URL="postgresql+asyncpg://postgres:postgres@127.0.0.1:15432/postgres"
export TEMPORAL_URL="127.0.0.1:17233"
export S3_ENDPOINT="http://127.0.0.1:19000"
export S3_ACCESS_KEY=minioadmin
export S3_SECRET_KEY=minioadmin
export AWS_ACCESS_KEY_ID=minioadmin
export AWS_SECRET_ACCESS_KEY=minioadmin
export WORKER_URL="http://127.0.0.1:18001"
export ASSET_S3_BUCKET="problemologist"

# Load .env if it exists to pick up API keys (OpenAI, etc.)
if [ -f .env ]; then
  echo "Loading environment from .env..."
  while IFS= read -r line || [ -n "$line" ]; do
    line=$(echo "$line" | sed -e 's/^[[:space:]]*//' -e 's/[[:space:]]*$//')
    [[ "$line" =~ ^#.*$ ]] && continue
    [[ -z "$line" ]] && continue
    if [[ "$line" == *#* ]]; then
      if [[ "$line" != *\"*#*\"* && "$line" != *\'*#*\'* ]]; then
        line="${line%%#*}"
      fi
    fi
    clean_line=$(echo "$line" | sed -E 's/[[:space:]]*=[[:space:]]*/=/')
    if [[ "$clean_line" =~ ^[A-Za-z_][A-Za-z0-0_]*=.*$ ]]; then
      clean_line=$(echo "$clean_line" | sed -e 's/[[:space:]]*$//')
      export "$clean_line"
    fi
  done < .env
fi

# Ensure Docker is working correctly (fix for sandboxed environments / Docker-in-Docker).
# DO NOT REMOVE: This is required for agent execution environments where overlay2 fails.
bash scripts/ensure_docker_vfs.sh

echo "Spinning up infrastructure (Postgres, Temporal, Minio)..."
docker compose -f docker-compose.test.yaml up -d

echo "Waiting for infra to be ready..."
MAX_INFRA_RETRIES=60

# Wait for Postgres
INFRA_COUNT=0
until docker compose -f docker-compose.test.yaml exec postgres pg_isready -U postgres > /dev/null 2>&1 || [ $INFRA_COUNT -eq $MAX_INFRA_RETRIES ]; do
  echo "Waiting for Postgres... ($INFRA_COUNT/$MAX_INFRA_RETRIES)"
  sleep 2
  INFRA_COUNT=$((INFRA_COUNT + 1))
done

# Wait for Minio
INFRA_COUNT=0
until curl -s http://127.0.0.1:19000/minio/health/live > /dev/null 2>&1 || [ $INFRA_COUNT -eq $MAX_INFRA_RETRIES ]; do
  echo "Waiting for Minio... ($INFRA_COUNT/$MAX_INFRA_RETRIES)"
  sleep 2
  INFRA_COUNT=$((INFRA_COUNT + 1))
done

# Wait for Temporal
INFRA_COUNT=0
# Using python3 instead of nc for portability as nc is missing in some environments
until python3 -c "import socket; socket.create_connection(('127.0.0.1', 17233), timeout=1)" > /dev/null 2>&1 || [ $INFRA_COUNT -eq $MAX_INFRA_RETRIES ]; do
  echo "Waiting for Temporal... ($INFRA_COUNT/$MAX_INFRA_RETRIES)"
  sleep 2
  INFRA_COUNT=$((INFRA_COUNT + 1))
done

echo "Running migrations..."
uv run alembic upgrade head

echo "Starting Application Servers..."

# Ensure log directory exists and manage log history
LOG_DIR="logs/integration_tests"
ARCHIVE_DIR="logs/archives"
mkdir -p "$LOG_DIR"
mkdir -p "$ARCHIVE_DIR"

# Archive previous logs if they exist
if [ -d "$LOG_DIR" ] && [ "$(ls -A "$LOG_DIR")" ]; then
  TIMESTAMP=$(date +%Y%m%d_%H%M%S)
  mv "$LOG_DIR" "$ARCHIVE_DIR/run_${TIMESTAMP}"
  mkdir -p "$LOG_DIR"
fi

# Clean up archives older than 24 hours
find "$ARCHIVE_DIR" -maxdepth 1 -name "run_*" -mmin +1440 -exec rm -rf {} + 2>/dev/null || true

# Start Worker (port 18001)
uv run fastapi run worker/app.py --host 0.0.0.0 --port 18001 > "$LOG_DIR/worker.log" 2>&1 &
WORKER_PID=$!
echo $WORKER_PID > logs/worker.pid
echo "Worker started (PID: $WORKER_PID)"

# Start Controller (port 18000)
uv run fastapi run controller/api/main.py --host 0.0.0.0 --port 18000 > "$LOG_DIR/controller.log" 2>&1 &
CONTROLLER_PID=$!
echo $CONTROLLER_PID > logs/controller.pid
echo "Controller started (PID: $CONTROLLER_PID)"

# Start Temporal Worker
export PYTHONPATH=$PYTHONPATH:.
uv run python -m controller.temporal_worker > "$LOG_DIR/temporal_worker.log" 2>&1 &
TEMP_WORKER_PID=$!
echo $TEMP_WORKER_PID > logs/temporal_worker.pid
echo "Temporal Worker started (PID: $TEMP_WORKER_PID)"

echo "Waiting for services to be healthy..."
sleep 5
if curl -s http://127.0.0.1:18001/health | grep -q "healthy"; then
  echo "Worker is healthy!"
else
  echo "Worker health check failed (see logs/worker.log)"
fi

if curl -s http://127.0.0.1:18000/health | grep -q "healthy"; then
  echo "Controller is healthy!"
else
  echo "Controller health check failed (see logs/controller.log)"
fi

echo "Environment is UP. Use 'scripts/env_down.sh' to stop."
