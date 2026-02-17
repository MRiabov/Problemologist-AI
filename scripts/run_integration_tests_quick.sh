#!/bin/bash
set -e

# Problemologist Quick Integration Test Runner
# This script uses local FastAPI servers instead of Docker containers for app services
# to avoid long build times and heavy resource usage.

export IS_INTEGRATION_TEST=true
export LOG_LEVEL=${LOG_LEVEL:-INFO}

# Networking for local services (infra is still in Docker but exposed on host)
export POSTGRES_URL="postgresql+asyncpg://postgres:postgres@localhost:15432/postgres"
export TEMPORAL_URL="localhost:17233"
export S3_ENDPOINT="http://localhost:19000"
export S3_ACCESS_KEY=minioadmin
export S3_SECRET_KEY=minioadmin
export AWS_ACCESS_KEY_ID=minioadmin
export AWS_SECRET_ACCESS_KEY=minioadmin
export WORKER_URL="http://localhost:18001"
export ASSET_S3_BUCKET="problemologist"

# Ensure we are in project root
cd "$(dirname "$0")/.."

# Load .env if it exists to pick up API keys (OpenAI, etc.)
if [ -f .env ]; then
  echo "Loading environment from .env..."
  # Strip spaces around '=' and comments, then export
  # We use sed to:
  # 1. Remove lines starting with # or empty lines
  # 2. Remove spaces around '='
  # 3. Export each resulting line
  while IFS= read -r line || [ -n "$line" ]; do
    # 1. Remove leading/trailing whitespace
    line=$(echo "$line" | sed -e 's/^[[:space:]]*//' -e 's/[[:space:]]*$//')
    
    # 2. Skip comments and empty lines
    [[ "$line" =~ ^#.*$ ]] && continue
    [[ -z "$line" ]] && continue
    
    # 3. Handle inline comments (strip anything after the first # that isn't inside quotes)
    # This is a bit complex for bash, but a simple version is to strip after # 
    # if it's not the first character.
    if [[ "$line" == *#* ]]; then
      # Only strip if it's not a quoted value containing # (simplified)
      if [[ "$line" != *\"*#*\"* && "$line" != *\'*#*\'* ]]; then
        line="${line%%#*}"
      fi
    fi

    # 4. Remove spaces around the FIRST '='
    clean_line=$(echo "$line" | sed -E 's/[[:space:]]*=[[:space:]]*/=/')
    
    # 5. Export if it looks like a variable assignment
    if [[ "$clean_line" =~ ^[A-Za-z_][A-Za-z0-0_]*=.*$ ]]; then
      # Trim any trailing spaces again after stripping comment
      clean_line=$(echo "$clean_line" | sed -e 's/[[:space:]]*$//')
      export "$clean_line"
    fi
  done < .env
fi

echo "Spinning up infrastructure (Postgres, Temporal, Minio)..."
docker compose -f docker-compose.test.yaml up -d

echo "Waiting for infra to be ready..."
MAX_INFRA_RETRIES=15
INFRA_COUNT=0
# Wait for Postgres
until docker compose -f docker-compose.test.yaml exec postgres pg_isready -U postgres > /dev/null 2>&1 || [ $INFRA_COUNT -eq $MAX_INFRA_RETRIES ]; do
  echo "Waiting for Postgres... ($INFRA_COUNT/$MAX_INFRA_RETRIES)"
  sleep 2
  INFRA_COUNT=$((INFRA_COUNT + 1))
done

if [ $INFRA_COUNT -eq $MAX_INFRA_RETRIES ]; then
  echo "Postgres failed to start in time."
  exit 1
fi

echo "Running migrations..."
uv run alembic upgrade head

echo "Starting Application Servers (Controller, Worker, Temporal Worker)..."

# Ensure log directory exists
mkdir -p logs

# Start Worker (port 18001)
uv run uvicorn worker.app:app --host 0.0.0.0 --port 18001 > logs/worker.log 2>&1 &
WORKER_PID=$!
echo "Worker started (PID: $WORKER_PID)"

# Start Controller (port 18000)
uv run uvicorn controller.api.main:app --host 0.0.0.0 --port 18000 > logs/controller.log 2>&1 &
CONTROLLER_PID=$!
echo "Controller started (PID: $CONTROLLER_PID)"

# Start Temporal Worker
uv run python controller/temporal_worker.py > logs/temporal_worker.log 2>&1 &
TEMP_WORKER_PID=$!
echo "Temporal Worker started (PID: $TEMP_WORKER_PID)"

cleanup() {
  # Record the exit status of the test command
  EXIT_STATUS=$?
  echo ""
  echo "Cleaning up processes (Controller: $CONTROLLER_PID, Worker: $WORKER_PID, Temporal: $TEMP_WORKER_PID)..."
  kill $CONTROLLER_PID $WORKER_PID $TEMP_WORKER_PID 2>/dev/null || true
  
  echo "Bringing down infrastructure containers..."
  docker compose -f docker-compose.test.yaml down -v
  
  exit $EXIT_STATUS
}

# Trap signals for cleanup
trap cleanup SIGINT SIGTERM EXIT

echo "Waiting for servers to be healthy..."
MAX_HEALTH_RETRIES=30
HEALTH_COUNT=0
while [ $HEALTH_COUNT -lt $MAX_HEALTH_RETRIES ]; do
  if curl -s http://localhost:18000/health | grep -q "healthy"; then
    echo "Controller is healthy!"
    break
  fi
  echo "Waiting for controller... ($HEALTH_COUNT/$MAX_HEALTH_RETRIES)"
  sleep 2
  HEALTH_COUNT=$((HEALTH_COUNT + 1))
done

if [ $HEALTH_COUNT -eq $MAX_HEALTH_RETRIES ]; then
  echo "Timeout waiting for services to be healthy."
  echo "--- LAST 20 LINES OF CONTROLLER LOG ---"
  tail -n 20 logs/controller.log
  exit 1
fi

# Default marker to run
MARKER=${1:-"integration_p0 or integration_p1 or integration_p2"}

# We use uv run to execute pytest within the virtual environment. 
# We pass -n0 to ensure integration tests run sequentially (stateful infra),
# but we DO NOT use -p no:xdist because -n2 is in pyproject.toml addopts 
# and pytest will error if the plugin is disabled while the flag is present.
if uv run pytest -v -m "$MARKER" --maxfail=3 -s -n0; then
  echo "Integration tests PASSED!"
else
  echo "Integration tests FAILED!"
  # The trap will handle the exit status
fi
