#!/bin/bash
set -e

echo "Integration Tests Started at: $(date)"

# Problemologist Integration Test Runner
# This script uses local FastAPI servers instead of Docker containers for app services
# to avoid long build times and heavy resource usage.

export IS_INTEGRATION_TEST=true
export LOG_LEVEL=${LOG_LEVEL:-INFO}

# Headless rendering for MuJoCo
export MUJOCO_GL=osmesa
export PYOPENGL_PLATFORM=osmesa

# Networking for local services (infra is still in Docker but exposed on host)
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

# Shared sessions directory for local integration tests
export WORKER_SESSIONS_DIR=$(mktemp -d -t pb-sessions-XXXXXX)
echo "Shared sessions directory: $WORKER_SESSIONS_DIR"

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

# Ensure Docker is working correctly (fix for sandboxed environments / Docker-in-Docker).
# DO NOT REMOVE: This is required for agent execution environments where overlay2 fails.
bash scripts/ensure_docker_vfs.sh

# Ensure ngspice is installed for electronics validation
bash scripts/ensure_ngspice.sh

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
if [ $INFRA_COUNT -eq $MAX_INFRA_RETRIES ]; then
  echo "Postgres failed to start in time."
  exit 1
fi

# Wait for Minio
INFRA_COUNT=0
until curl -s http://127.0.0.1:19000/minio/health/live > /dev/null 2>&1 || [ $INFRA_COUNT -eq $MAX_INFRA_RETRIES ]; do
  echo "Waiting for Minio... ($INFRA_COUNT/$MAX_INFRA_RETRIES)"
  sleep 2
  INFRA_COUNT=$((INFRA_COUNT + 1))
done
if [ $INFRA_COUNT -eq $MAX_INFRA_RETRIES ]; then
  echo "Minio failed to start in time."
  exit 1
fi

# Wait for Temporal (port only, auto-setup might still be running migrations)
INFRA_COUNT=0
# Using python3 instead of nc for portability as nc is missing in some environments
until python3 -c "import socket; socket.create_connection(('127.0.0.1', 17233), timeout=1)" > /dev/null 2>&1 || [ $INFRA_COUNT -eq $MAX_INFRA_RETRIES ]; do
  echo "Waiting for Temporal... ($INFRA_COUNT/$MAX_INFRA_RETRIES)"
  sleep 2
  INFRA_COUNT=$((INFRA_COUNT + 1))
done
if [ $INFRA_COUNT -eq $MAX_INFRA_RETRIES ]; then
  echo "Temporal failed to start in time."
  exit 1
fi

# Give Temporal auto-setup a bit more time to finish migrations
echo "Infrastructure is up. Giving Temporal a few seconds to settle..."
sleep 5

echo "Running migrations..."
uv run alembic upgrade head

echo "Initializing COTS catalog (parts.db)..."
uv run python3 -m shared.cots.indexer

echo "Starting Application Servers (Controller, Worker, Temporal Worker)..."

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

# Pre-emptive cleanup of any stale processes from previous runs
pkill -f "uvicorn.*18000" || true
pkill -f "uvicorn.*18001" || true
pkill -f "uvicorn.*18002" || true
pkill -f "python -m controller.temporal_worker" || true

# Start Worker Light (port 18001)
export WORKER_TYPE=light
uv run uvicorn worker_light.app:app --host 0.0.0.0 --port 18001 > "$LOG_DIR/worker_light.log" 2>&1 &
WORKER_LIGHT_PID=$!
echo "Worker Light started (PID: $WORKER_LIGHT_PID)"

# Start Worker Heavy (port 18002)
export WORKER_TYPE=heavy
uv run uvicorn worker_heavy.app:app --host 0.0.0.0 --port 18002 > "$LOG_DIR/worker_heavy.log" 2>&1 &
WORKER_HEAVY_PID=$!
echo "Worker Heavy started (PID: $WORKER_HEAVY_PID)"

# Start Controller (port 18000)
uv run uvicorn controller.api.main:app --host 0.0.0.0 --port 18000 > "$LOG_DIR/controller.log" 2>&1 &
CONTROLLER_PID=$!
echo $CONTROLLER_PID > logs/controller.pid
echo "Controller started (PID: $CONTROLLER_PID)"

# Start Temporal Worker
export PYTHONPATH=$PYTHONPATH:.
uv run python -m controller.temporal_worker > "$LOG_DIR/temporal_worker.log" 2>&1 &
TEMP_WORKER_PID=$!
echo $TEMP_WORKER_PID > logs/temporal_worker.pid
echo "Temporal Worker started (PID: $TEMP_WORKER_PID)"

cleanup() {
  # Record the exit status of the test command
  EXIT_STATUS=$?
  echo ""
  echo "Cleaning up processes (Controller: $CONTROLLER_PID, Worker Light: $WORKER_LIGHT_PID, Worker Heavy: $WORKER_HEAVY_PID, Temporal: $TEMP_WORKER_PID)..."
  
  # Kill the captured PIDs
  kill $CONTROLLER_PID $WORKER_LIGHT_PID $WORKER_HEAVY_PID $TEMP_WORKER_PID 2>/dev/null || true
  
  # Force kill any remaining uvicorn/worker processes by pattern to handle orphans
  # We use -9 here as some processes (especially when uv run is involved) can hang
  pkill -9 -f "uvicorn.*18000" || true
  pkill -9 -f "uvicorn.*18001" || true
  pkill -9 -f "python -m controller.temporal_worker" || true
  pkill -9 -f "uv run uvicorn" || true
  
  # Remove PID files
  rm -f logs/worker.pid logs/controller.pid logs/temporal_worker.pid
  
  # Clean up shared sessions directory
  if [ -n "$WORKER_SESSIONS_DIR" ] && [ -d "$WORKER_SESSIONS_DIR" ]; then
    rm -rf "$WORKER_SESSIONS_DIR"
  fi

  echo "Bringing down infrastructure containers..."
  docker compose -f docker-compose.test.yaml down -v
  
  exit $EXIT_STATUS
}

# Trap signals for cleanup
trap cleanup SIGINT SIGTERM EXIT

echo "Waiting for servers to be healthy..."
MAX_HEALTH_RETRIES=60
HEALTH_COUNT=0
while [ $HEALTH_COUNT -lt $MAX_HEALTH_RETRIES ]; do
  if ! kill -0 $CONTROLLER_PID 2>/dev/null; then
    echo "Controller process (PID: $CONTROLLER_PID) died unexpectedly!"
    echo "--- LAST 20 LINES OF CONTROLLER LOG ---"
    tail -n 20 logs/controller.log
    exit 1
  fi
  if curl -s http://127.0.0.1:18000/health | grep -q "healthy"; then
    echo "Controller is healthy!"
    break
  fi
  echo "Waiting for controller... ($HEALTH_COUNT/$MAX_HEALTH_RETRIES)"
  sleep 2
  HEALTH_COUNT=$((HEALTH_COUNT + 1))
done

# Wait for Workers to be healthy
HEALTH_COUNT=0
while [ $HEALTH_COUNT -lt $MAX_HEALTH_RETRIES ]; do
  if curl -s http://127.0.0.1:18001/health | grep -q "healthy" && curl -s http://127.0.0.1:18002/health | grep -q "healthy"; then
    echo "Workers are healthy!"
    break
  fi
  echo "Waiting for workers... ($HEALTH_COUNT/$MAX_HEALTH_RETRIES)"
  sleep 2
  HEALTH_COUNT=$((HEALTH_COUNT + 1))
done

# Final check that all processes are still alive
if ! kill -0 $CONTROLLER_PID 2>/dev/null; then echo "Controller died!"; exit 1; fi
if ! kill -0 $WORKER_LIGHT_PID 2>/dev/null; then echo "Worker Light died!"; exit 1; fi
if ! kill -0 $WORKER_HEAVY_PID 2>/dev/null; then echo "Worker Heavy died!"; exit 1; fi
if ! kill -0 $TEMP_WORKER_PID 2>/dev/null; then
  echo "Temporal Worker (PID: $TEMP_WORKER_PID) died unexpectedly!"
  echo "--- LAST 20 LINES OF TEMPORAL WORKER LOG ---"
  tail -n 20 logs/temporal_worker.log
  exit 1
fi

if [ $HEALTH_COUNT -eq $MAX_HEALTH_RETRIES ]; then
  echo "Timeout waiting for services to be healthy."
  echo "--- LAST 20 LINES OF CONTROLLER LOG ---"
  tail -n 20 logs/controller.log
  exit 1
fi

# Parse arguments
REVERSE_FLAG=""
PYTEST_ARGS=()

while [[ $# -gt 0 ]]; do
  case $1 in
    --reverse)
      REVERSE_FLAG="--reverse"
      shift
      ;;
    --no-smoke)
      echo "High-fidelity simulation ENABLED (smoke test mode DISABLED)"
      export SMOKE_TEST_MODE=false
      shift
      ;;
    *)
      PYTEST_ARGS+=("$1")
      shift
      ;;
  esac
done

# If no marker/file specified, add default markers
HAS_MARKER=false
HAS_FILE=false
for arg in "${PYTEST_ARGS[@]}"; do
  if [[ "$arg" == "-m" ]]; then HAS_MARKER=true; fi
  if [[ "$arg" == tests/* ]] || [[ "$arg" == */tests/* ]]; then HAS_FILE=true; fi
done

if [ "$HAS_MARKER" = false ] && [ "$HAS_FILE" = false ]; then
  if [ ${#PYTEST_ARGS[@]} -eq 1 ]; then
    # Treat single positional arg as the marker
    MARKER="${PYTEST_ARGS[0]}"
    PYTEST_ARGS=("-m" "$MARKER")
  elif [ ${#PYTEST_ARGS[@]} -eq 0 ]; then
    # No args, use default markers
    PYTEST_ARGS=("-m" "integration_p0 or integration_p1 or integration_p2")
  fi
  # If more than 1 arg and no -m/file, we don't know what to do, 
  # so we leave it to pytest (it might be -k, etc.)
fi

# We use uv run to execute pytest within the virtual environment. 
# We pass -n0 to ensure integration tests run sequentially (stateful infra).
# We override addopts to clear the default 'not integration' markers from pyproject.toml
if uv run pytest -v -o "addopts=-n0" --maxfail=3 -s $REVERSE_FLAG "${PYTEST_ARGS[@]}"; then
  echo "Integration tests PASSED!"
else
  echo "Integration tests FAILED!"
  # The trap will handle the exit status
fi
