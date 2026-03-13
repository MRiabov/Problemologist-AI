#!/bin/bash
# scripts/env_up.sh
# Sets up the Problemologist-AI development environment using local infra and FastAPI servers.
# Use this for local debugging, optimization, or manual testing.

set -e

# Ensure we are in project root
cd "$(dirname "$0")/.."

# Stop any existing environment first to ensure a clean start
./scripts/env_down.sh

# Networking for local services (infra is still in Docker but exposed on host).
# Default to real/manual mode unless explicitly overridden by caller.
export IS_INTEGRATION_TEST="${IS_INTEGRATION_TEST:-false}"
export LOG_LEVEL=${LOG_LEVEL:-INFO}
export PYTHONUNBUFFERED=1
export WORKER_SESSIONS_DIR="${WORKER_SESSIONS_DIR:-/tmp/pb-sessions-dev}"
mkdir -p "$WORKER_SESSIONS_DIR"

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
    if [[ "$clean_line" =~ ^[A-Za-z_][A-Za-z0-9_]*=.*$ ]]; then
      clean_line=$(echo "$clean_line" | sed -e 's/[[:space:]]*$//')
      key="${clean_line%%=*}"
      value="${clean_line#*=}"
      # If the value is wrapped in matching quotes, strip one layer so env vars
      # contain the raw value (e.g. https://... instead of \"https://...\").
      if [[ "$value" =~ ^\".*\"$ ]]; then
        value="${value:1:-1}"
      elif [[ "$value" =~ ^\'.*\'$ ]]; then
        value="${value:1:-1}"
      fi
      # Preserve explicit caller-provided env so eval/integration runs can
      # override .env-backed provider defaults.
      if [[ -z "${!key+x}" ]]; then
        export "$key=$value"
      fi
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
LOG_DIR="${LOG_DIR:-logs/manual_run}"
ARCHIVE_DIR="${ARCHIVE_DIR:-logs/archives}"
SKIP_LOG_ARCHIVE="${SKIP_LOG_ARCHIVE:-0}"
mkdir -p "$LOG_DIR"
mkdir -p "$ARCHIVE_DIR"

# Archive previous logs if they exist
if [ "$SKIP_LOG_ARCHIVE" != "1" ] && [ -d "$LOG_DIR" ] && [ "$(ls -A "$LOG_DIR")" ]; then
  TIMESTAMP=$(date +%Y%m%d_%H%M%S)
  mv "$LOG_DIR" "$ARCHIVE_DIR/run_${TIMESTAMP}"
  mkdir -p "$LOG_DIR"
fi

# Clean up archives older than 24 hours
find "$ARCHIVE_DIR" -maxdepth 1 -name "run_*" -mmin +1440 -exec rm -rf {} + 2>/dev/null || true

# Start Worker Light (port 18001)
export PYTHONPATH=$PYTHONPATH:.
export WORKER_TYPE=light
export EXTRA_DEBUG_LOG="$LOG_DIR/worker_light_debug.log"
nohup .venv/bin/python -m uvicorn worker_light.app:app --host 0.0.0.0 --port 18001 > "$LOG_DIR/worker_light.log" 2>&1 &
WORKER_LIGHT_PID=$!
echo $WORKER_LIGHT_PID > logs/worker_light.pid
echo "Worker Light started (PID: $WORKER_LIGHT_PID)"

# Start Worker Heavy (port 18002)
export WORKER_TYPE=heavy
export EXTRA_DEBUG_LOG="$LOG_DIR/worker_heavy_debug.log"
nohup .venv/bin/python -m uvicorn worker_heavy.app:app --host 0.0.0.0 --port 18002 > "$LOG_DIR/worker_heavy.log" 2>&1 &
WORKER_HEAVY_PID=$!
echo $WORKER_HEAVY_PID > logs/worker_heavy.pid
echo "Worker Heavy started (PID: $WORKER_HEAVY_PID)"

# Start Controller (port 18000)
export EXTRA_DEBUG_LOG="$LOG_DIR/controller_debug.log"
nohup .venv/bin/python -m uvicorn controller.api.main:app --host 0.0.0.0 --port 18000 > "$LOG_DIR/controller.log" 2>&1 &
CONTROLLER_PID=$!
echo $CONTROLLER_PID > logs/controller.pid
echo "Controller started (PID: $CONTROLLER_PID)"

# Start Temporal Worker
export PYTHONPATH=$PYTHONPATH:.
export EXTRA_DEBUG_LOG="$LOG_DIR/temporal_worker_debug.log"
nohup .venv/bin/python -m controller.temporal_worker > "$LOG_DIR/temporal_worker.log" 2>&1 &
TEMP_WORKER_PID=$!
echo $TEMP_WORKER_PID > logs/temporal_worker.pid
echo "Temporal Worker started (PID: $TEMP_WORKER_PID)"

# Start Heavy Temporal Worker (separate from worker-heavy API process)
export EXTRA_DEBUG_LOG="$LOG_DIR/worker_heavy_temporal_debug.log"
nohup .venv/bin/python -m worker_heavy.temporal_worker > "$LOG_DIR/worker_heavy_temporal.log" 2>&1 &
HEAVY_TEMP_WORKER_PID=$!
echo $HEAVY_TEMP_WORKER_PID > logs/worker_heavy_temporal.pid
echo "Heavy Temporal Worker started (PID: $HEAVY_TEMP_WORKER_PID)"

# Start Frontend dev server if default port is available
FRONTEND_PORT=15173
FRONTEND_STARTED=false
if python3 - "$FRONTEND_PORT" <<'PY'
import socket, sys
port = int(sys.argv[1])
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.settimeout(0.2)
    sys.exit(0 if s.connect_ex(("127.0.0.1", port)) == 0 else 1)
PY
then
  echo "Frontend port ${FRONTEND_PORT} is already in use; skipping npm dev server startup."
else
  echo "Starting Frontend dev server on port ${FRONTEND_PORT}..."
  (
    cd frontend
    nohup npm run dev > "../$LOG_DIR/frontend.log" 2>&1 &
    echo $! > ../logs/frontend.pid
  )
  FRONTEND_STARTED=true
  echo "Frontend dev server started (PID: $(cat logs/frontend.pid))"
fi

# Create convenience symlinks in the logs/ root for the current environment
# We strip the logs/ prefix from LOG_DIR to get the relative target
REL_LOG_DIR="${LOG_DIR#logs/}"
ln -sf "$REL_LOG_DIR/controller.log" logs/controller.log
ln -sf "$REL_LOG_DIR/controller_debug.log" logs/controller_debug.log
ln -sf "$REL_LOG_DIR/worker_light.log" logs/worker_light.log
ln -sf "$REL_LOG_DIR/worker_light_debug.log" logs/worker_light_debug.log
ln -sf "$REL_LOG_DIR/worker_heavy.log" logs/worker_heavy.log
ln -sf "$REL_LOG_DIR/worker_heavy_debug.log" logs/worker_heavy_debug.log
ln -sf "$REL_LOG_DIR/worker_heavy_temporal.log" logs/worker_heavy_temporal.log
ln -sf "$REL_LOG_DIR/worker_heavy_temporal_debug.log" logs/worker_heavy_temporal_debug.log
ln -sf "$REL_LOG_DIR/temporal_worker.log" logs/temporal_worker.log
ln -sf "$REL_LOG_DIR/temporal_worker_debug.log" logs/temporal_worker_debug.log
ln -sf "$REL_LOG_DIR/frontend.log" logs/frontend.log

echo "Waiting for services to be healthy..."
sleep 2

wait_for_health() {
  local name="$1"
  local url="$2"
  local retries="${3:-30}"
  local delay="${4:-1}"
  local i=0

  while [ "$i" -lt "$retries" ]; do
    if curl -s "$url" | grep -q "healthy"; then
      echo "$name is healthy!"
      return 0
    fi
    i=$((i + 1))
    sleep "$delay"
  done

  echo "$name health check failed after ${retries}s (see logs in $LOG_DIR)"
  return 1
}

check_pid_alive() {
  local name="$1"
  local pid="$2"
  if kill -0 "$pid" 2>/dev/null; then
    return 0
  fi
  echo "$name process is not running (PID: $pid)"
  return 1
}

FAIL=0

check_pid_alive "Worker Light" "$WORKER_LIGHT_PID" || FAIL=1
check_pid_alive "Worker Heavy" "$WORKER_HEAVY_PID" || FAIL=1
check_pid_alive "Controller" "$CONTROLLER_PID" || FAIL=1
check_pid_alive "Temporal Worker" "$TEMP_WORKER_PID" || FAIL=1
check_pid_alive "Heavy Temporal Worker" "$HEAVY_TEMP_WORKER_PID" || FAIL=1

wait_for_health "Worker Light" "http://127.0.0.1:18001/health" || FAIL=1
wait_for_health "Worker Heavy" "http://127.0.0.1:18002/health" || FAIL=1
wait_for_health "Controller" "http://127.0.0.1:18000/health" || FAIL=1

if [ "$FRONTEND_STARTED" = true ]; then
  sleep 2
  if curl -sf "http://127.0.0.1:${FRONTEND_PORT}" > /dev/null 2>&1; then
    echo "Frontend is healthy on http://127.0.0.1:${FRONTEND_PORT}!"
  else
    echo "Frontend health check failed (see logs/manual_run/frontend.log)"
  fi
fi

if [ "$FAIL" -ne 0 ]; then
  echo "Environment startup failed."
  exit 1
fi

echo "Environment is UP. Use 'scripts/env_down.sh' to stop."
echo "Mode: IS_INTEGRATION_TEST=$IS_INTEGRATION_TEST"
