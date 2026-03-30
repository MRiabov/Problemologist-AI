#!/bin/bash
# scripts/env_down.sh
# Stops the Problemologist-AI development environment.

set -e

# Ensure we are in project root
cd "$(dirname "$0")/.."

STACK_PROFILE="${PROBLEMOLOGIST_STACK_PROFILE:-integration}"
while [ "$#" -gt 0 ]; do
  case "$1" in
    --profile)
      if [ "$#" -lt 2 ]; then
        echo "Missing value for --profile"
        exit 1
      fi
      STACK_PROFILE="$2"
      shift 2
      ;;
    --profile=*)
      STACK_PROFILE="${1#*=}"
      shift
      ;;
    *)
      echo "Unknown argument: $1"
      exit 1
      ;;
  esac
done
export PROBLEMOLOGIST_STACK_PROFILE="$STACK_PROFILE"

eval "$(python3 -m evals.logic.stack_profiles --profile "$STACK_PROFILE" --root "$(pwd)" --format shell)"

echo "Stopping Application Servers..."

stop_pid_file() {
  local label="$1"
  local pid_file="$2"
  if [ -f "$pid_file" ]; then
    PID=$(cat "$pid_file" | tr -d '\r\n')
    if [ -n "$PID" ]; then
      if [[ "$PID" =~ ^[0-9]+$ ]]; then
        echo "Stopping ${label} (PID: $PID)..."
        kill "$PID" 2>/dev/null || true
      else
        echo "Stopping ${label} container (ID: $PID)..."
        docker stop "$PID" 2>/dev/null || true
      fi
    fi
    rm -f "$pid_file"
  fi
}

stop_pid_file "Worker Light" "$STACK_PID_DIR/worker_light.pid"
stop_pid_file "Worker Renderer" "$STACK_PID_DIR/worker_renderer.pid"
stop_pid_file "Worker Heavy" "$STACK_PID_DIR/worker_heavy.pid"
stop_pid_file "Controller" "$STACK_PID_DIR/controller.pid"
stop_pid_file "Temporal Worker" "$STACK_PID_DIR/temporal_worker.pid"
stop_pid_file "Heavy Temporal Worker" "$STACK_PID_DIR/worker_heavy_temporal.pid"
stop_pid_file "Frontend dev server" "$STACK_PID_DIR/frontend.pid"

docker compose -p "$COMPOSE_PROJECT_NAME" -f docker-compose.yml stop worker-renderer >/dev/null 2>&1 || true

if [ "$STACK_CREATE_ROOT_LOG_SYMLINKS" = "1" ]; then
  rm -f \
    logs/controller.log \
    logs/controller_debug.log \
    logs/worker_light.log \
    logs/worker_light_debug.log \
    logs/worker_renderer.log \
    logs/worker_renderer_debug.log \
    logs/worker_heavy.log \
    logs/worker_heavy_debug.log \
    logs/worker_heavy_temporal.log \
    logs/worker_heavy_temporal_debug.log \
    logs/temporal_worker.log \
    logs/temporal_worker_debug.log \
    logs/frontend.log
fi

echo "Bringing down infrastructure containers..."
docker compose -p "$COMPOSE_PROJECT_NAME" -f docker-compose.test.yaml down -v --remove-orphans

echo "Environment is DOWN."
