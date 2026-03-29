#!/bin/bash
# scripts/env_down.sh
# Stops the Problemologist-AI development environment.

# Ensure we are in project root
cd "$(dirname "$0")/.."

echo "Stopping Application Servers..."

if [ -f logs/worker_light.pid ]; then
  PID=$(cat logs/worker_light.pid)
  echo "Stopping Worker Light (PID: $PID)..."
  kill $PID 2>/dev/null || true
  rm logs/worker_light.pid
fi

if [ -f logs/worker_heavy.pid ]; then
  PID=$(cat logs/worker_heavy.pid)
  echo "Stopping Worker Heavy (PID: $PID)..."
  kill $PID 2>/dev/null || true
  rm logs/worker_heavy.pid
fi

if [ -f logs/controller.pid ]; then
  PID=$(cat logs/controller.pid)
  echo "Stopping Controller (PID: $PID)..."
  kill $PID 2>/dev/null || true
  rm logs/controller.pid
fi

if [ -f logs/temporal_worker.pid ]; then
  PID=$(cat logs/temporal_worker.pid)
  echo "Stopping Temporal Worker (PID: $PID)..."
  kill $PID 2>/dev/null || true
  rm logs/temporal_worker.pid
fi

if [ -f logs/worker_heavy_temporal.pid ]; then
  PID=$(cat logs/worker_heavy_temporal.pid)
  echo "Stopping Heavy Temporal Worker (PID: $PID)..."
  kill $PID 2>/dev/null || true
  rm logs/worker_heavy_temporal.pid
fi

if [ -f logs/frontend.pid ]; then
  PID=$(cat logs/frontend.pid)
  echo "Stopping Frontend dev server (PID: $PID)..."
  kill $PID 2>/dev/null || true
  rm logs/frontend.pid
fi

# Also kill any leftover FastAPI/Uvicorn processes just in case
pkill -9 -f "uvicorn.*18000" || true
pkill -9 -f "uvicorn.*18001" || true
pkill -9 -f "uvicorn.*18002" || true
pkill -9 -f "fastapi run" || true
pkill -9 -f "controller.temporal_worker" || true
pkill -9 -f "worker_heavy.temporal_worker" || true
pkill -9 -f "uv run uvicorn" || true
pkill -9 -f "uv run python -m controller.temporal_worker" || true
pkill -9 -f "uv run python -m worker_heavy.temporal_worker" || true

# Tear down any Xvfb instances started by the local dev/integration flows.
# The integration runner uses :99, while the VTK fallback helper uses the
# 10000-10100 range when it has to start a private display.
pkill -9 -f "Xvfb :(99|10000|1000[1-9]|100[1-9][0-9]|10100)([[:space:]]|$)" || true
pkill -9 -f "Xvfb -displayfd" || true

echo "Bringing down infrastructure containers..."
docker compose -f docker-compose.test.yaml down -v --remove-orphans

echo "Environment is DOWN."
