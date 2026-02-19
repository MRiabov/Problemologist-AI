#!/bin/bash
# scripts/env_down.sh
# Stops the Problemologist-AI development environment.

# Ensure we are in project root
cd "$(dirname "$0")/.."

echo "Stopping Application Servers..."

if [ -f logs/worker.pid ]; then
  PID=$(cat logs/worker.pid)
  echo "Stopping Worker (PID: $PID)..."
  kill $PID 2>/dev/null || true
  rm logs/worker.pid
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

# Also kill any leftover FastAPI/Uvicorn processes just in case
pkill -9 -f "uvicorn.*18000" || true
pkill -9 -f "uvicorn.*18001" || true
pkill -9 -f "fastapi run" || true
pkill -9 -f "controller.temporal_worker" || true
pkill -9 -f "uv run uvicorn" || true
pkill -9 -f "uv run python -m controller.temporal_worker" || true

echo "Bringing down infrastructure containers..."
docker compose -f docker-compose.test.yaml down

echo "Environment is DOWN."
