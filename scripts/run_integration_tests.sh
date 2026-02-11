#!/bin/bash
set -e

# Problemologist Integration Test Runner
# This script brings up the full stack using docker compose and runs integration tests.

# Cleanup function
cleanup() {
  echo "Cleaning up..."
  docker compose down -v
}

# Ensure we are in the project root
cd "$(dirname "$0")/.."

echo "Building and starting services..."
docker compose up -d --build

echo "Waiting for services to be healthy..."
# We wait for the controller to be healthy, which depends on others
MAX_RETRIES=30
COUNT=0
while [ $COUNT -lt $MAX_RETRIES ]; do
  if curl -s http://localhost:8000/health | grep -q "healthy"; then
    echo "Controller is healthy!"
    break
  fi
  echo "Waiting for controller... ($COUNT/$MAX_RETRIES)"
  sleep 2
  COUNT=$((COUNT + 1))
done

if [ $COUNT -eq $MAX_RETRIES ]; then
  echo "Timeout waiting for services to be healthy."
  docker compose logs
  cleanup
  exit 1
fi

# Default marker to run
MARKER=${1:-integration_p0}

echo "Running integration tests with marker: $MARKER..."
if uv run pytest -v -m "$MARKER" --maxfail=3 -s -p no:xdist; then
  echo "Integration tests PASSED!"
  SUCCESS=1
else
  echo "Integration tests FAILED!"
  docker compose logs controller
  docker compose logs worker
  docker compose logs controller-worker
  SUCCESS=0
fi

cleanup

if [ $SUCCESS -eq 1 ]; then
  exit 0
else
  exit 1
fi
