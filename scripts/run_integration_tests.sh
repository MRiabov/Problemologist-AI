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
  if curl -s http://localhost:18000/health | grep -q "healthy"; then
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
MARKER=${1:-"integration_p0 or integration_p1 or integration_p2"}

# Playwright setup
echo "Installing Playwright browser..."
uv run playwright install chromium

echo "Running integration tests with marker: $MARKER..."
if uv run pytest -v -m "$MARKER" --maxfail=3 -s -p no:xdist; then
  echo "Integration tests PASSED!"
  INTEGRATION_SUCCESS=1
else
  echo "Integration tests FAILED!"
  docker compose logs controller
  docker compose logs worker
  docker compose logs controller-worker
  INTEGRATION_SUCCESS=0
fi

echo "Running Playwright E2E tests..."
# We run the specific playwright test. It uses the 'e2e' marker.
if uv run pytest tests/e2e/test_playwright_benchmark.py -v -s; then
  echo "Playwright E2E tests PASSED!"
  E2E_SUCCESS=1
else
  echo "Playwright E2E tests FAILED!"
  # Don't fail the whole script immediately if E2E fails, 
  # but we'll report it in the final status.
  E2E_SUCCESS=0
fi

cleanup

if [ $INTEGRATION_SUCCESS -eq 1 ] && [ $E2E_SUCCESS -eq 1 ]; then
  exit 0
else
  exit 1
fi
