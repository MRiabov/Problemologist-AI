#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."

# Ensure integration test log directory exists
mkdir -p logs/integration_tests

# Default to fail-fast integration triage when backend errors are already conclusive.
# Override per run with:
#   INTEGRATION_EARLY_STOP_ON_BACKEND_ERRORS=0 ./scripts/run_integration_tests.sh ...
export INTEGRATION_EARLY_STOP_ON_BACKEND_ERRORS="${INTEGRATION_EARLY_STOP_ON_BACKEND_ERRORS:-1}"
# Default to non-blocking teardown so test command exits immediately after pytest.
# Override per run with:
#   INTEGRATION_ASYNC_CLEANUP=0 ./scripts/run_integration_tests.sh ...
export INTEGRATION_ASYNC_CLEANUP="${INTEGRATION_ASYNC_CLEANUP:-1}"

exec python3 scripts/internal/integration_runner.py run "$@"
