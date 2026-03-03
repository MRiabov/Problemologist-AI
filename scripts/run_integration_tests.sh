#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."

# Ensure integration test log directory exists
mkdir -p logs/integration_test

exec python3 scripts/internal/integration_runner.py run "$@"
