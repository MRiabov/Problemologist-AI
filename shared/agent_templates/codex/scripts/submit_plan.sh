#!/bin/sh
set -eu

# Thin wrapper around the Python submission helper.
# The shell script is the agent-facing command surface; Python does the validation.
exec python scripts/submit_plan.py "$@"
