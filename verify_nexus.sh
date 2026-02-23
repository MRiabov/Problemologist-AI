#!/bin/bash
PROJ_ROOT="/home/maksym/Work/proj/Problemologist/Problemologist-AI"
PULSE_FILE="$PROJ_ROOT/nexus_pulse.md"
STATE_FILE="$PROJ_ROOT/nexus_state.json"

echo "=== Nexus Orchestrator Health Check ==="

if [ ! -f "$PULSE_FILE" ]; then
    echo "✗ PULSE file missing. Orchestrator may not be running."
    exit 1
fi

echo "Current Pulse:"
cat "$PULSE_FILE"

echo ""
echo "=== Activity Verification ==="
# Check for Nexus author in git history
RECENT_MERGES=$(git -C "$PROJ_ROOT" log --author="Nexus" --since="4 hours ago" --oneline)
if [ -z "$RECENT_MERGES" ]; then
    echo "✗ No merges by Nexus in the last 4 hours."
else
    echo "✓ Recent Nexus merges found:"
    echo "$RECENT_MERGES"
fi

# Check for running processes
if pgrep -f "nexus_orchestrator.py" > /dev/null; then
    echo "✓ Orchestrator process is RUNNING."
else
    echo "✗ Orchestrator process is MISSING."
fi

if pgrep -f "pytest" > /dev/null; then
    echo "✓ Integration tests are currently ACTIVE."
fi
