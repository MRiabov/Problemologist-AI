#!/bin/bash
echo "Stopping Nexus Orchestrator and active tests..."

# Kill the orchestrator
pkill -f "nexus_orchestrator.py"

# Kill any active test runs
pkill -f "pytest"

# Optional: Bring down test infrastructure
# docker compose -f /home/maksym/Work/proj/Problemologist/Problemologist-AI/docker-compose.test.yaml down -v

echo "Nexus Orchestrator stopped."
