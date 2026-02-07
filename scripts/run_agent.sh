#!/bin/bash
set -e

# Script to build and run the agent in a container
# Usage: ./scripts/run_agent.sh "Create a 10x10x10 cube"

TASK=$1

if [ -z "$TASK" ]; then
    echo "Usage: $0 "Your task here""
    exit 1
fi

echo "📦 Building agent image..."
docker build -t problemologist-agent -f src/worker/Dockerfile .

echo "🏃 Running agent..."
docker run --rm 
  -e OPENAI_API_KEY=$OPENAI_API_KEY 
  -e SPEC_001_API_URL=${SPEC_001_API_URL:-http://worker:8001} 
  problemologist-agent 
  python -m controller.agent.run "$TASK"
