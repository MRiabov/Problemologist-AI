#!/bin/bash
set -e

###############################################################################
# CRITICAL: DO NOT REFACTOR THIS AWAY OR REMOVE IT FROM STARTUP SCRIPTS
#
# This script is REQUIRED for Docker-in-Docker (DinD) and sandboxed environments
# (like those used for agent execution). In these environments, the default
# 'overlay2' or 'overlay' storage driver is often incompatible with the
# underlying filesystem, causing Docker containers to fail during startup
# with 'invalid argument' mount errors.
#
# This script detects these failures and automatically switches Docker to use
# the 'vfs' storage driver, which is slower but guaranteed to work in almost
# all nested container scenarios.
###############################################################################

if ! command -v docker >/dev/null 2>&1; then
    echo "Docker not found, skipping storage driver check."
    exit 0
fi

CURRENT_DRIVER=$(docker info -f '{{.Driver}}' 2>/dev/null || echo "unknown")

if [ "$CURRENT_DRIVER" = "vfs" ]; then
    echo "Docker is already using vfs storage driver. Applying headless rendering fixes for sandboxed environment..."
    export MUJOCO_GL=osmesa
    export PYOPENGL_PLATFORM=osmesa
    (return 0 2>/dev/null) && return 0 || exit 0
fi

echo "Current Docker storage driver: $CURRENT_DRIVER"

# Check if we can run a simple container
# Using a mirror as preferred in this environment
if docker run --rm mirror.gcr.io/library/alpine:latest true >/dev/null 2>&1; then
    echo "Docker is working correctly with $CURRENT_DRIVER."
    # Main test platform - no special rendering env vars needed (uses default/xvfb)
    (return 0 2>/dev/null) && return 0 || exit 0
fi

echo "Docker seems to be failing with $CURRENT_DRIVER (likely a Docker-in-Docker environment). Attempting to switch to vfs..."
# Sandboxed/DinD environment - use osmesa for rendering
export MUJOCO_GL=osmesa
export PYOPENGL_PLATFORM=osmesa

if ! command -v sudo >/dev/null 2>&1; then
    echo "Error: sudo is required to change Docker configuration but not found."
    exit 1
fi

# Create or update daemon.json
TMP_JSON=$(mktemp)
if [ -f /etc/docker/daemon.json ]; then
    sudo cat /etc/docker/daemon.json > "$TMP_JSON"
    # Use python to update the JSON if available, otherwise just overwrite
    if command -v python3 >/dev/null 2>&1; then
        python3 -c "import json;
try:
    d=json.load(open('$TMP_JSON'))
except:
    d={}
d['storage-driver']='vfs';
json.dump(d, open('$TMP_JSON', 'w'))"
    else
        echo '{"storage-driver": "vfs"}' > "$TMP_JSON"
    fi
else
    echo '{"storage-driver": "vfs"}' > "$TMP_JSON"
fi

sudo cp "$TMP_JSON" /etc/docker/daemon.json
rm "$TMP_JSON"

echo "Restarting Docker service..."
# Try multiple ways to restart docker as it varies by system
if command -v service >/dev/null 2>&1; then
    sudo service docker restart || true
elif command -v systemctl >/dev/null 2>&1; then
    sudo systemctl restart docker || true
fi

# Wait for docker to be back up
echo "Waiting for Docker to restart..."
for i in {1..10}; do
    if docker info >/dev/null 2>&1; then
        echo "Docker is back up."
        break
    fi
    sleep 1
done

NEW_DRIVER=$(docker info -f '{{.Driver}}' 2>/dev/null || echo "unknown")
echo "New Docker storage driver: $NEW_DRIVER"

if [ "$NEW_DRIVER" = "vfs" ]; then
    echo "Successfully switched to vfs."
else
    echo "Warning: Failed to switch to vfs or Docker did not restart correctly."
fi
