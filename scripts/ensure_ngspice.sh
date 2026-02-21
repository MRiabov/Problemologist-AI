#!/bin/bash
# scripts/ensure_ngspice.sh
# Ensures that ngspice and libngspice are installed on the host.
# Used by integration tests and local development.

set -e

if command -v ngspice >/dev/null 2>&1 && [ -f /usr/lib/x86_64-linux-gnu/libngspice.so ] || [ -f /usr/lib/libngspice.so ]; then
    echo "ngspice and libngspice already installed."
    exit 0
fi

echo "Installing ngspice and libngspice0-dev..."
if [ -f /etc/debian_version ]; then
    sudo apt-get update && sudo apt-get install -y ngspice libngspice0-dev
elif [ -f /etc/redhat-release ]; then
    sudo dnf install -y ngspice libngspice-devel
else
    echo "Unsupported OS for automatic ngspice installation. Please install manually."
    exit 1
fi
