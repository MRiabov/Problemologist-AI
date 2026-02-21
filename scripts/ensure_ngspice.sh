#!/bin/bash
set -e

###############################################################################
# This script ensures that ngspice and libngspice are installed and correctly
# configured for PySpice to find them.
#
# It is used in integration tests and sandboxed environments to guarantee
# that circuit simulation features are available.
###############################################################################

if command -v ngspice >/dev/null 2>&1 && [ -f /usr/lib/x86_64-linux-gnu/libngspice.so.0 ]; then
    echo "ngspice and libngspice already installed and available."
    exit 0
fi

echo "ngspice or libngspice not found. Attempting to install..."

if ! command -v apt-get >/dev/null 2>&1; then
    echo "Warning: apt-get not found. Cannot install ngspice automatically."
    echo "Please install ngspice, libngspice0, and libngspice0-dev manually."
    exit 0
fi

if ! command -v sudo >/dev/null 2>&1; then
    echo "Warning: sudo not found. Attempting installation without sudo..."
    apt-get update && apt-get install -y ngspice libngspice0 libngspice0-dev || {
        echo "Installation failed. Please install dependencies manually."
    }
else
    echo "Installing ngspice dependencies with sudo..."
    sudo apt-get update && sudo apt-get install -y ngspice libngspice0 libngspice0-dev || {
        echo "Installation failed. Please install dependencies manually."
    }
fi

# Double check if discovery still needs help
if [ ! -f /usr/lib/x86_64-linux-gnu/libngspice.so ] && [ -f /usr/lib/x86_64-linux-gnu/libngspice.so.0 ]; then
    echo "Creating libngspice.so symlink for older PySpice compatibility..."
    if command -v sudo >/dev/null 2>&1; then
        sudo ln -sf /usr/lib/x86_64-linux-gnu/libngspice.so.0 /usr/lib/x86_64-linux-gnu/libngspice.so
    else
        ln -sf /usr/lib/x86_64-linux-gnu/libngspice.so.0 /usr/lib/x86_64-linux-gnu/libngspice.so || true
    fi
fi

echo "ngspice setup complete."
