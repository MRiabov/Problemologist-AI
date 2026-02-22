#!/bin/bash
# scripts/ensure_ngspice.sh
# Ensures that ngspice and libngspice are installed on the host.
# Used by integration tests and local development.

set -e

# Check for ngspice, libngspice, and OSMesa (for headless rendering)
HAS_NGSPICE=$(command -v ngspice >/dev/null 2>&1 && echo "yes" || echo "no")
HAS_LIBNGSPICE=$( [ -f /usr/lib/x86_64-linux-gnu/libngspice.so ] || [ -f /usr/lib/libngspice.so ] && echo "yes" || echo "no")
HAS_OSMESA=$( [ -f /usr/lib/x86_64-linux-gnu/libOSMesa.so ] || [ -f /usr/lib/libOSMesa.so ] && echo "yes" || echo "no")

if [ "$HAS_NGSPICE" = "yes" ] && [ "$HAS_LIBNGSPICE" = "yes" ] && [ "$HAS_OSMESA" = "yes" ]; then
    echo "ngspice, libngspice, and OSMesa already installed."
    exit 0
fi

echo "Installing ngspice, libngspice0-dev, and headless rendering libraries..."
if [ -f /etc/debian_version ]; then
    sudo apt-get update && sudo apt-get install -y ngspice libngspice0 libngspice0-dev libosmesa6 libosmesa6-dev libgl1-mesa-dev
elif [ -f /etc/redhat-release ]; then
    sudo dnf install -y ngspice libngspice-devel
else
    echo "Unsupported OS for automatic ngspice installation. Please install manually."
    exit 1
fi
