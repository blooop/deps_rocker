#!/bin/bash

set -e

echo "Testing Foxglove Studio installation..."

# Check if foxglove-studio command is available
if ! command -v foxglove-studio &> /dev/null; then
    echo "ERROR: foxglove-studio command not found"
    exit 1
fi

# Check if foxglove-studio package is installed via dpkg
if ! dpkg -l | grep -q foxglove-studio; then
    echo "ERROR: foxglove-studio package not installed"
    exit 1
fi

# Check version (this should work even without display)
echo "Testing foxglove-studio version..."
dpkg-query -W -f='${Version}\n' foxglove-studio

echo "Foxglove Studio extension test completed successfully!"
