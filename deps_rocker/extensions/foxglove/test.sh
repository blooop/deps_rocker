#!/bin/bash

set -e

echo "Testing Foxglove Studio installation..."

# Check if snapd is installed
if ! command -v snap &> /dev/null; then
    echo "ERROR: snap command not found"
    exit 1
fi

# Check if foxglove-studio snap is installed
if ! snap list | grep -q foxglove-studio; then
    echo "ERROR: foxglove-studio snap not installed"
    exit 1
fi

# Check if foxglove-studio command is available
if ! command -v foxglove-studio &> /dev/null; then
    echo "ERROR: foxglove-studio command not found"
    exit 1
fi

# Check version (this should work even without display)
echo "Testing foxglove-studio version..."
foxglove-studio --version

echo "Foxglove Studio extension test completed successfully!"