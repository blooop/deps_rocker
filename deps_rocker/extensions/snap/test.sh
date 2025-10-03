#!/bin/bash

set -e

echo "Testing snap installation..."

# Check if snap command is available
if ! command -v snap &> /dev/null; then
    echo "ERROR: snap command not found"
    exit 1
fi

# Check if snapd service is available (it might not be running in container)
if ! dpkg -l | grep -q snapd; then
    echo "ERROR: snapd package not installed"
    exit 1
fi

# Test basic snap functionality
echo "Testing snap version..."
snap version

echo "snap extension test completed successfully!"