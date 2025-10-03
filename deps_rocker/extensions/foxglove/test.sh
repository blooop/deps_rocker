#!/bin/bash

set -e

echo "Testing Foxglove Studio installation..."

# Check if foxglove-studio command is available
if ! command -v foxglove-studio &> /dev/null; then
    echo "ERROR: foxglove-studio command not found"
    exit 1
fi

# Check version (this should work even without display)
echo "Testing foxglove-studio version..."
foxglove-studio --version

# Verify required dependencies are installed
echo "Verifying required system dependencies..."

if ! dpkg -l | grep -q libnotify4; then
    echo "ERROR: libnotify4 not installed"
    exit 1
fi

if ! dpkg -l | grep -q xdg-utils; then
    echo "ERROR: xdg-utils not installed"
    exit 1
fi

if ! dpkg -l | grep -q libappindicator3-1; then
    echo "ERROR: libappindicator3-1 not installed"
    exit 1
fi

echo "Foxglove Studio extension test completed successfully!"