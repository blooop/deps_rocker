#!/bin/bash
set -e

echo "Testing foxglove installation..."

if ! command -v foxglove-studio &> /dev/null; then
    echo "ERROR: foxglove-studio command not found"
    exit 1
fi

# Verify the binary exists and is executable
if [ ! -x "$(command -v foxglove-studio)" ]; then
    echo "ERROR: foxglove-studio is not executable"
    exit 1
fi

# For GUI applications in headless environments, just verify the binary exists
# The actual functionality is tested when running with proper X11 display
echo "foxglove-studio binary found at: $(command -v foxglove-studio)"

echo "foxglove extension test completed successfully!"
