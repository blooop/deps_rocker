#!/bin/bash
set -e

echo "Testing workdir extension..."

# Get current directory
CURRENT_DIR=$(pwd)
echo "Current working directory: $CURRENT_DIR"

# The test framework will pass --workdir /test_workdir when building
# This script runs after WORKDIR is set, so we should be in /test_workdir
if [ "$CURRENT_DIR" = "/test_workdir" ]; then
    echo "SUCCESS: WORKDIR was set correctly to /test_workdir"
else
    echo "ERROR: Expected to be in /test_workdir but found: $CURRENT_DIR"
    exit 1
fi

echo "workdir extension test completed successfully!"
