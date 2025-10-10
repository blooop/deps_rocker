#!/bin/bash

set -e

echo "Testing ccache installation..."

# Check that ccache is installed
if ! command -v ccache &> /dev/null
then
    echo "ERROR: ccache command not found"
    exit 1
fi

# Test ccache version
ccache --version
if [ $? -eq 0 ]; then
    echo "ccache is installed and working"
else
    echo "ERROR: ccache is installed but failed to run"
    exit 1
fi

# Verify CCACHE_DIR environment variable is set
if [ -z "$CCACHE_DIR" ]; then
    echo "ERROR: CCACHE_DIR environment variable is not set"
    exit 1
fi

if [ "$CCACHE_DIR" != "/root/.ccache" ]; then
    echo "ERROR: CCACHE_DIR is set to '$CCACHE_DIR', expected '/root/.ccache'"
    exit 1
fi

echo "CCACHE_DIR is correctly set to: $CCACHE_DIR"

# Verify ccache directory exists (should be mounted from host)
if [ ! -d "$CCACHE_DIR" ]; then
    echo "ERROR: ccache directory $CCACHE_DIR does not exist"
    exit 1
fi

echo "ccache directory exists at: $CCACHE_DIR"
echo "ccache extension test completed successfully!"
