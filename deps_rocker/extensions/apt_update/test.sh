#!/bin/bash
set -e

echo "Testing apt_update extension..."

# Check if apt-get command is available (should always be on Ubuntu/Debian)
if ! command -v apt-get &> /dev/null; then
    echo "ERROR: apt-get command not found"
    exit 1
fi

# Verify that apt lists directory exists (created by apt update)
if [ ! -d "/var/lib/apt/lists" ]; then
    echo "ERROR: /var/lib/apt/lists directory not found"
    exit 1
fi

# Check if any package lists exist (indicating apt update ran)
if [ -z "$(ls -A /var/lib/apt/lists/ 2>/dev/null)" ]; then
    echo "ERROR: No package lists found - apt update may not have run properly"
    exit 1
fi

echo "apt_update extension test completed successfully!"