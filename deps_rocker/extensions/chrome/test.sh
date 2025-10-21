#!/bin/bash
set -e

echo "Testing chrome installation..."

if ! command -v google-chrome &> /dev/null; then
    echo "ERROR: google-chrome command not found"
    exit 1
fi

# Test that Chrome can report its version
google-chrome --version

# Test that Chrome can run in headless mode (safe for containerized environments)
google-chrome --headless --disable-gpu --dump-dom --virtual-time-budget=1000 about:blank > /dev/null

echo "chrome extension test completed successfully!"