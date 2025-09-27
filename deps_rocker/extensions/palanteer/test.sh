#!/bin/bash
set -e

echo "Testing palanteer installation..."

# Debug: Show PATH and check for palanteer binaries
echo "DEBUG: PATH=$PATH"
echo "DEBUG: Looking for palanteer binaries..."
find /usr -name "palanteer" 2>/dev/null || echo "No palanteer found in /usr"
find /usr/local -name "palanteer" 2>/dev/null || echo "No palanteer found in /usr/local"

# Test that Python palanteer module can be imported
echo "Testing Python palanteer module import..."
if ! python3 -c "import palanteer" &> /dev/null; then
    echo "ERROR: palanteer Python module not found"
    exit 1
fi
echo "✓ Python palanteer module import successful"

# Test palanteer Python module help (without arguments it shows help)
echo "Testing Python palanteer module usage..."
echo "print('Hello from test script')" > /tmp/test_script.py
if ! python3 -m palanteer 2>&1 | grep -q "Palanteer profiler usage"; then
    echo "ERROR: python3 -m palanteer did not show expected help output"
    exit 1
fi
echo "✓ Python palanteer module usage successful"

# Test that palanteer command is available (REQUIRED)
echo "Testing palanteer command availability..."
if ! command -v palanteer &> /dev/null; then
    echo "ERROR: palanteer command not found in PATH"
    exit 1
fi
echo "✓ palanteer command found"

# Test palanteer command help
echo "Testing palanteer command help..."
if ! palanteer --help &> /dev/null; then
    echo "ERROR: palanteer --help failed"
    exit 1
fi
echo "✓ palanteer --help successful"

echo "palanteer extension test completed successfully!"
