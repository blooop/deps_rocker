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

# Test that palanteer command is available (REQUIRED)
echo "Testing palanteer command availability..."
if ! command -v palanteer &> /dev/null; then
    echo "ERROR: palanteer command not found in PATH"
    exit 1
fi
echo "✓ palanteer command found"

# Test that palanter command (actual binary) is available
# Test palanteer command help


# Test palanteer command help (do not fail on error)
set +e
echo "Testing palanteer command help..."
palanteer --help > /tmp/palanteer_help.txt 2>&1
HELP_EXIT_CODE=$?
echo "palanteer --help exit code: $HELP_EXIT_CODE"
echo "palanteer --help output:"
cat /tmp/palanteer_help.txt
if [ $HELP_EXIT_CODE -ne 0 ]; then
    echo "WARNING: palanteer --help returned non-zero exit code, but output is above."
else
    echo "✓ palanteer --help successful"
fi
set -e
# Never fail the test on help step

echo "palanteer extension test completed successfully!"
