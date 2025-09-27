#!/bin/bash
set -e

echo "Testing palanteer installation..."

# Test that palanteer command is available
if ! command -v palanteer &> /dev/null; then
    echo "ERROR: palanteer command not found"
    exit 1
fi

# Test that Python palanteer module can be imported
if ! python3 -c "import palanteer" &> /dev/null; then
    echo "ERROR: palanteer Python module not found"
    exit 1
fi

# Test palanteer command help
if ! palanteer --help &> /dev/null; then
    echo "ERROR: palanteer --help failed"
    exit 1
fi

# Test palanteer Python module help (without arguments it shows help)
echo "print('Hello from test script')" > /tmp/test_script.py
if ! python3 -m palanteer 2>&1 | grep -q "Palanteer profiler usage"; then
    echo "ERROR: python3 -m palanteer did not show expected help output"
    exit 1
fi

echo "palanteer extension test completed successfully!"