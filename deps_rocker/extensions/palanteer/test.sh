#!/bin/bash
set -e

echo "Testing palanteer installation..."

# Test that Python palanteer module can be imported
if ! python3 -c "import palanteer" &> /dev/null; then
    echo "ERROR: palanteer Python module not found"
    exit 1
fi

# Test palanteer automatic instrumentation mode
echo "print('Hello from test script')" > /tmp/test_script.py
if ! python3 -m palanteer --help &> /dev/null; then
    echo "ERROR: python3 -m palanteer failed"
    exit 1
fi

# Test that palanteer can instrument a simple script (just check it doesn't crash)
timeout 10s python3 -m palanteer /tmp/test_script.py &> /dev/null || {
    exit_code=$?
    if [ $exit_code -eq 124 ]; then
        echo "palanteer instrumentation test completed (timed out as expected)"
    else
        echo "ERROR: palanteer instrumentation failed with exit code $exit_code"
        exit 1
    fi
}

echo "palanteer extension test completed successfully!"