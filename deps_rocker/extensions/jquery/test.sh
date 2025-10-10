#!/bin/bash
set -e

echo "Testing jquery (jq) installation..."

if ! command -v jq &> /dev/null; then
    echo "ERROR: jq command not found"
    exit 1
fi

echo '{"a":1,"b":2}' | jq '.a' | grep -q 1

echo "jq basic functionality confirmed"

echo "jquery extension test completed successfully!"
