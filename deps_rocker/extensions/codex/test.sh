#!/bin/bash
set -e

echo "Testing codex installation..."

# Check if codex command is available
if ! command -v codex &> /dev/null; then
    echo "ERROR: codex command not found"
    exit 1
fi

# Test codex version command
codex --version

# # Test that host mounting works by checking for auth file
# echo "Testing host mounting (checking for auth file)..."
# if [ ! -f ~/.codex/auth.json ]; then
#     echo "ERROR: Host mounting failed - ~/.codex/auth.json not found"
#     exit 1
# fi

echo "codex extension test completed successfully!"
