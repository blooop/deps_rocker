#!/bin/bash
set -e

echo "Testing GitUI installation..."

# Source bashrc to get pixi global binaries in PATH
export PATH="$HOME/.pixi/bin:$PATH"

# Test that gitui command is available
if ! command -v gitui &> /dev/null; then
    echo "ERROR: gitui command not found"
    exit 1
fi

# Test that gitui binary is executable and accessible
echo "GitUI binary location: $(which gitui)"

# Verify the binary file exists and is executable
if [ ! -x "$(which gitui)" ]; then
    echo "ERROR: gitui binary is not executable"
    exit 1
fi

echo "GitUI extension test completed successfully!"