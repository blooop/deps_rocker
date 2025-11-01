#!/bin/bash
set -e

echo "Testing workdir extension..."

# Test that the current directory is accessible (basic functionality test)
echo "Current working directory: $(pwd)"
echo "Directory listing:"
ls -la .

echo "workdir extension test completed successfully!"
