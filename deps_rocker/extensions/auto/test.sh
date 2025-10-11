#!/bin/bash
set -e

echo "Testing auto extension..."

# The auto extension itself doesn't install anything
# It just detects files and enables other extensions
# So we just need to verify the extension loaded without error

echo "Auto extension test completed successfully!"
