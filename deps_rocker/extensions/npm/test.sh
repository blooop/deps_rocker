#!/bin/bash

set -e

echo "Testing npm installation..."

# Check if node is installed and accessible
if ! command -v node &> /dev/null; then
    echo "ERROR: node command not found"
    exit 1
fi

# Check if npm is installed and accessible
if ! command -v npm &> /dev/null; then
    echo "ERROR: npm command not found"
    exit 1
fi

# Check node version
NODE_VERSION=$(node --version)
echo "Node version: $NODE_VERSION"

# Check npm version
NPM_VERSION=$(npm --version)
echo "npm version: $NPM_VERSION"

# Test npm functionality by listing global packages
echo "Testing npm list..."
npm list -g --depth=0

echo "npm extension test completed successfully!"
