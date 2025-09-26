#!/bin/bash

set -e

echo "Testing Codex CLI installation..."

# Debug PATH and npm setup
echo "Current PATH: $PATH"
echo "npm prefix: $(npm prefix -g)"
echo "ls npm prefix bin:"
ls -la "$(npm prefix -g)/bin" | grep -i codex || echo "No codex found in npm prefix bin"

# Check npm global packages
echo "npm global packages:"
npm list -g --depth=0 | grep -i codex || echo "No codex found in global packages"

# Try to find codex binary
echo "Searching for codex binary:"
find /usr -name "codex*" 2>/dev/null || echo "No codex found in /usr"
find $HOME -name "codex*" 2>/dev/null || echo "No codex found in home"

# Check if codex is installed and accessible
if ! command -v codex &> /dev/null; then
    # Try to find and add to PATH
    NPM_PREFIX=$(npm prefix -g)
    if [ -f "$NPM_PREFIX/bin/codex" ]; then
        echo "Found codex in npm prefix bin, adding to PATH"
        export PATH="$NPM_PREFIX/bin:$PATH"
    else
        echo "ERROR: codex command not found"
        echo "Attempting to check if @openai/codex package was properly installed..."
        npm list -g @openai/codex || echo "Package not found in global list"
        exit 1
    fi
fi

# Check codex version
CODEX_VERSION=$(codex --version 2>&1 || echo "version check failed")
echo "Codex version: $CODEX_VERSION"

# Test basic codex functionality (without requiring authentication)
echo "Testing codex help command..."
codex --help > /dev/null

echo "Codex extension test completed successfully!"
