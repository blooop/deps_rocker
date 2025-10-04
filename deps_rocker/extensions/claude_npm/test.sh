#!/bin/bash
set -e

echo "Testing Claude npm extension installation..."

# Test that claude command is available
if ! command -v claude &> /dev/null; then
    echo "ERROR: claude command not found"
    exit 1
fi

# Test that uv was installed (dependency)
if ! command -v uv &> /dev/null; then
    echo "ERROR: uv command not found"
    exit 1
fi

# Test that claude-monitor was installed via uv
if ! command -v claude-monitor &> /dev/null; then
    echo "ERROR: claude-monitor command not found"
    exit 1
fi

# Test that npm is available (required dependency)
if ! command -v npm &> /dev/null; then
    echo "ERROR: npm command not found"
    exit 1
fi

# Verify Claude was installed via npm (check npm global list)
if ! npm list -g @anthropic-ai/claude-code &> /dev/null; then
    echo "ERROR: @anthropic-ai/claude-code not installed via npm"
    exit 1
fi

# Check version outputs
echo "Claude version:"
claude --version || echo "Claude version check returned non-zero (may be expected)"

echo "UV version:"
uv --version

echo "Claude-monitor version:"
claude-monitor --version

# Verify environment variable is set
if [ -z "$CLAUDE_NPM_EXTENSION_VERSION" ]; then
    echo "ERROR: CLAUDE_NPM_EXTENSION_VERSION environment variable not set"
    exit 1
fi

echo "CLAUDE_NPM_EXTENSION_VERSION=$CLAUDE_NPM_EXTENSION_VERSION"

# Check CLAUDE_CONFIG_DIR if set
if [ -n "$CLAUDE_CONFIG_DIR" ]; then
    echo "CLAUDE_CONFIG_DIR is set to: $CLAUDE_CONFIG_DIR"
fi

echo "Claude npm extension test completed successfully!"
