#!/bin/bash

set -e

# Determine active config dir (prefer explicit env)
CONFIG_DIR="${CLAUDE_CONFIG_DIR:-$HOME/.claude}"

echo "whoami=$(whoami)"; id || true
echo "HOME=$HOME"
echo "CLAUDE_CONFIG_DIR=$CONFIG_DIR"
echo "XDG_CONFIG_HOME=${XDG_CONFIG_HOME:-unset}"
echo "XDG_CACHE_HOME=${XDG_CACHE_HOME:-unset}"
echo "XDG_DATA_HOME=${XDG_DATA_HOME:-unset}"

# Show where the binary is resolved from (helps ensure PATH/symlink works)
which claude || true

# Verify the CLI is available and prints a version
claude --version

# Ensure help executes without triggering first-time setup
HELP_OUT=$(claude 2>&1 | head -n 50 || true)
echo "$HELP_OUT" | sed 's/.*/[HELP] &/'
# Fail on onboarding phrases that indicate first-run experience or tips block
if echo "$HELP_OUT" | grep -E "[Ff]irst[[:space:]-]*time" >/dev/null || \
   echo "$HELP_OUT" | grep -F "Tips for getting started:" >/dev/null; then
  echo "ERROR: 'claude' appears to include first-run onboarding text" >&2
  exit 1
fi

# Fail if PATH warning is printed
if echo "$HELP_OUT" | grep -F "~/.local/bin is not in your PATH" >/dev/null || \
   echo "$HELP_OUT" | grep -F "Native installation exists" >/dev/null; then
  echo "ERROR: PATH warning detected in 'claude --help' output (missing ~/.local/bin in PATH)" >&2
  exit 1
fi

echo "Using Claude config at: $CONFIG_DIR"

# Sanity: ensure config dir exists and is writable
if [ ! -d "$CONFIG_DIR" ]; then
  echo "ERROR: Claude config dir does not exist: $CONFIG_DIR" >&2
  exit 1
fi
if [ ! -w "$CONFIG_DIR" ]; then
  echo "ERROR: Claude config dir not writable: $CONFIG_DIR" >&2
  exit 1
fi

# Print top of directory to aid debugging
ls -al "$CONFIG_DIR" | head || true

# Best-effort check for credentials presence
if [ -f "$CONFIG_DIR/.credentials.json" ]; then
  echo "Claude credentials detected."
fi

# Ensure no stray root-owned config dir exists
if [ ! -e "/root/.claude" ]; then
  echo "No stale root Claude config found"
fi

echo "claude is installed and working"
