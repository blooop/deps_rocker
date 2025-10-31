#!/bin/bash
set -e

echo "=== DEBUG ENVIRONMENT LOADING ==="
echo "Current user: $(whoami)"
echo "Current HOME: $HOME"
echo "Current working directory: $(pwd)"
echo ""

echo "=== BEFORE SOURCING ==="
echo "ROS_UNDERLAY_ROOT='$ROS_UNDERLAY_ROOT'"
echo ""

echo "=== BASHRC CONTENT ==="
if [ -f "$HOME/.bashrc" ]; then
    echo "$HOME/.bashrc exists, showing last 10 lines:"
    tail -10 "$HOME/.bashrc"
else
    echo "$HOME/.bashrc does not exist"
fi
echo ""

echo "=== SOURCING BASHRC ==="
if [ -f "$HOME/.bashrc" ]; then
    echo "Sourcing $HOME/.bashrc..."
    set +e  # Don't exit on errors during sourcing
    # shellcheck source=/dev/null
    source "$HOME/.bashrc"
    SOURCE_RESULT=$?
    set -e
    echo "Source result: $SOURCE_RESULT"
else
    echo "Cannot source $HOME/.bashrc - file does not exist"
fi
echo ""

echo "=== AFTER SOURCING ==="
echo "ROS_UNDERLAY_ROOT='$ROS_UNDERLAY_ROOT'"
echo "HOME='$HOME'"
echo ""

echo "=== MANUAL EXPORT TEST ==="
export ROS_UNDERLAY_ROOT="$HOME/underlay"
echo "After manual export: ROS_UNDERLAY_ROOT='$ROS_UNDERLAY_ROOT'"
echo ""

echo "=== DIRECTORY TESTS ==="
echo "Checking if $HOME/underlay exists:"
if [ -d "$HOME/underlay" ]; then
    echo "✓ $HOME/underlay exists"
    ls -la "$HOME/underlay" || true
else
    echo "✗ $HOME/underlay does not exist"
fi

echo ""
echo "=== DONE ==="