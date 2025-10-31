#!/bin/bash
set -e

echo "Testing ROS Jazzy Environment Variables - Deep Debug..."
echo "USER: $(whoami)"
echo "HOME: $HOME"
echo "PWD: $(pwd)"
echo ""

echo "=== ENVIRONMENT BEFORE ANY SOURCING ==="
echo "ROS_DISTRO: '$ROS_DISTRO'"
echo "ROS_UNDERLAY_ROOT: '$ROS_UNDERLAY_ROOT'"
echo ""

echo "=== BASHRC INVESTIGATION ==="
if [ -f "$HOME/.bashrc" ]; then
    echo "✓ $HOME/.bashrc exists"
    echo "File size: $(wc -c < "$HOME/.bashrc") bytes"
    echo "Last modified: $(stat -c %Y "$HOME/.bashrc")"
    echo ""
    echo "Content of $HOME/.bashrc:"
    cat "$HOME/.bashrc"
    echo ""
    echo "=== SEARCHING FOR ROS EXPORTS ==="
    grep -n "ROS_" "$HOME/.bashrc" || echo "No ROS variables found"
    echo ""
else
    echo "✗ $HOME/.bashrc does not exist"
    exit 1
fi

echo "=== MANUAL EXPORT TESTING ==="
export ROS_UNDERLAY_ROOT="$HOME/underlay"
echo "After manual export: ROS_UNDERLAY_ROOT='$ROS_UNDERLAY_ROOT'"
echo ""

echo "=== NESTED BASH TESTING ==="
echo "Testing environment loading in nested bash..."

bash -c '
    echo "=== INSIDE NESTED BASH ==="
    echo "HOME: $HOME"
    echo "Before sourcing: ROS_UNDERLAY_ROOT=\"$ROS_UNDERLAY_ROOT\""
    
    if [ -f "$HOME/.bashrc" ]; then
        echo "Sourcing $HOME/.bashrc..."
        source "$HOME/.bashrc"
        echo "After sourcing: ROS_UNDERLAY_ROOT=\"$ROS_UNDERLAY_ROOT\""
        echo "HOME resolved: $(echo $HOME)"
        echo "Expected: $HOME/underlay"
        echo "Got: $ROS_UNDERLAY_ROOT"
        
        if [ "$ROS_UNDERLAY_ROOT" = "$HOME/underlay" ]; then
            echo "✓ SUCCESS: Environment variables work in nested bash!"
        else
            echo "✗ FAILED: Environment variables not working even in nested bash"
        fi
    else
        echo "✗ bashrc not found in nested bash"
    fi
'

echo ""
echo "=== FINAL STATE ==="
echo "ROS_UNDERLAY_ROOT: '$ROS_UNDERLAY_ROOT'"