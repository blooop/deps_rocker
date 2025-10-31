#!/bin/bash
set -e

echo "Testing ROS Jazzy Environment Variables with proper sourcing..."

# Force interactive shell behavior to ensure bashrc is read
export BASH_ENV="$HOME/.bashrc"

# Debug information
echo "DEBUG: USERNAME=$USERNAME"
echo "DEBUG: HOME=$HOME"
echo "DEBUG: Current shell: $0"
echo "DEBUG: BASH_ENV=$BASH_ENV"
echo ""

echo "=== BEFORE SOURCING ==="
echo "ROS_UNDERLAY_ROOT='$ROS_UNDERLAY_ROOT'"
echo ""

echo "=== BASHRC CONTENT CHECK ==="
if [ -f "$HOME/.bashrc" ]; then
    echo "$HOME/.bashrc exists, showing environment variable lines:"
    grep "export ROS_" "$HOME/.bashrc" || echo "No ROS exports found"
else
    echo "$HOME/.bashrc does not exist"
fi
echo ""

echo "=== MANUAL SOURCING WITH EXPLICIT BASH ==="
# Use bash -c to run in a context where bashrc would be sourced
bash -c '
    source "$HOME/.bashrc"
    echo "Inside sourced bash context:"
    echo "ROS_UNDERLAY_ROOT=$ROS_UNDERLAY_ROOT"
    echo "ROS_OVERLAY_ROOT=$ROS_OVERLAY_ROOT"
    echo "HOME=$HOME"
    
    # Test the actual values expected by the specification
    expected_underlay="$HOME/underlay"
    expected_overlay="$HOME/overlay"
    
    if [ "$ROS_UNDERLAY_ROOT" = "$expected_underlay" ]; then
        echo "✓ ROS_UNDERLAY_ROOT matches expected value"
    else
        echo "✗ ROS_UNDERLAY_ROOT mismatch - Expected: $expected_underlay, Got: $ROS_UNDERLAY_ROOT"
        exit 1
    fi
    
    if [ "$ROS_OVERLAY_ROOT" = "$expected_overlay" ]; then
        echo "✓ ROS_OVERLAY_ROOT matches expected value"
    else
        echo "✗ ROS_OVERLAY_ROOT mismatch - Expected: $expected_overlay, Got: $ROS_OVERLAY_ROOT"
        exit 1
    fi
    
    echo "All environment variables match specification!"
'

echo "Test completed successfully!"