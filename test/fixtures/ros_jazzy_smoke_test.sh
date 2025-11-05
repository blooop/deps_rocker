#!/bin/bash
set -euo pipefail

log() {
    echo "[$(date +%H:%M:%S)] $*"
}

require_cmd() {
    if ! command -v "$1" >/dev/null 2>&1; then
        echo "ERROR: required command '$1' not found"
        exit 1
    fi
}

log "ROS Jazzy smoke test starting"
require_cmd ros2
require_cmd colcon

workspace="${ROS_WORKSPACE_ROOT:-}"
if [ -z "$workspace" ]; then
    echo "ERROR: ROS_WORKSPACE_ROOT not set"
    exit 1
fi

fixture_source="/tmp/ros_jazzy_fixture"
if [ ! -d "$fixture_source" ]; then
    echo "ERROR: Fixture directory not found at $fixture_source"
    exit 1
fi

cd "$workspace"

fixture_target="test/test_package"
mkdir -p "$(dirname "$fixture_target")"
rm -rf "$fixture_target"
cp -R "$fixture_source" "$fixture_target"
touch "$fixture_target/COLCON_IGNORE"

cleanup() {
    rm -rf src/test_package
    rm -rf "$fixture_target"
}
trap cleanup EXIT

mkdir -p src
rm -rf src/test_package
cp -R "$fixture_target" src/test_package
rm -f src/test_package/COLCON_IGNORE
log "Copied test package into src/test_package"

set +u
source /opt/ros/jazzy/setup.bash
set -u

if [ -n "${ROS_UNDERLAY_INSTALL:-}" ] && [ -f "${ROS_UNDERLAY_INSTALL}/setup.bash" ]; then
    set +u
    source "${ROS_UNDERLAY_INSTALL}/setup.bash"
    set -u
    log "Sourced underlay install from ${ROS_UNDERLAY_INSTALL}"
fi

build_base="${ROS_BUILD_BASE:-$workspace/build}"
install_base="${ROS_INSTALL_BASE:-$workspace/install}"

colcon build \
    --packages-select test_package \
    --build-base "$build_base" \
    --install-base "$install_base" \
    --merge-install \
    --event-handlers console_direct+

if [ ! -f "${install_base}/setup.bash" ]; then
    echo "ERROR: Expected workspace setup file at ${install_base}/setup.bash"
    exit 1
fi

set +u
source "${install_base}/setup.bash"
set -u

if ! ros2 pkg list | grep -q test_package; then
    echo "ERROR: test_package not found after colcon build"
    exit 1
fi

log "ROS Jazzy smoke test completed successfully"
