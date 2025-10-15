#!/bin/bash
set -e

echo "=========================================="
echo "Testing ROS Jazzy Extension"
echo "=========================================="

# Test that ROS environment is available
echo "1. Testing ROS installation..."
if ! command -v ros2 &> /dev/null; then
    echo "ERROR: ros2 command not found"
    exit 1
fi
echo "✓ ros2 command available"

# Test ROS environment variables
echo ""
echo "2. Testing ROS environment variables..."
if [ -z "$ROS_DISTRO" ]; then
    echo "ERROR: ROS_DISTRO not set"
    exit 1
fi

if [ "$ROS_DISTRO" != "jazzy" ]; then
    echo "ERROR: Expected ROS_DISTRO=jazzy, got $ROS_DISTRO"
    exit 1
fi
echo "✓ ROS_DISTRO=$ROS_DISTRO"

# Test additional required environment variables
for var in AMENT_PREFIX_PATH COLCON_PREFIX_PATH ROS_VERSION ROS_PYTHON_VERSION; do
    if [ -z "${!var}" ]; then
        echo "ERROR: $var not set"
        exit 1
    fi
    echo "✓ $var=${!var}"
done

# Test that workspace directories exist
echo ""
echo "3. Testing workspace structure..."
for dir in ROS_WORKSPACE_ROOT ROS_UNDERLAY_PATH ROS_UNDERLAY_BUILD ROS_UNDERLAY_INSTALL; do
    if [ ! -d "${!dir}" ]; then
        echo "ERROR: $dir not found at ${!dir}"
        exit 1
    fi
    echo "✓ $dir=${!dir}"
done

# Test that colcon is available
echo ""
echo "4. Testing build tools..."
if ! command -v colcon &> /dev/null; then
    echo "ERROR: colcon command not found"
    exit 1
fi
echo "✓ colcon command available"

# Test that vcstool is available
if ! command -v vcs &> /dev/null; then
    echo "ERROR: vcs command not found"
    exit 1
fi
echo "✓ vcstool command available"

# Test that rosdep is available
if ! command -v rosdep &> /dev/null; then
    echo "ERROR: rosdep command not found"
    exit 1
fi
echo "✓ rosdep command available"

# Source ROS environment
echo ""
echo "5. Testing ROS environment sourcing..."
source /opt/ros/jazzy/setup.bash
echo "✓ Successfully sourced /opt/ros/jazzy/setup.bash"

# Test basic ROS functionality
echo ""
echo "6. Testing basic ROS functionality..."
if ! ros2 --help > /dev/null 2>&1; then
    echo "ERROR: ros2 --help failed"
    exit 1
fi
echo "✓ ros2 --help working"

# Test ros2 pkg list
if ! ros2 pkg list > /dev/null 2>&1; then
    echo "ERROR: ros2 pkg list failed"
    exit 1
fi
echo "✓ ros2 pkg list working"

# Check if underlay was built
echo ""
echo "7. Testing underlay workspace..."
if [ -f "${ROS_UNDERLAY_INSTALL}/setup.bash" ]; then
    echo "✓ Underlay install found at ${ROS_UNDERLAY_INSTALL}"

    # Test underlay can be sourced
    if ! source "${ROS_UNDERLAY_INSTALL}/setup.bash" 2>&1; then
        echo "ERROR: Failed to source underlay"
        exit 1
    fi
    echo "✓ Successfully sourced underlay"

    # Test that underlay packages are available
    echo ""
    echo "8. Testing underlay package availability..."
    if ! ros2 pkg list | grep -q unique_identifier_msgs; then
        echo "ERROR: unique_identifier_msgs not found in package list"
        echo "Available packages:"
        ros2 pkg list | head -20
        exit 1
    fi
    echo "✓ Underlay package unique_identifier_msgs found"

    # Test workspace chaining
    echo ""
    echo "9. Testing workspace chaining and package build..."

    # Create a test package in the main workspace that depends on underlay
    cd "$ROS_WORKSPACE_ROOT"
    mkdir -p src/test_package
    cp /tmp/test_package.xml src/test_package/package.xml
    cp /tmp/test_setup.py src/test_package/setup.py
    mkdir -p src/test_package/resource
    touch src/test_package/resource/test_package
    mkdir -p src/test_package/test_package
    touch src/test_package/test_package/__init__.py
    echo "✓ Created test package"

    # Source both ROS and underlay
    source /opt/ros/jazzy/setup.bash
    source "${ROS_UNDERLAY_INSTALL}/setup.bash"

    # Verify dependency is available before building
    if ! ros2 pkg list | grep -q unique_identifier_msgs; then
        echo "ERROR: unique_identifier_msgs not available after sourcing"
        exit 1
    fi
    echo "✓ Underlay dependency available for build"

    # Test building the main workspace
    echo ""
    echo "10. Building test package that depends on underlay..."
    if ! colcon build --packages-select test_package 2>&1 | tee /tmp/build_output.log; then
        echo "ERROR: Failed to build test package"
        cat /tmp/build_output.log
        exit 1
    fi
    echo "✓ Successfully built package depending on underlay"

    # Test that we can source the built workspace
    if [ ! -f install/setup.bash ]; then
        echo "ERROR: Built workspace setup.bash not found"
        exit 1
    fi

    source install/setup.bash
    echo "✓ Successfully sourced built workspace"

    # Verify the test package is now in the package list
    if ! ros2 pkg list | grep -q test_package; then
        echo "ERROR: test_package not found in package list after build"
        exit 1
    fi
    echo "✓ Built package is available in ROS environment"

    # Test that we can still see underlay packages
    if ! ros2 pkg list | grep -q unique_identifier_msgs; then
        echo "ERROR: Lost visibility to underlay packages after main workspace source"
        exit 1
    fi
    echo "✓ Workspace chaining working correctly"

else
    echo "⚠ No underlay install found - skipping underlay tests"
    echo "  This is expected if no depends.repos was provided"
fi

# Test permissions
echo ""
echo "11. Testing file permissions..."
if [ ! -r "${ROS_UNDERLAY_BUILD}" ] || [ ! -r "${ROS_UNDERLAY_INSTALL}" ]; then
    echo "ERROR: Underlay directories not readable"
    exit 1
fi
echo "✓ Underlay directories have correct permissions"

echo ""
echo "=========================================="
echo "✓ All ROS Jazzy tests passed successfully!"
echo "=========================================="
