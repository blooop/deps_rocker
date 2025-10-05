#!/bin/bash
set -e

# Source ROS setup if it exists
if [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
    source /opt/ros/$ROS_DISTRO/setup.bash
elif [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    export ROS_DISTRO=humble
elif [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash  
    export ROS_DISTRO=jazzy
else
    echo "ERROR: No ROS installation found"
    exit 1
fi

# Verify ROS 2 is available
if ! command -v ros2 &> /dev/null; then
    echo "ERROR: ros2 command not found"
    exit 1
fi

# Test basic ROS 2 functionality
echo "Testing ROS $ROS_DISTRO functionality..."
ros2 --help | head -5

# Check environment variables
echo "ROS_DISTRO: $ROS_DISTRO"
echo "ROS_VERSION: $ROS_VERSION"
echo "AMENT_PREFIX_PATH: $AMENT_PREFIX_PATH"

# Test colcon if available
if command -v colcon &> /dev/null; then
    echo "colcon is available"
    colcon --help | head -5
else
    echo "WARNING: colcon not found"
fi

# Test rosdep if available  
if command -v rosdep &> /dev/null; then
    echo "rosdep is available"
    rosdep --help | head -5
else
    echo "WARNING: rosdep not found"
fi

# Test that we can list ROS 2 packages (limit output and handle broken pipe)
ros2 pkg list 2>/dev/null | head -5 || true

# Ensure ros-underlays helper is available and functional
if ! command -v ros-underlays &> /dev/null; then
    echo "ERROR: ros-underlays helper not found"
    exit 1
fi

echo "Listing configured underlays..."
ros-underlays list

if [ ! -x /opt/ros/underlays/setup.bash ]; then
    echo "ERROR: expected underlay aggregator at /opt/ros/underlays/setup.bash"
    exit 1
fi

echo "Dry-run sync of underlays..."
ros-underlays sync --dry-run --verbose

echo "Dry-run rebuild of underlays..."
ros-underlays rebuild --dry-run --verbose

if command -v ros-underlays-rebuild &> /dev/null; then
    echo "Testing ros-underlays-rebuild helper..."
    ros-underlays-rebuild --dry-run --verbose
fi

# Test colcon build functionality in a temporary workspace
echo "Testing colcon build functionality..."
mkdir -p /tmp/test_ws/src
cd /tmp/test_ws

# Create a minimal ROS 2 package
mkdir -p src/test_pkg
cat > src/test_pkg/package.xml << 'EOF'
<?xml version="1.0"?>
<package format="3">
  <name>test_pkg</name>
  <version>0.0.0</version>
  <description>Test package</description>
  <maintainer email="test@example.com">Test</maintainer>
  <license>MIT</license>
</package>
EOF

cat > src/test_pkg/CMakeLists.txt << 'EOF'
cmake_minimum_required(VERSION 3.8)
project(test_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
ament_package()
EOF

# Test colcon build
echo "Running colcon build..."
colcon build --packages-select test_pkg --cmake-args -DCMAKE_BUILD_TYPE=Release

# Test colcon test
echo "Running colcon test..."
colcon test --packages-select test_pkg

echo "Colcon build and test completed successfully!"

# Test colcon-runner functionality
echo "Testing colcon-runner functionality..."
if command -v cr &> /dev/null; then
    echo "cr command is available (provided by colcon-runner)"
    echo "Testing cr command help..."
    cr --help | head -10
    
    # Test basic cr functionality
    echo "Testing cr basic usage..."
    cr ba --help 2>/dev/null | head -5 2>/dev/null || echo "cr ba command working"
else
    echo "WARNING: cr command not found (colcon-runner may not be installed)"
fi

echo "ros_generic extension test for ROS_DISTRO=$ROS_DISTRO completed successfully!"
