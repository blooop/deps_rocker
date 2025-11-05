#!/bin/bash
set -e

echo "=========================================="
echo "Testing ROS Jazzy Extension"
echo "=========================================="

# Function to test basic ROS installation
test_ros_installation() {
    echo "1. Testing ROS installation..."
    if ! command -v ros2 &> /dev/null; then
        echo "ERROR: ros2 command not found"
        cd "$ROS_WORKSPACE_ROOT"
    
        # Use the pre-existing test/test_package for build and test
        echo "✓ Using test/test_package for workspace chaining and build"
        # Ensure the test package is present
        if [ ! -d "test/test_package" ]; then
            echo "ERROR: test/test_package directory not found"
            exit 1
        fi
        # Symlink test/test_package into the workspace src/ if needed
        mkdir -p src
        if [ ! -e src/test_package ]; then
            ln -s ../../test/test_package src/test_package
            echo "✓ Symlinked test/test_package into src/test_package"
        fi
    # Test each workspace directory exists
    test_dir_exists() {
        local var_name="$1"
        local dir_value="${!var_name}"
        
        # Handle Docker ENV variables that may not be expanded properly
        # During Docker build, $HOME may not expand correctly in ENV statements
        # So we need to manually substitute it at test runtime
        local expanded_path="${dir_value}"
        if [[ "${expanded_path}" == '$HOME'* ]]; then
            expanded_path="${expanded_path/\$HOME/${HOME}}"
        fi
        
        if [ ! -d "${expanded_path}" ]; then
            echo "ERROR: $var_name not found at ${expanded_path} (from ${dir_value})"
            exit 1
        fi
        echo "✓ $var_name=${expanded_path}"
    }
    
    test_dir_exists ROS_WORKSPACE_ROOT
    test_dir_exists ROS_UNDERLAY_PATH 
    test_dir_exists ROS_UNDERLAY_BUILD
    test_dir_exists ROS_UNDERLAY_INSTALL
}

# Function to test build tools availability
test_build_tools() {
    echo ""
    echo "4. Testing build tools..."
    if ! command -v colcon &> /dev/null; then
        echo "ERROR: colcon command not found"
        exit 1
    fi
    echo "✓ colcon command available"

    if ! command -v ros2 &> /dev/null; then
    if ! command -v rosdep &> /dev/null; then
        echo "ERROR: rosdep command not found"
        exit 1
    fi
fi
    echo "✓ rosdep command available"
}

# Function to test ROS environment sourcing and basic functionality
test_ros_functionality() {
    echo ""
    echo "5. Testing ROS environment sourcing..."
    source /opt/ros/jazzy/setup.bash
    echo "✓ Successfully sourced /opt/ros/jazzy/setup.bash"

    echo ""
    echo "6. Testing basic ROS functionality..."
    if ! ros2 --help > /dev/null 2>&1; then
        echo "ERROR: ros2 --help failed"
        exit 1
    fi
    echo "✓ ros2 --help working"

    if ! ros2 pkg list > /dev/null 2>&1; then
        echo "ERROR: ros2 pkg list failed"
        exit 1
    fi
    echo "✓ ros2 pkg list working"
}

# Function to test underlay workspace
test_underlay_workspace() {
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
        test_underlay_packages

        # Test workspace chaining and building
        test_workspace_chaining
    else
        echo "⚠ No underlay install found - skipping underlay tests"
        echo "  This is expected if no depends.repos was provided"
    fi
}

# Function to test underlay package availability
test_underlay_packages() {
    echo ""
    echo "8. Testing underlay package availability..."
    if ! ros2 pkg list | grep -q unique_identifier_msgs; then
        echo "ERROR: unique_identifier_msgs not found in package list"
        echo "Available packages:"
        ros2 pkg list | head -20
        exit 1
    fi
    echo "✓ Underlay package unique_identifier_msgs found"
}

# Function to test underlay build process
test_underlay_build_process() {
    echo ""
    echo "8.1. Testing underlay build process..."
    
    # Test that underlay scripts work
    if [ ! -x /usr/local/bin/underlay_deps.sh ]; then
        echo "ERROR: underlay_deps.sh not found or not executable"
        exit 1
    fi
    echo "✓ underlay_deps.sh script is available and executable"

    if [ ! -x /usr/local/bin/underlay_build.sh ]; then
        echo "ERROR: underlay_build.sh not found or not executable"
        exit 1
    fi
    echo "✓ underlay_build.sh script is available and executable"

    # Test that underlay can be rebuilt if needed
    if [ -d "${ROS_UNDERLAY_PATH}" ] && [ -n "$(find "${ROS_UNDERLAY_PATH}" -name 'package.xml' -print -quit)" ]; then
        echo "Testing underlay rebuild capability..."
        
        # Clean previous build to test fresh build
        if [ -d "${ROS_UNDERLAY_BUILD}" ]; then
            rm -rf "${ROS_UNDERLAY_BUILD:?}/"*
        fi
        if [ -d "${ROS_UNDERLAY_INSTALL}" ]; then
            rm -rf "${ROS_UNDERLAY_INSTALL:?}/"*
        fi
        
        # Test dependency installation
        echo "Testing rosdep dependency installation..."
        underlay_deps.sh
        echo "✓ Underlay dependencies installed successfully"
        
        # Test build process
        echo "Testing underlay build process..."
        underlay_build.sh
        echo "✓ Underlay build completed successfully"
        
        # Verify build artifacts exist
        NEW_INSTALL_COUNT=$(find "${ROS_UNDERLAY_INSTALL}" -type f -name "*.cmake" | wc -l)
        if [ "${NEW_INSTALL_COUNT}" -eq 0 ]; then
            echo "ERROR: No build artifacts found after underlay build"
            exit 1
        fi
        echo "✓ Underlay build artifacts present (${NEW_INSTALL_COUNT} cmake files)"
    else
        echo "⚠ No underlay packages to test build process"
    fi
}

# Function to test rosdep functionality 
test_rosdep_functionality() {
    echo ""
    echo "8.2. Testing rosdep functionality..."
    
    # Test rosdep update works
    if ! rosdep update > /dev/null 2>&1; then
        echo "ERROR: rosdep update failed"
        exit 1
    fi
    echo "✓ rosdep update successful"
    
    # Test rosdep can resolve dependencies
    if [ -d "${ROS_UNDERLAY_PATH}" ] && [ -n "$(find "${ROS_UNDERLAY_PATH}" -name 'package.xml' -print -quit)" ]; then
        echo "Testing rosdep dependency resolution..."
        if ! rosdep check --from-paths "${ROS_UNDERLAY_PATH}" --ignore-src > /dev/null 2>&1; then
            echo "⚠ Some rosdep dependencies may not be satisfied (this might be expected)"
        else
            echo "✓ All rosdep dependencies satisfied"
        fi
        # Remove any duplicate test_package at root if present
        if [ -d "test_package" ]; then
            echo "Cleaning up duplicate root test_package directory..."
            rm -rf test_package
            echo "✓ Duplicate root test_package removed."
        fi
        
        # Test rosdep install (dry-run to avoid actually installing)
        echo "Testing rosdep install capability..."
        if rosdep install --from-paths "${ROS_UNDERLAY_PATH}" --ignore-src -y -r --simulate > /dev/null 2>&1; then
            echo "✓ rosdep install simulation successful"
        else
            echo "⚠ rosdep install simulation had issues (might be expected)"
        fi
    else
        echo "⚠ No underlay packages to test rosdep against"
    fi
}

# Function to test workspace chaining and package building
test_workspace_chaining() {
    echo ""
    echo "9. Testing workspace chaining and package build..."

    # Create a test package in the main workspace that depends on underlay
    cd "$ROS_WORKSPACE_ROOT"
    
    # Ensure we have write permissions to the workspace
    if [ ! -w "$ROS_WORKSPACE_ROOT" ]; then
        echo "WARNING: No write permission to workspace root $ROS_WORKSPACE_ROOT, attempting to fix..."
        # Try to fix permissions (with multiple approaches)
        if sudo chown -R "$(whoami):$(whoami)" "$ROS_WORKSPACE_ROOT" 2>/dev/null; then
            echo "✓ Fixed permissions for $ROS_WORKSPACE_ROOT using sudo"
        elif chmod -R u+w "$ROS_WORKSPACE_ROOT" 2>/dev/null; then
            echo "✓ Fixed permissions for $ROS_WORKSPACE_ROOT using chmod"
        else
            echo "ERROR: Cannot fix permissions for workspace root $ROS_WORKSPACE_ROOT"
            echo "Directory info:"
            ls -la "$ROS_WORKSPACE_ROOT" || echo "Directory listing failed"
            echo "Current user: $(whoami) ($(id))"
            exit 1
        fi
    fi
    
    # Use the pre-existing test/test_package for build and test
    echo "✓ Using test/test_package for workspace chaining and build"
    # Ensure the test package is present
    if [ ! -d "test/test_package" ]; then
        echo "ERROR: test/test_package directory not found"
        exit 1
    fi
    # Symlink test/test_package into the workspace src/ if needed
    mkdir -p src
    if [ ! -e src/test_package ]; then
        ln -s ../../test/test_package src/test_package
        echo "✓ Symlinked test/test_package into src/test_package"
    fi

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
    if ! colcon build --packages-select test_package --event-handlers console_direct+ 2>&1 | tee /tmp/build_output.log; then
        echo "ERROR: Failed to build test package"
        cat /tmp/build_output.log
        exit 1
    fi
    echo "✓ Successfully built package depending on underlay"

    # Cleanup: remove src/test_package to prevent dirty duplicates before any further tests
    if [ -d "src/test_package" ]; then
        echo "Cleaning up src/test_package to prevent duplicate package errors..."
        rm -rf src/test_package
        echo "✓ src/test_package removed."
    fi

    # Test that we can source the built workspace
    if [ ! -f "${ROS_INSTALL_BASE}/setup.bash" ]; then
        echo "ERROR: Built workspace setup.bash not found at ${ROS_INSTALL_BASE}/setup.bash"
        ls -la "${ROS_INSTALL_BASE}/" || echo "Install directory does not exist"
        exit 1
    fi

    source "${ROS_INSTALL_BASE}/setup.bash"
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
}

# Function to test file permissions
test_file_permissions() {
    echo ""
    echo "11. Testing file permissions..."

    # Test basic directory readability
    if [ ! -r "${ROS_UNDERLAY_BUILD}" ] || [ ! -x "${ROS_UNDERLAY_BUILD}" ]; then
        echo "ERROR: Underlay directory ${ROS_UNDERLAY_BUILD} is not readable/executable"
        exit 1
    fi
    if [ ! -r "${ROS_UNDERLAY_INSTALL}" ] || [ ! -x "${ROS_UNDERLAY_INSTALL}" ]; then
        echo "ERROR: Underlay directory ${ROS_UNDERLAY_INSTALL} is not readable/executable"
        exit 1
    fi
    echo "✓ Underlay directories are readable"
    echo "✓ File permissions test completed"
}



# Main test execution
main() {
    # Source bashrc to get proper environment variable definitions
    source /etc/bash.bashrc || true
    
    test_ros_installation
    test_ros_environment
    test_workspace_structure
    test_build_tools
    test_ros_functionality
    test_underlay_workspace
    test_underlay_build_process
    test_rosdep_functionality
    test_file_permissions

    echo ""
    echo "=========================================="
    echo "✓ All ROS Jazzy tests passed successfully!"
    echo "=========================================="

    # Cleanup: remove src/test_package to prevent dirty duplicates
    if [ -d "src/test_package" ]; then
        echo "Cleaning up src/test_package to prevent duplicate package errors..."
        rm -rf src/test_package
        echo "✓ src/test_package removed."
    fi
    # Cleanup: remove duplicate root test_package if present
    if [ -d "test_package" ]; then
        echo "Cleaning up duplicate root test_package directory..."
        rm -rf test_package
        echo "✓ Duplicate root test_package removed."
    fi
}

# Run main function
main
