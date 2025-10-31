#!/bin/bash
set -e

# Comprehensive ROS Jazzy Extension Test Suite
# This test suite validates all features described in the README specification

echo "=========================================="
echo "ROS Jazzy Extension Comprehensive Test Suite"
echo "=========================================="

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Logging functions
log_info() {
    echo -e "${GREEN}✓${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}⚠${NC} $1"
}

log_error() {
    echo -e "${RED}✗${NC} $1"
    exit 1
}

log_section() {
    echo ""
    echo "=========================================="
    echo "$1"
    echo "=========================================="
}

# Test counters
TESTS_RUN=0
TESTS_PASSED=0

run_test() {
    local test_name="$1"
    local test_function="$2"
    
    echo ""
    echo "Testing: $test_name"
    echo "----------------------------------------"
    
    TESTS_RUN=$((TESTS_RUN + 1))
    
    if $test_function; then
        TESTS_PASSED=$((TESTS_PASSED + 1))
        log_info "$test_name: PASSED"
    else
        log_error "$test_name: FAILED"
    fi
}

# =============================================================================
# 1. BASE ROS INSTALLATION TESTS
# =============================================================================

test_ros_commands_available() {
    # Test ROS 2 command availability
    command -v ros2 >/dev/null 2>&1 || { log_error "ros2 command not found"; return 1; }
    log_info "ros2 command available"
    
    command -v colcon >/dev/null 2>&1 || { log_error "colcon command not found"; return 1; }
    log_info "colcon command available"
    
    command -v rosdep >/dev/null 2>&1 || { log_error "rosdep command not found"; return 1; }
    log_info "rosdep command available"
    
    command -v vcs >/dev/null 2>&1 || { log_error "vcs command not found"; return 1; }
    log_info "vcs command available"
    
    return 0
}

test_ros_base_environment_variables() {
    # Test basic ROS environment variables from specification
    [ "$ROS_DISTRO" = "jazzy" ] || { log_error "Expected ROS_DISTRO=jazzy, got: $ROS_DISTRO"; return 1; }
    log_info "ROS_DISTRO=$ROS_DISTRO"
    
    [ -n "$ROS_VERSION" ] || { log_error "ROS_VERSION not set"; return 1; }
    log_info "ROS_VERSION=$ROS_VERSION"
    
    [ -n "$ROS_PYTHON_VERSION" ] || { log_error "ROS_PYTHON_VERSION not set"; return 1; }
    log_info "ROS_PYTHON_VERSION=$ROS_PYTHON_VERSION"
    
    [ -n "$AMENT_PREFIX_PATH" ] || { log_error "AMENT_PREFIX_PATH not set"; return 1; }
    log_info "AMENT_PREFIX_PATH set"
    
    return 0
}

test_ros_functionality() {
    # Test basic ROS functionality
    source /opt/ros/jazzy/setup.bash || { log_error "Failed to source ROS setup"; return 1; }
    log_info "Successfully sourced ROS setup"
    
    ros2 --help >/dev/null 2>&1 || { log_error "ros2 --help failed"; return 1; }
    log_info "ros2 --help working"
    
    ros2 pkg list >/dev/null 2>&1 || { log_error "ros2 pkg list failed"; return 1; }
    log_info "ros2 pkg list working"
    
    return 0
}

# =============================================================================
# 2. ENVIRONMENT VARIABLE SPECIFICATION TESTS  
# =============================================================================

test_workspace_environment_variables() {
    # Test all workspace environment variables match README specification
    
    # Underlay workspace variables (specification)
    local expected_underlay_root="$HOME/underlay"
    local expected_underlay_path="$HOME/underlay/src"
    local expected_underlay_build="$HOME/underlay/build" 
    local expected_underlay_install="$HOME/underlay/install"
    
    # Overlay workspace variables (specification)
    local expected_overlay_root="$HOME/overlay"
    local expected_workspace_root="$HOME/overlay"  # Main workspace points to overlay
    local expected_build_base="$HOME/overlay/build"
    local expected_install_base="$HOME/overlay/install"
    local expected_log_base="$HOME/overlay/log"
    
    # Test underlay environment variables
    [ "$ROS_UNDERLAY_ROOT" = "$expected_underlay_root" ] || { log_error "ROS_UNDERLAY_ROOT mismatch. Expected: $expected_underlay_root, Got: $ROS_UNDERLAY_ROOT"; return 1; }
    log_info "ROS_UNDERLAY_ROOT=$ROS_UNDERLAY_ROOT"
    
    [ "$ROS_UNDERLAY_PATH" = "$expected_underlay_path" ] || { log_error "ROS_UNDERLAY_PATH mismatch. Expected: $expected_underlay_path, Got: $ROS_UNDERLAY_PATH"; return 1; }
    log_info "ROS_UNDERLAY_PATH=$ROS_UNDERLAY_PATH"
    
    [ "$ROS_UNDERLAY_BUILD" = "$expected_underlay_build" ] || { log_error "ROS_UNDERLAY_BUILD mismatch. Expected: $expected_underlay_build, Got: $ROS_UNDERLAY_BUILD"; return 1; }
    log_info "ROS_UNDERLAY_BUILD=$ROS_UNDERLAY_BUILD"
    
    [ "$ROS_UNDERLAY_INSTALL" = "$expected_underlay_install" ] || { log_error "ROS_UNDERLAY_INSTALL mismatch. Expected: $expected_underlay_install, Got: $ROS_UNDERLAY_INSTALL"; return 1; }
    log_info "ROS_UNDERLAY_INSTALL=$ROS_UNDERLAY_INSTALL"
    
    # Test overlay environment variables
    [ "$ROS_OVERLAY_ROOT" = "$expected_overlay_root" ] || { log_error "ROS_OVERLAY_ROOT mismatch. Expected: $expected_overlay_root, Got: $ROS_OVERLAY_ROOT"; return 1; }
    log_info "ROS_OVERLAY_ROOT=$ROS_OVERLAY_ROOT"
    
    [ "$ROS_WORKSPACE_ROOT" = "$expected_workspace_root" ] || { log_error "ROS_WORKSPACE_ROOT mismatch. Expected: $expected_workspace_root, Got: $ROS_WORKSPACE_ROOT"; return 1; }
    log_info "ROS_WORKSPACE_ROOT=$ROS_WORKSPACE_ROOT"
    
    [ "$ROS_BUILD_BASE" = "$expected_build_base" ] || { log_error "ROS_BUILD_BASE mismatch. Expected: $expected_build_base, Got: $ROS_BUILD_BASE"; return 1; }
    log_info "ROS_BUILD_BASE=$ROS_BUILD_BASE"
    
    [ "$ROS_INSTALL_BASE" = "$expected_install_base" ] || { log_error "ROS_INSTALL_BASE mismatch. Expected: $expected_install_base, Got: $ROS_INSTALL_BASE"; return 1; }
    log_info "ROS_INSTALL_BASE=$ROS_INSTALL_BASE"
    
    [ "$ROS_LOG_BASE" = "$expected_log_base" ] || { log_error "ROS_LOG_BASE mismatch. Expected: $expected_log_base, Got: $ROS_LOG_BASE"; return 1; }
    log_info "ROS_LOG_BASE=$ROS_LOG_BASE"
    
    return 0
}

# =============================================================================
# 3. WORKSPACE STRUCTURE TESTS
# =============================================================================

test_underlay_workspace_structure() {
    # Test underlay workspace has correct structure per specification
    local underlay_dirs=(
        "$HOME/underlay"
        "$HOME/underlay/src"
        "$HOME/underlay/build"
        "$HOME/underlay/install"
        "$HOME/underlay/log"
    )
    
    for dir in "${underlay_dirs[@]}"; do
        [ -d "$dir" ] || { log_error "Underlay directory missing: $dir"; return 1; }
        log_info "Underlay directory exists: $dir"
    done
    
    return 0
}

test_overlay_workspace_structure() {
    # Test overlay workspace has correct structure per specification
    local overlay_dirs=(
        "$HOME/overlay"
        "$HOME/overlay/src"
        "$HOME/overlay/build"
        "$HOME/overlay/install"
        "$HOME/overlay/log"
    )
    
    for dir in "${overlay_dirs[@]}"; do
        [ -d "$dir" ] || { log_error "Overlay directory missing: $dir"; return 1; }
        log_info "Overlay directory exists: $dir"
    done
    
    return 0
}

test_workspace_permissions() {
    # Test that both workspaces are in user home with correct ownership
    local current_user=$(whoami)
    
    # Test underlay permissions
    if [ -d "$HOME/underlay" ]; then
        [ -r "$HOME/underlay" ] && [ -x "$HOME/underlay" ] || { log_error "Underlay not readable/executable"; return 1; }
        log_info "Underlay workspace accessible"
    fi
    
    # Test overlay permissions
    if [ -d "$HOME/overlay" ]; then
        [ -r "$HOME/overlay" ] && [ -x "$HOME/overlay" ] || { log_error "Overlay not readable/executable"; return 1; }
        log_info "Overlay workspace accessible"
    fi
    
    return 0
}

# =============================================================================
# 4. UNIFIED SCRIPT TESTS
# =============================================================================

test_unified_scripts_available() {
    # Test all unified scripts are available and executable per specification
    local scripts=(
        "rosdep_underlay.sh"
        "rosdep_overlay.sh" 
        "build_underlay.sh"
        "update_repos.sh"
    )
    
    for script in "${scripts[@]}"; do
        if command -v "$script" >/dev/null 2>&1; then
            log_info "Script available: $script"
        else
            log_error "Script not found or not executable: $script"
            return 1
        fi
    done
    
    return 0
}

# =============================================================================
# MAIN TEST EXECUTION (Simplified for initial implementation)
# =============================================================================

main() {
    echo "Starting comprehensive ROS Jazzy extension test suite..."
    echo "Testing against specification requirements..."
    echo ""
    
    # Source environment to get proper variable definitions
    source /etc/bash.bashrc 2>/dev/null || true
    
    # Category 1: Base ROS Installation Tests
    log_section "1. BASE ROS INSTALLATION TESTS"
    run_test "ROS Commands Available" test_ros_commands_available
    run_test "ROS Base Environment Variables" test_ros_base_environment_variables  
    run_test "ROS Functionality" test_ros_functionality
    
    # Category 2: Environment Variable Specification Tests
    log_section "2. ENVIRONMENT VARIABLE SPECIFICATION TESTS"
    run_test "Workspace Environment Variables" test_workspace_environment_variables
    
    # Category 3: Workspace Structure Tests
    log_section "3. WORKSPACE STRUCTURE TESTS" 
    run_test "Underlay Workspace Structure" test_underlay_workspace_structure
    run_test "Overlay Workspace Structure" test_overlay_workspace_structure
    run_test "Workspace Permissions" test_workspace_permissions
    
    # Category 4: Unified Script Tests
    log_section "4. UNIFIED SCRIPT TESTS"
    run_test "Unified Scripts Available" test_unified_scripts_available
    
    # Final results
    log_section "TEST RESULTS SUMMARY"
    echo "Tests Run: $TESTS_RUN"
    echo "Tests Passed: $TESTS_PASSED"
    echo "Tests Failed: $((TESTS_RUN - TESTS_PASSED))"
    
    if [ "$TESTS_PASSED" -eq "$TESTS_RUN" ]; then
        echo ""
        log_info "🎉 ALL TESTS PASSED! ROS Jazzy extension meets specification requirements."
        echo ""
    else
        echo ""
        log_error "❌ Some tests failed. Implementation needs updates to meet specification."
        echo ""
        exit 1
    fi
}

# Run the comprehensive test suite
main "$@"