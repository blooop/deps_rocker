#!/bin/bash
set -e

echo "Building ROS underlay from vcstool repositories..."

ROS_DISTRO_NAME="${ROS_DISTRO:-jazzy}"
ROS_SETUP="/opt/ros/${ROS_DISTRO_NAME}/setup.bash"

if [ -f "$ROS_SETUP" ]; then
    # shellcheck disable=SC1090
    source "$ROS_SETUP"
else
    echo "ROS setup script not found at $ROS_SETUP, skipping underlay build"
    exit 0
fi

ROS_WORKSPACE_ROOT="${ROS_WORKSPACE_ROOT:-/workspaces/ros_ws}"
ROS_REPOS_ROOT="${ROS_REPOS_ROOT:-${ROS_WORKSPACE_ROOT}/repos}"
ROS_DEPENDENCIES_ROOT="${ROS_DEPENDENCIES_ROOT:-${ROS_WORKSPACE_ROOT}/src}"
ROS_UNDERLAY_PATH="${ROS_UNDERLAY_PATH:-${ROS_WORKSPACE_ROOT}/underlay}"
ROS_BUILD_BASE="${ROS_BUILD_BASE:-${ROS_WORKSPACE_ROOT}/build}"
ROS_LOG_BASE="${ROS_LOG_BASE:-${ROS_WORKSPACE_ROOT}/log}"

ensure_dir_writable() {
    local dir="$1"
    if [ ! -d "$dir" ]; then
        echo "Creating directory $dir"
        mkdir -p "$dir" || {
            echo "ERROR: Unable to create $dir" >&2
            exit 1
        }
    fi

    if [ ! -w "$dir" ]; then
        echo "Adjusting permissions for $dir"
        if command -v sudo >/dev/null 2>&1; then
            sudo chmod -R a+rwX "$dir" || {
                echo "ERROR: Unable to change permissions for $dir" >&2
                exit 1
            }
        else
            chmod -R a+rwX "$dir" || {
                echo "ERROR: Unable to change permissions for $dir" >&2
                exit 1
            }
        fi
    fi
}

# Gather repository paths based on discovered manifests
repos_paths=()
if [ -d "$ROS_REPOS_ROOT" ]; then
    while IFS= read -r -d '' repos_file; do
        relative_dir="${repos_file#$ROS_REPOS_ROOT/}"
        parent_dir="$(dirname "$relative_dir")"
        target_dir="$ROS_DEPENDENCIES_ROOT/$parent_dir"

        if [ -d "$target_dir" ] && \
            [ "$(find "$target_dir" -mindepth 1 -maxdepth 1 -type d 2>/dev/null | wc -l)" -gt 0 ]; then
            repos_paths+=("$target_dir")
        fi
    done < <(find "$ROS_REPOS_ROOT" \( -name "*.repos" -o -name "depends.repos.yaml" \) -type f -print0)
fi

# Fall back to the dependencies root if manifests are missing but packages exist
if [ ${#repos_paths[@]} -eq 0 ] && [ -d "$ROS_DEPENDENCIES_ROOT" ]; then
    if [ "$(find "$ROS_DEPENDENCIES_ROOT" -mindepth 1 -maxdepth 1 -type d 2>/dev/null | wc -l)" -gt 0 ]; then
        repos_paths=("$ROS_DEPENDENCIES_ROOT")
    fi
fi

if [ ${#repos_paths[@]} -eq 0 ]; then
    echo "No packages found from discovered manifests, skipping underlay build"
    mkdir -p "$ROS_UNDERLAY_PATH"
    exit 0
fi

echo "Found ${#repos_paths[@]} repository path(s) to build"

echo "Updating rosdep..."
rosdep update || true

echo "Installing dependencies with rosdep..."
for path in "${repos_paths[@]}"; do
    echo "  Installing dependencies from: $path"
    rosdep install --from-paths "$path" --ignore-src -y || true
done

ensure_dir_writable "$ROS_UNDERLAY_PATH"
ensure_dir_writable "$ROS_BUILD_BASE"
ensure_dir_writable "$ROS_LOG_BASE"

echo "Building packages with colcon..."
for path in "${repos_paths[@]}"; do
    echo "  Building packages from: $path"
    cd "$path"
    colcon --log-base "$ROS_LOG_BASE" build --install-base "$ROS_UNDERLAY_PATH" --merge-install --build-base "$ROS_BUILD_BASE"
done

echo "ROS underlay build complete!"
echo "Underlay installed to: $ROS_UNDERLAY_PATH"
echo "Source it with: source $ROS_UNDERLAY_PATH/setup.bash"
