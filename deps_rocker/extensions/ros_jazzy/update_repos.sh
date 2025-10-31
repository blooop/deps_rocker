#!/bin/bash
# Unified script to update repositories dynamically
# Part of the ROS Unified Workspace Architecture

set -e

# Use environment variables for unified workspace architecture
UNDERLAY_SRC="${ROS_UNDERLAY_PATH:-$HOME/underlay/src}"

echo "Updating repositories in underlay workspace: $UNDERLAY_SRC"

# Function to import repos from a file
import_repos() {
    local repos_file="$1"
    local target_dir="$2"
    
    if [ -f "$repos_file" ] && [ -s "$repos_file" ]; then
        echo "Importing repositories from: $repos_file"
        mkdir -p "$target_dir"
        vcs import --recursive "$target_dir" < "$repos_file"
        echo "Successfully imported repositories from $repos_file"
    else
        echo "Repository file not found or empty: $repos_file"
    fi
}

# Look for repos files in common locations
REPOS_FILES=()

# Check current directory for repos files
for pattern in "*.repos" "repos" "*.rosinstall" ".rosinstall"; do
    for file in $pattern; do
        if [ -f "$file" ]; then
            REPOS_FILES+=("$(realpath "$file")")
        fi
    done
done

# If no repos files found, try to use consolidated.repos if available
if [ ${#REPOS_FILES[@]} -eq 0 ] && [ -f "/tmp/consolidated.repos" ] && [ -s "/tmp/consolidated.repos" ]; then
    # Check if consolidated.repos actually has repositories (not just empty structure)
    # Look for non-empty repositories section
    if grep -q "repositories:" "/tmp/consolidated.repos"; then
        # Check if there are actual repositories (not just empty {})
        if ! grep -A 1 "repositories:" "/tmp/consolidated.repos" | grep -q "{}"; then
            # Further check: look for actual repository entries (lines that don't start with # and have content after repositories:)
            if sed -n '/^repositories:/,$p' "/tmp/consolidated.repos" | grep -v "^#" | grep -v "^repositories:" | grep -v "^[[:space:]]*$" | grep -q .; then
                REPOS_FILES+=("/tmp/consolidated.repos")
                echo "Using consolidated.repos with detected repositories"
            else
                echo "consolidated.repos contains empty repositories section"
            fi
        else
            echo "consolidated.repos contains empty repositories: {}"
        fi
    else
        echo "consolidated.repos does not contain repositories section"
    fi
fi

# Import repositories
if [ ${#REPOS_FILES[@]} -eq 0 ]; then
    echo "No repository files found to import"
    echo "Looked for: *.repos, repos, *.rosinstall, .rosinstall"
else
    echo "Found ${#REPOS_FILES[@]} repository file(s)"
    for repos_file in "${REPOS_FILES[@]}"; do
        import_repos "$repos_file" "$UNDERLAY_SRC"
    done
fi

echo "Repository update completed"