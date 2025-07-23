#!/bin/bash

# Update extensions with package paths
extensions=("fzf" "git_clone" "isaac_sim" "locales" "neovim" "ros_humble" "tzdata" "urdf_viz" "uv" "vcstool")

for ext in "${extensions[@]}"; do
    echo "Updating $ext extension..."
    
    # Get the class name from the file
    class_name=$(grep "^class " deps_rocker/extensions/$ext/$ext.py | cut -d' ' -f2 | cut -d'(' -f1)
    
    # Update the extension file to add the pkg attribute
    sed -i "/name = \"$ext\"/a\\    pkg = \"deps_rocker.extensions.$ext\"" deps_rocker/extensions/$ext/$ext.py
    
    # Create __init__.py for the extension
    echo "from .$ext import $class_name" > deps_rocker/extensions/$ext/__init__.py
    echo "" >> deps_rocker/extensions/$ext/__init__.py
    echo "__all__ = [\"$class_name\"]" >> deps_rocker/extensions/$ext/__init__.py
done
