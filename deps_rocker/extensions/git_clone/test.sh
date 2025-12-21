#!/bin/bash

set -e

git --version
echo "git is installed and working"

# Test git-lfs
git lfs version
echo "git-lfs is installed and working"
