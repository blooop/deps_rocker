#!/bin/bash

set -e

# Check that .ssh directory exists
if [ ! -d "$HOME/.ssh" ]; then
  echo ".ssh directory does not exist in the container."
  exit 1
fi

# Print SSH version (fix: use correct command)
ssh -V
