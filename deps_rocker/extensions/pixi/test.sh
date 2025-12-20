#!/bin/bash

set -e

export PATH="$HOME/.pixi/bin:$PATH"
source ~/.bashrc
pixi --version
echo "pixi is installed and working"
