#!/bin/bash
set -euo pipefail

echo "Testing pixi installation and baked tools env..."

if ! command -v pixi >/dev/null 2>&1; then
  echo "ERROR: pixi binary not found on PATH"
  exit 1
fi

pixi --version
echo "PIXI_HOME=${PIXI_HOME:-unset}"
echo "PATH=$PATH"

if [ ! -d "${PIXI_HOME:-/opt/pixi/.pixi}/envs/tools/bin" ]; then
  echo "ERROR: pixi tools environment not found at ${PIXI_HOME:-/opt/pixi/.pixi}/envs/tools/bin"
  exit 1
fi

echo "pixi is installed and baked tools env is present"
