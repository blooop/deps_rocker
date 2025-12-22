#!/usr/bin/env bash
# Pixi runtime environment entrypoint
# Ensures pixi environment exists and activates it before executing user command

set -euo pipefail

# Pixi runtime configuration
export PIXI_CACHE_DIR="${PIXI_CACHE_DIR:-/opt/pixi/cache}"
export PIXI_CONFIG_FILE="${PIXI_CONFIG_FILE:-/etc/pixi/config.toml}"

# Environment configuration
MANIFEST="${PIXI_MANIFEST:-/workspace/.deps-rocker/pixi.toml}"
ENV_NAME="${PIXI_ENV_NAME:-tools}"

# Install and activate pixi environment if manifest exists
if [ -f "$MANIFEST" ]; then
    echo "Installing pixi environment '$ENV_NAME' from $MANIFEST..."

    install_flags="--environment $ENV_NAME"

    # Use locked install if lockfile exists
    if [ -f "${MANIFEST%.toml}.lock" ]; then
        echo "Using locked dependencies from ${MANIFEST%.toml}.lock"
        install_flags="$install_flags --locked"
    fi

    # Install environment (idempotent, uses cache if available)
    pixi install --manifest-path "$MANIFEST" $install_flags

    echo "Activating pixi environment '$ENV_NAME'..."

    # Generate and source activation script
    ACTIVATION_SCRIPT=$(mktemp)
    pixi shell-hook --manifest-path "$MANIFEST" --environment "$ENV_NAME" > "$ACTIVATION_SCRIPT"

    # shellcheck disable=SC1090
    source "$ACTIVATION_SCRIPT"
    rm "$ACTIVATION_SCRIPT"

    echo "Pixi environment activated successfully"
else
    echo "No pixi manifest found at $MANIFEST, skipping environment setup"
fi

# Execute user command
exec "$@"
