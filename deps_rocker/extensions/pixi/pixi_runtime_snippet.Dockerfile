# syntax=docker/dockerfile:1.4

# Copy pixi binary only (no baked environment)
COPY --from=@(pixi_builder_image_ref) /usr/local/bin/pixi /usr/local/bin/pixi

# Create directories for cache and detached environments
RUN mkdir -p /etc/pixi /opt/pixi/cache /opt/pixi/envs

# Copy pixi configuration for detached environments
COPY pixi_config.toml /etc/pixi/config.toml

# Copy manifest and lockfile for runtime installation
COPY @(pixi_manifest_name) /opt/pixi/pixi.toml
@(f"COPY {pixi_lock_name} /opt/pixi/pixi.lock" if pixi_lock_present else "")

# Environment variables for pixi runtime
ENV PIXI_CACHE_DIR="/opt/pixi/cache"
ENV PIXI_CONFIG_FILE="/etc/pixi/config.toml"
ENV PIXI_MANIFEST="/opt/pixi/pixi.toml"
ENV PIXI_ENV_NAME="@(pixi_env_name)"

# Copy and set entrypoint for runtime environment activation
COPY pixi_entrypoint.sh /usr/local/bin/pixi_entrypoint.sh
RUN chmod +x /usr/local/bin/pixi_entrypoint.sh

# Note: Entrypoint can be set via rocker's --entrypoint flag or by extension
# For now, document that users should run with proper volume mounts
