# syntax=docker/dockerfile:1.4

# Use official pixi image to install nodejs
FROM ghcr.io/prefix-dev/pixi:latest AS npm-builder

# Install nodejs via pixi global install (installs to /root/.pixi/envs/nodejs)
RUN pixi global install nodejs

# Copy nodejs environment to builder output
@(f"FROM {base_image} AS {builder_stage}")
RUN mkdir -p @(f"{builder_output_dir}")/nodejs
COPY --from=npm-builder /root/.pixi/envs/nodejs @(f"{builder_output_dir}")/nodejs
