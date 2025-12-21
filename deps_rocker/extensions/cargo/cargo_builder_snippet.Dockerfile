# syntax=docker/dockerfile:1.4

# Use official pixi image to install rust
FROM ghcr.io/prefix-dev/pixi:latest AS cargo-builder

# Install rust via pixi global install (installs to /root/.pixi/envs/rust)
RUN pixi global install rust

# Copy rust environment to builder output
@(f"FROM {base_image} AS {builder_stage}")
RUN mkdir -p @(f"{builder_output_dir}")/rust
COPY --from=cargo-builder /root/.pixi/envs/rust @(f"{builder_output_dir}")/rust
