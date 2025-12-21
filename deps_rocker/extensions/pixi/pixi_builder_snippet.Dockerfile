# syntax=docker/dockerfile:1.4

# Use official pixi image to extract the binary
FROM ghcr.io/prefix-dev/pixi:latest AS pixi-official

@(f"FROM {base_image} AS {builder_stage}")

# Copy pixi binary from official image
RUN mkdir -p @(f"{builder_output_dir}")/bin
COPY --from=pixi-official /usr/local/bin/pixi @(f"{builder_output_dir}")/bin/pixi
