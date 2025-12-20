# syntax=docker/dockerfile:1.4

@(f"ARG PIXI_VERSION={PIXI_VERSION}")

# Use BuildKit cache mounts for pixi and uv caches
RUN --mount=type=cache,target=/root/.cache/pixi \
	echo "Pixi and uv caches mounted for faster builds"

# Provide Pixi installation bundle for user stage
@(f"COPY --from={builder_stage} {builder_output_dir}/.pixi /opt/deps_rocker/pixi")
RUN chmod -R a+rX /opt/deps_rocker/pixi
