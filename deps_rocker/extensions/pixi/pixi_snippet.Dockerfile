# syntax=docker/dockerfile:1.4

# Use BuildKit cache mounts for pixi and uv caches
RUN --mount=type=cache,target=/root/.cache/pixi \
	echo "Pixi and uv caches mounted for faster builds"

# Provide Pixi binary for user stage
@(f"COPY --from={builder_stage} {builder_output_dir}/bin/pixi /usr/local/bin/pixi")
RUN chmod a+rx /usr/local/bin/pixi
