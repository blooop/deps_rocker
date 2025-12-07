# syntax=docker/dockerfile:1.4

@(f"ARG PIXI_VERSION={PIXI_VERSION}")

# Copy pixi binary to system location for immediate use
@(f"COPY --from={builder_stage} {builder_output_dir}/.pixi/bin/pixi /usr/local/bin/pixi")

# Provide Pixi installation bundle for user stage
@(f"COPY --from={builder_stage} {builder_output_dir}/.pixi /opt/deps_rocker/pixi")
RUN chmod -R a+rX /opt/deps_rocker/pixi
