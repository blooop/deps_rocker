# syntax=docker/dockerfile:1.4
@(f"ARG PIXI_VERSION={PIXI_VERSION}")

# Provide Pixi installation bundle for user stage
@(f"COPY --from={builder_stage} {builder_output_dir}/.pixi /opt/deps_rocker/pixi")
RUN chmod -R a+rX /opt/deps_rocker/pixi
