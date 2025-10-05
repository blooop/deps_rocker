# syntax=docker/dockerfile:1.4
ARG PIXI_VERSION=@pixi_version@

# Provide Pixi installation bundle for user stage
COPY --from=@builder_stage@ @builder_output_dir@/.pixi /opt/deps_rocker/pixi
RUN chmod -R a+rX /opt/deps_rocker/pixi
