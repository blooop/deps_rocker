# Provide Pixi installation bundle for user stage
COPY --from=@builder_stage@ @builder_output_dir@/.pixi /opt/deps_rocker/pixi
RUN chmod -R a+rX /opt/deps_rocker/pixi
