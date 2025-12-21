# syntax=docker/dockerfile:1.4

# Provide Pixi binary and baked tools environment
@(f"COPY --from={builder_stage} {builder_output_dir}/bin/pixi /usr/local/bin/pixi")
@(f"COPY --from={builder_stage} {builder_output_dir}/pixi/.pixi {pixi_runtime_root}/.pixi")
@(f"COPY --from={builder_stage} {builder_output_dir}/pixi/pixi.toml {pixi_runtime_root}/pixi.toml")
@(f"COPY --from={builder_stage} {builder_output_dir}/pixi/pixi.lock {pixi_runtime_root}/pixi.lock")

RUN chmod -R a+rx @(pixi_runtime_root)

ENV PIXI_HOME="@(pixi_runtime_root)/.pixi"
ENV PATH="@(pixi_runtime_root)/.pixi/envs/@(pixi_env_name)/bin:${PATH}"
