# syntax=docker/dockerfile:1.4

# Provide Pixi binary and baked tools environment
@(f"COPY --chmod=755 --from={builder_stage} {builder_output_dir}/bin/pixi /usr/local/bin/pixi")
# Copy only the specific environment, not the entire .pixi directory (avoids task-cache, other envs)
@(f"COPY --chmod=755 --from={builder_stage} {builder_output_dir}/pixi/.pixi/envs/{pixi_env_name} {pixi_runtime_root}/.pixi/envs/{pixi_env_name}")
@(f"COPY --chmod=644 --from={builder_stage} {builder_output_dir}/pixi/pixi.toml {pixi_runtime_root}/pixi.toml")
@(f"COPY --chmod=644 --from={builder_stage} {builder_output_dir}/pixi/pixi.lock {pixi_runtime_root}/pixi.lock")

ENV PIXI_HOME="@(pixi_runtime_root)/.pixi"
ENV PATH="@(pixi_runtime_root)/.pixi/envs/@(pixi_env_name)/bin:${PATH}"
