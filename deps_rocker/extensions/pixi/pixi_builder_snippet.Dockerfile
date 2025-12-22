# syntax=docker/dockerfile:1.4

@(f"FROM {pixi_builder_image_ref} AS {builder_stage}")

WORKDIR /tmp/pixi-build

@(f"COPY {pixi_manifest_name} pixi.toml")
@(f"COPY {pixi_lock_name} pixi.lock" if pixi_lock_present else "")

# Install the tools environment into a deterministic builder output path
@(f"ENV HOME={builder_output_dir}/home")
@(f"ENV PIXI_HOME={builder_output_dir}/pixi/.pixi")
ENV PATH="/usr/local/bin:${PATH}"

RUN mkdir -p @(f"{builder_output_dir}")/bin @(f"{builder_output_dir}")/pixi

RUN --mount=type=cache,target=/root/.cache/pixi,id=pixi-cache \
    --mount=type=cache,target=/root/.cache/rattler,id=rattler-cache \
    bash -c "set -euo pipefail; \
    mkdir -p \"${HOME}\" \"${PIXI_HOME}\"; \
    install_args='-e @(pixi_env_name)'; \
    if [ -f pixi.lock ]; then install_args=\"${install_args} --locked\"; fi; \
    pixi install \${install_args}; \
    DEFAULT_PIXI=\"/tmp/pixi-build/.pixi\"; \
    PIXI_SOURCE=\"\${DEFAULT_PIXI}\"; \
    if [ -d \"\${PIXI_HOME}/envs/@(pixi_env_name)\" ]; then \
        PIXI_SOURCE=\"\${PIXI_HOME}\"; \
    fi; \
    echo \"Pixi sources: home=\${PIXI_HOME} default=\${DEFAULT_PIXI} selected=\${PIXI_SOURCE}\"; \
    ls -la \"\${PIXI_SOURCE}\" || true; \
    if [ ! -d \"\${PIXI_SOURCE}\" ]; then \
        ALT_PIXI=\$(find /tmp/pixi-build /opt /root -maxdepth 6 -type d -name .pixi -print -quit || true); \
        if [ -n \"\${ALT_PIXI}\" ]; then PIXI_SOURCE=\"\${ALT_PIXI}\"; fi; \
    fi; \
    if [ ! -d \"\${PIXI_SOURCE}\" ]; then \
        echo 'Pixi environment not found after install' >&2; \
        find /tmp/pixi-build /opt /root -maxdepth 6 -type d -name .pixi -print || true; \
        exit 1; \
    fi; \
    mkdir -p \"\${PIXI_SOURCE}/envs/@(pixi_env_name)/bin\"; \
    mkdir -p @(f"{builder_output_dir}")/pixi/.pixi; \
    cp /usr/local/bin/pixi @(f"{builder_output_dir}")/bin/pixi; \
    cp pixi.toml @(f"{builder_output_dir}")/pixi/pixi.toml; \
    if [ -f pixi.lock ]; then cp pixi.lock @(f"{builder_output_dir}")/pixi/pixi.lock; fi; \
    if [ \"\${PIXI_SOURCE}\" != \"@(f"{builder_output_dir}")/pixi/.pixi\" ]; then \
        cp -a \"\${PIXI_SOURCE}\"/. @(f"{builder_output_dir}")/pixi/.pixi; \
    fi"
