# syntax=docker/dockerfile:1.4
ARG LAZYGIT_VERSION=@LAZYGIT_VERSION@

@(f"FROM {base_image} AS {builder_stage}")

RUN --mount=type=cache,target=/tmp/lazygit-cache,id=lazygit-release-cache \
    bash -c "set -euxo pipefail && \
    OUTPUT_DIR='@(f"{builder_output_dir}")' && \
    mkdir -p /tmp/lazygit-cache \"\$OUTPUT_DIR\" && \
    version_file=/tmp/lazygit-cache/version.txt && \
    if [ ! -f \"\$version_file\" ]; then \
        curl -s \"https://api.github.com/repos/jesseduffield/lazygit/releases/latest\" \
            | grep -Eo '\"tag_name\": \"v[^\" ]*\"' \
            | head -n1 \
            | sed 's/.*\"v\\([^\" ]*\\)\".*/\\1/' > \"\$version_file\"; \
    fi && \
    LAZYGIT_VERSION=\$(cat \"\$version_file\") && \
    tarball=\"/tmp/lazygit-cache/lazygit_\${LAZYGIT_VERSION}_Linux_x86_64.tar.gz\" && \
    if [ ! -f \"\$tarball\" ]; then \
        curl -sSL \"https://github.com/jesseduffield/lazygit/releases/download/v\${LAZYGIT_VERSION}/lazygit_\${LAZYGIT_VERSION}_Linux_x86_64.tar.gz\" -o \"\$tarball\"; \
    fi && \
    tar -xzf \"\$tarball\" lazygit && \
    install -Dm755 lazygit \"\$OUTPUT_DIR/lazygit\" && \
    rm -f lazygit"
