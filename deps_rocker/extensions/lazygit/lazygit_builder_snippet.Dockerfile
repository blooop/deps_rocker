# syntax=docker/dockerfile:1.4

@(f"FROM {base_image} AS {builder_stage}")

ARG LAZYGIT_VERSION=@LAZYGIT_VERSION@

RUN --mount=type=cache,target=/tmp/lazygit-cache,id=lazygit-release-cache \
    bash -c "set -euxo pipefail && \
    OUTPUT_DIR='@(f"{builder_output_dir}")' && \
    mkdir -p /tmp/lazygit-cache \"\$OUTPUT_DIR\" && \
    version_file=/tmp/lazygit-cache/version.txt && \
    if [ ! -f \"\$version_file\" ]; then \
        echo \"${LAZYGIT_VERSION}\" > \"\$version_file\"; \
    fi && \
    LAZYGIT_VERSION=\$(cat \"\$version_file\" | tr -d '[:space:]') && \
    : \"Using lazygit version '\${LAZYGIT_VERSION}'\" && \
    tarball=\"/tmp/lazygit-cache/lazygit_\${LAZYGIT_VERSION}_Linux_x86_64.tar.gz\" && \
    if [ ! -f \"\$tarball\" ]; then \
        curl -fL --retry 3 --retry-delay 3 \"https://github.com/jesseduffield/lazygit/releases/download/v\${LAZYGIT_VERSION}/lazygit_\${LAZYGIT_VERSION}_Linux_x86_64.tar.gz\" -o \"\$tarball\"; \
    fi && \
    tar -xzf \"\$tarball\" lazygit && \
    install -Dm755 lazygit \"\$OUTPUT_DIR/lazygit\" && \
    rm -f lazygit"
