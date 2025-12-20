# syntax=docker/dockerfile:1.4

@(f"FROM {base_image} AS {builder_stage}")



# Download and install Homebrew with caching
RUN --mount=type=cache,target=/tmp/homebrew-cache,id=homebrew-install-cache \
    bash -c "set -euxo pipefail && \
    OUTPUT_DIR='@(f"{builder_output_dir}")' && \
    mkdir -p /tmp/homebrew-cache \"\$OUTPUT_DIR\" && \
    installer=/tmp/homebrew-cache/install.sh && \
    if [ ! -f \"\$installer\" ]; then \
        curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh -o \"\$installer\"; \
    fi && \
    NONINTERACTIVE=1 bash \"\$installer\" && \
    cp -a /home/linuxbrew/.linuxbrew \"\$OUTPUT_DIR/.linuxbrew\""
