# syntax=docker/dockerfile:1.4

@(f"FROM {base_image} AS {builder_stage}")

# Install required dependencies for Homebrew
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked,id=apt-cache \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked,id=apt-lists \
    apt-get update && \
    apt-get install -y --no-install-recommends \
        build-essential \
        procps \
        curl \
        file \
        git && \
    rm -rf /var/lib/apt/lists/*

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
