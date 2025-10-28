# syntax=docker/dockerfile:1.4

@(f"FROM {base_image} AS {builder_stage}")

RUN --mount=type=cache,target=/tmp/claude-install-cache,id=claude-install-cache \
    bash -c "set -euxo pipefail && \
    mkdir -p /tmp/claude-install-cache && \
    mkdir -p @(builder_output_dir) && \
    \
    # Detect platform \
    OS=\$(uname -s | tr '[:upper:]' '[:lower:]') && \
    ARCH=\$(uname -m) && \
    case \$ARCH in \
        x86_64) ARCH=x64 ;; \
        aarch64|arm64) ARCH=arm64 ;; \
        *) echo \"Unsupported architecture: \$ARCH\" && exit 1 ;; \
    esac && \
    PLATFORM=\"\${OS}-\${ARCH}\" && \
    \
    # Get current stable version \
    GCS_BUCKET=\"https://storage.googleapis.com/claude-code-dist-86c565f3-f756-42ad-8dfa-d59b1c096819/claude-code-releases\" && \
    CURRENT_STABLE=\$(curl -sSL \$GCS_BUCKET/stable) && \
    VERSION_FILE=/tmp/claude-install-cache/\$PLATFORM-version && \
    BINARY_CACHE=/tmp/claude-install-cache/claude-\$CURRENT_STABLE-\$PLATFORM && \
    BOOTSTRAP_CACHE=/tmp/claude-install-cache/bootstrap.sh && \
    \
    # Download bootstrap script if needed \
    if [ ! -f \$BOOTSTRAP_CACHE ] || [ ! -f \$VERSION_FILE ] || [ \"\$(cat \$VERSION_FILE 2>/dev/null || echo '')\" != \"\$CURRENT_STABLE\" ]; then \
        echo \"Downloading install script (version: \$CURRENT_STABLE)\" && \
        curl -sSL -o \$BOOTSTRAP_CACHE \$GCS_BUCKET/bootstrap.sh; \
    else \
        echo \"Using cached install script for version \$CURRENT_STABLE\"; \
    fi && \
    \
    # Download Claude binary if not cached \
    if [ ! -f \$BINARY_CACHE ]; then \
        echo \"Downloading Claude binary for \$PLATFORM (version: \$CURRENT_STABLE)\" && \
        curl -sSL -o \$BINARY_CACHE \$GCS_BUCKET/\$CURRENT_STABLE/\$PLATFORM/claude && \
        echo \$CURRENT_STABLE > \$VERSION_FILE; \
    else \
        echo \"Using cached Claude binary for version \$CURRENT_STABLE\"; \
    fi && \
    \
    # Copy cached files to builder output \
    cp \$BOOTSTRAP_CACHE @(builder_output_dir)/install.sh && \
    cp \$BINARY_CACHE @(builder_output_dir)/claude-binary && \
    chmod +x @(builder_output_dir)/claude-binary"

COPY claude-wrapper.sh @(builder_output_dir)/claude-wrapper.sh
