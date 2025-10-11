# syntax=docker/dockerfile:1.4

FROM curl_builder AS @(builder_stage)

RUN --mount=type=cache,target=/tmp/claude-install-cache,id=claude-install-cache \
    bash -c "set -euxo pipefail && \
    mkdir -p /tmp/claude-install-cache && \
    mkdir -p @(builder_output_dir) && \
    CACHE_FILE=/tmp/claude-install-cache/bootstrap.sh && \
    VERSION_FILE=/tmp/claude-install-cache/stable-version && \
    CURRENT_STABLE=\$(curl -sSL https://storage.googleapis.com/claude-code-dist-86c565f3-f756-42ad-8dfa-d59b1c096819/claude-code-releases/stable) && \
    if [ ! -f \$CACHE_FILE ] || [ ! -f \$VERSION_FILE ] || [ \"\$(cat \$VERSION_FILE 2>/dev/null || echo '')\" != \"\$CURRENT_STABLE\" ]; then \
        echo \"Downloading install script (current stable: \$CURRENT_STABLE)\" && \
        curl -sSL -o \$CACHE_FILE https://claude.ai/install.sh && \
        echo \$CURRENT_STABLE > \$VERSION_FILE; \
    else \
        echo \"Using cached install script for version \$(cat \$VERSION_FILE)\"; \
    fi && \
    cp \$CACHE_FILE @(builder_output_dir)/install.sh"

COPY claude-wrapper.sh @(builder_output_dir)/claude-wrapper.sh
