# syntax=docker/dockerfile:1.4

@(f"FROM {base_image} AS {builder_stage}")

ARG FOXGLOVE_VERSION=@FOXGLOVE_VERSION@

RUN --mount=type=cache,target=/tmp/foxglove-cache,sharing=locked,id=foxglove-deb-cache \
    bash -c "set -euxo pipefail && \
    OUTPUT_DIR='@(f"{builder_output_dir}")' && \
    mkdir -p /tmp/foxglove-cache \"\$OUTPUT_DIR\" && \
    DEB_NAME=\"foxglove-studio-\${FOXGLOVE_VERSION}-linux-amd64.deb\" && \
    CACHE_DEB=\"/tmp/foxglove-cache/\${FOXGLOVE_VERSION}_foxglove-studio.deb\" && \
    DOWNLOAD_URL=\"https://get.foxglove.dev/desktop/v\${FOXGLOVE_VERSION}/\${DEB_NAME}\" && \
    if [ ! -f \"\$CACHE_DEB\" ]; then \
        echo \"Downloading Foxglove Studio \${FOXGLOVE_VERSION}...\" && \
        curl --retry 3 --retry-delay 5 --progress-bar -L \"\$DOWNLOAD_URL\" -o \"\$CACHE_DEB\"; \
    fi && \
    dpkg-deb --info \"\$CACHE_DEB\" > /dev/null 2>&1 && \
    install -Dm644 \"\$CACHE_DEB\" \"\$OUTPUT_DIR/foxglove-studio.deb\" && \
    printf '%s' \"\${FOXGLOVE_VERSION}\" > \"\$OUTPUT_DIR/version.txt\""
