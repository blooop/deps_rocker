@(f"FROM {base_image} AS {builder_stage}")

RUN --mount=type=cache,target=/tmp/urdf-viz-cache,id=urdf-viz-cache \
    set -euxo pipefail && \
    OUTPUT_DIR="@builder_output_dir@" && \
    mkdir -p /tmp/urdf-viz-cache "$OUTPUT_DIR" && \
    release_json=$(curl -sL https://api.github.com/repos/openrr/urdf-viz/releases/latest) && \
    download_url=$(echo "$release_json" | jq -r '.assets[] | select(.name == "urdf-viz-x86_64-unknown-linux-gnu.tar.gz") | .browser_download_url') && \
    release_tag=$(echo "$release_json" | jq -r '.tag_name') && \
    asset_name=$(basename "$download_url") && \
    tarball="/tmp/urdf-viz-cache/${release_tag}-${asset_name}" && \
    if [ ! -f "$tarball" ]; then \
        curl -sSL "$download_url" -o "$tarball"; \
    fi && \
    tar -xzf "$tarball" -C /tmp && \
    find /tmp -name urdf-viz -type f -exec install -Dm755 {} "$OUTPUT_DIR/urdf-viz" \; && \
    rm -rf /tmp/urdf-viz*
