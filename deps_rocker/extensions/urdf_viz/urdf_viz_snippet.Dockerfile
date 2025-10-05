
# Download and install latest urdf-viz release using BuildKit cache for downloads
RUN --mount=type=cache,target=/tmp/urdf-viz-cache \
    mkdir -p /tmp/urdf-viz-cache && \
    release_json=$(curl -sL https://api.github.com/repos/openrr/urdf-viz/releases/latest) && \
    download_url=$(echo "${release_json}" | jq -r '.assets[] | select(.name == "urdf-viz-x86_64-unknown-linux-gnu.tar.gz") | .browser_download_url') && \
    release_tag=$(echo "${release_json}" | jq -r '.tag_name') && \
    asset_name=$(basename "${download_url}") && \
    tarball="/tmp/urdf-viz-cache/${release_tag}-${asset_name}" && \
    if [ ! -f "${tarball}" ]; then \
        curl -sSL "${download_url}" -o "${tarball}"; \
    fi && \
    mkdir -p /tmp/urdf-viz && \
    tar -xzf "${tarball}" -C /tmp/urdf-viz && \
    find /tmp/urdf-viz -name urdf-viz -type f -exec cp {} /usr/local/bin/ \; && \
    chmod +x /usr/local/bin/urdf-viz && \
    rm -rf /tmp/urdf-viz
