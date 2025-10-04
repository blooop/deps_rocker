
# Download and install latest urdf-viz release using BuildKit cache for downloads
RUN --mount=type=cache,target=/tmp/urdf-viz-cache \
    curl -sL https://api.github.com/repos/openrr/urdf-viz/releases/latest | \
    jq -r '.assets[] | select(.name == "urdf-viz-x86_64-unknown-linux-gnu.tar.gz") | .browser_download_url' | \
    xargs -I {{}} curl -sL {{}} -o /tmp/urdf-viz-cache/urdf-viz.tar.gz && \
    mkdir -p /tmp/urdf-viz && \
    tar -xzf /tmp/urdf-viz-cache/urdf-viz.tar.gz -C /tmp/urdf-viz && \
    find /tmp/urdf-viz -name urdf-viz -type f -exec cp {{}} /usr/local/bin/ \; && \
    chmod +x /usr/local/bin/urdf-viz && \
    rm -rf /tmp/urdf-viz /tmp/urdf-viz-cache/urdf-viz.tar.gz
