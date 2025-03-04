# Download and install latest urdf-viz release

RUN apt-get update && apt-get install -y --no-install-recommends \
    libxi6 \
    libxcursor-dev \
    libxrandr-dev \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y jq ros-humble-xacro && \
    curl -sL https://api.github.com/repos/openrr/urdf-viz/releases/latest | \
    jq -r '.assets[] | select(.name == "urdf-viz-x86_64-unknown-linux-gnu.tar.gz") | .browser_download_url' | \
    xargs -I {} curl -sL {} -o urdf-viz.tar.gz && \
    mkdir -p /tmp/urdf-viz && \
    tar -xzf urdf-viz.tar.gz -C /tmp/urdf-viz && \
    find /tmp/urdf-viz -name urdf-viz -type f -exec cp {} /usr/local/bin/ \; && \
    chmod +x /usr/local/bin/urdf-viz && \
    rm -rf /tmp/urdf-viz urdf-viz.tar.gz
