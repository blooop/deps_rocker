RUN --mount=type=cache,target=/root/.cache/chrome-download,id=chrome-download-cache \
    mkdir -p /root/.cache/chrome-download && \
    cd /root/.cache/chrome-download && \
    if [ ! -f "google-chrome-stable_current_amd64.deb" ]; then \
        curl -fsSL -o google-chrome-stable_current_amd64.deb https://dl.google.com/linux/direct/google-chrome-stable_current_amd64.deb; \
    fi && \
    dpkg -i google-chrome-stable_current_amd64.deb || true && \
    apt-get update && \
    apt-get install -f -y && \
    rm -rf /var/lib/apt/lists/*