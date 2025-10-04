# Install Rust and Cargo via rustup, caching the installer download with BuildKit
RUN --mount=type=cache,target=/root/.cache/rustup \
    if [ ! -f /root/.cache/rustup/rustup-init.sh ]; then \
        curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs -o /root/.cache/rustup/rustup-init.sh; \
    fi && \
    sh /root/.cache/rustup/rustup-init.sh -y --default-toolchain stable && \
    . ~/.cargo/env && \
    echo 'source ~/.cargo/env' >> ~/.bashrc

ENV PATH="/root/.cargo/bin:${PATH}"
