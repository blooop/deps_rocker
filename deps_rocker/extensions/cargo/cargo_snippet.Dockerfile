# Install Rust and Cargo via rustup
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y --default-toolchain stable \
    && . ~/.cargo/env \
    && echo 'source ~/.cargo/env' >> ~/.bashrc

ENV PATH="/root/.cargo/bin:${PATH}"
