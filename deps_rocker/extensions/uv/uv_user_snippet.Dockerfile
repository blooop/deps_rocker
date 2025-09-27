RUN echo 'eval "$(uv generate-shell-completion bash)"' >> ~/.bashrc; echo 'eval "$(uvx --generate-shell-completion bash)"' >> ~/.bashrc \
    && mkdir -p ~/.local/bin \
    && echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc \
    && echo 'UV tool PATH configured'
