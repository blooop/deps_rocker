RUN mkdir -p ~/.local/bin
ENV PATH="/home/ags/.local/bin:$PATH"
RUN echo 'eval "$(uv generate-shell-completion bash)"' >> ~/.bashrc; echo 'eval "$(uvx --generate-shell-completion bash)"' >> ~/.bashrc \
    && echo 'UV tool PATH configured'
