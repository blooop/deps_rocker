RUN mkdir -p ~/.local/bin
ENV SHELL=/bin/bash
RUN echo 'eval "$(uv generate-shell-completion bash)"' >> ~/.bashrc; echo 'eval "$(uvx --generate-shell-completion bash)"' >> ~/.bashrc \
    && (uv tool update-shell || echo 'uv tool update-shell already configured') \
    && echo 'UV tool PATH configured'
