RUN curl -LsSf https://astral.sh/uv/install.sh | sh

RUN echo 'eval "$(uv generate-shell-completion bash)"' >> ~/.bashrc; echo 'eval "$(uvx --generate-shell-completion bash)"' >> ~/.bashrc
