# Set up UV shell completion
RUN echo 'eval "$(uv generate-shell-completion bash)"' >> ~/.bashrc
RUN echo 'eval "$(uvx --generate-shell-completion bash)"' >> ~/.bashrc