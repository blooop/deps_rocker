# Create cache directories for pixi and uv (used by pixi)
RUN mkdir -p ~/.cache/pixi ~/.cache/rattler ~/.cache/uv

# Set up shell completions and hooks for pixi
RUN echo 'eval "$(pixi completion --shell bash)"' >> ~/.bashrc
RUN echo '[ -f pixi.toml ] || [ -f pyproject.toml ] && eval "$(pixi shell-hook)" 2>/dev/null || true' >> ~/.bashrc

# Set up shell completions for pixi global tools (if available)
RUN if [ -d ~/.pixi/completions/bash/ ]; then \
    echo 'for f in ~/.pixi/completions/bash/*; do [ -f "$f" ] && source "$f"; done' >> ~/.bashrc; \
fi

# Configure shell to use bash for RUN commands
SHELL ["/bin/bash", "-c"]
