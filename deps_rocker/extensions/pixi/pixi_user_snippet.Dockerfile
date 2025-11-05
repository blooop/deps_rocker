# Create cache directories for pixi and uv (used by pixi)
RUN mkdir -p ~/.cache/pixi ~/.cache/rattler ~/.cache/uv

# Install Pixi into the user home from pre-populated bundle
RUN mkdir -p ~/.pixi && cp -a /opt/deps_rocker/pixi/. ~/.pixi/
RUN echo 'export PATH="$HOME/.pixi/bin:$PATH"' >> ~/.bashrc
RUN echo 'eval "$(pixi completion --shell bash)"' >> ~/.bashrc
RUN echo '[ -f pixi.toml ] || [ -f pyproject.toml ] && eval "$(pixi shell-hook)" 2>/dev/null || true' >> ~/.bashrc
# Set up shell completions for pixi global tools (if available)
RUN if [ -d ~/.pixi/completions/bash/ ]; then \
    echo 'for f in ~/.pixi/completions/bash/*; do [ -f "$f" ] && source "$f"; done' >> ~/.bashrc; \
fi

# Create a profile script that gets sourced by non-interactive shells
RUN echo 'export PATH="$HOME/.pixi/bin:$PATH"' >> ~/.profile

# Configure shell to use bash and source profile for RUN commands
SHELL ["/bin/bash", "-l", "-c"]
