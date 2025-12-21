# Configure pixi for the user; runtime entrypoint will populate ~/.pixi if needed
RUN echo 'export PATH="$HOME/.pixi/bin:$PATH"' >> ~/.bashrc
RUN echo 'eval "$(pixi completion --shell bash)"' >> ~/.bashrc
RUN echo '[ -f pixi.toml ] || [ -f pyproject.toml ] && eval "$(pixi shell-hook)" 2>/dev/null || true' >> ~/.bashrc
RUN echo 'for f in ~/.pixi/completions/bash/*; do [ -f "$f" ] && source "$f"; done' >> ~/.bashrc

# Create a profile script that gets sourced by non-interactive shells
RUN echo 'export PATH="$HOME/.pixi/bin:$PATH"' >> ~/.profile

# Configure shell to use bash and source profile for RUN commands
SHELL ["/bin/bash", "-l", "-c"]
