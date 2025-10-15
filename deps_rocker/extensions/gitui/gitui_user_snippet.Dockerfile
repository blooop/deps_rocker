# Install GitUI via pixi global install in user environment
RUN export PATH="$HOME/.pixi/bin:$PATH" && pixi global install gitui

# Set up shell completions for GitUI (if available)
RUN if [ -d ~/.pixi/completions/bash/ ]; then \
    echo 'for f in ~/.pixi/completions/bash/*; do [ -f "$f" ] && source "$f"; done' >> ~/.bashrc; \
fi