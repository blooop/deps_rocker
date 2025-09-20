RUN bash -lc 'export PATH="$HOME/.local/bin:$PATH"; curl -fsSL https://claude.ai/install.sh | bash' \
    && sudo ln -sf "$HOME/.local/bin/claude" /usr/local/bin/claude \
    && echo 'Claude installed. PATH ensured via /usr/local/bin; ignore any PATH note above.'
