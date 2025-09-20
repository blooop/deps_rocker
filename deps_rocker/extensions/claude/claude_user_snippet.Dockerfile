RUN bash -lc 'export PATH="$HOME/.local/bin:$PATH"; curl -fsSL https://claude.ai/install.sh | bash' \
    && printf '%s\n' '#!/usr/bin/env sh' 'export PATH="$HOME/.local/bin:$PATH"' 'exec "$HOME/.local/bin/claude" "$@@"' | sudo tee /usr/local/bin/claude >/dev/null \
    && sudo chmod +x /usr/local/bin/claude \
    && echo 'Claude installed. PATH wrapper ensures ~/.local/bin present.'
