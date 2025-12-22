# Install OpenAI Codex CLI via Homebrew cask (must run as non-root user)
# Install ripgrep first as it's a dependency
# Note: This runs in the root section but uses su to run as non-root
RUN su - $(getent passwd 1000 | cut -d: -f1) -c 'brew install ripgrep' && \
    su - $(getent passwd 1000 | cut -d: -f1) -c 'brew install --cask codex'
