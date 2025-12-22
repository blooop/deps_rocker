# Install OpenAI Codex CLI with BuildKit cache for npm global installs
RUN --mount=type=cache,target=/root/.npm,id=npm-cache \
    npm install -g @@openai/codex

# Create symlink in /usr/local/bin for non-interactive shell access
RUN sudo ln -sf ~/.npm-global/bin/codex /usr/local/bin/codex
