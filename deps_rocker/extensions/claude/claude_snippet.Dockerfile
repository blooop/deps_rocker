# Install Claude Code CLI via official installer and claude-monitor via uv
RUN curl -fsSL https://claude.ai/install.sh | bash \
    && uv tool install claude-monitor \
    && echo 'Claude Code and monitor installed.'

# Set environment variable to confirm latest Claude extension is loaded
ENV CLAUDE_EXTENSION_VERSION="2025-01-01-latest"
