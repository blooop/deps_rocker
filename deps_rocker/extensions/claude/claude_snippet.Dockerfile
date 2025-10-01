# Install Claude Code CLI via npm and claude-monitor via uv
RUN npm install -g @@anthropic-ai/claude-code \
    && uv tool install claude-monitor \
    && echo 'Claude Code and monitor installed.'

# Set environment variable to confirm latest Claude extension is loaded
ENV CLAUDE_EXTENSION_VERSION="2025-01-01-latest"
