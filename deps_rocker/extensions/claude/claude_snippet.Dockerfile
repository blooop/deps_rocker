# Install Claude Code CLI via npm
RUN npm install -g @@anthropic-ai/claude-code \
    && echo 'Claude Code installed.'

# Set environment variable to confirm latest Claude extension is loaded
ENV CLAUDE_EXTENSION_VERSION="2025-01-01-config-fix"
