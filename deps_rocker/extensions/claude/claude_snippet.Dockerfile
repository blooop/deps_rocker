# Install Claude Code CLI via npm
RUN npm install -g @@anthropic-ai/claude-code \
    && uv tool install claude-monitor \
    && echo 'Claude Code and monitor installed.'
