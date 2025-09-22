# Install OpenAI Codex CLI
RUN npm install -g @@openai/codex && \
    npm list -g @@openai/codex && \
    which codex || echo "Codex not in PATH, checking npm prefix" && \
    echo "npm prefix: $(npm prefix -g)" && \
    ls -la $(npm prefix -g)/bin/codex* || echo "Checking other locations" && \
    find /usr -name "codex*" 2>/dev/null || true