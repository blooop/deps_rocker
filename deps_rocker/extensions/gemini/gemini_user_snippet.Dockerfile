# Install Google Gemini CLI tool via npm with BuildKit cache for global installs
RUN --mount=type=cache,target=/root/.npm,id=npm-cache \
    npm install -g --prefer-offline --no-audit @@google/gemini-cli

# Create symlink in /usr/local/bin for non-interactive shell access
RUN sudo ln -sf ~/.npm-global/bin/gemini /usr/local/bin/gemini
