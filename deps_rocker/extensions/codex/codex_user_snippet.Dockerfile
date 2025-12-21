# Codex CLI is installed at runtime via npm after pixi sync

# Create symlink in /usr/local/bin for non-interactive shell access
RUN sudo ln -sf ~/.pixi/envs/nodejs/bin/codex /usr/local/bin/codex
