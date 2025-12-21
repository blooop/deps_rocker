# Install Google Gemini CLI tool via npm with BuildKit cache for global installs
RUN npm install -g @@google/gemini-cli

# Create symlink in /usr/local/bin for non-interactive shell access
RUN sudo ln -sf ~/.pixi/envs/nodejs/bin/gemini /usr/local/bin/gemini
