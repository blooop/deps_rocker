# User-specific setup for Claude Code CLI
# This runs after the user extension has set up the user environment

# Add ~/.local/bin to PATH for the user
RUN echo 'export PATH="$HOME/.local/bin:$PATH"' >> $HOME/.bashrc
