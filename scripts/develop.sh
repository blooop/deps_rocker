#!/usr/bin/env bash
set -euo pipefail

# Config
UV_ENV="$HOME/.venvs/dev-tools-uv"
PROJ_A="$HOME/projects/rockerc"
PROJ_B="$HOME/projects/deps_rocker"

# Create venv if missing
if [ ! -d "$UV_ENV" ]; then
    echo "Creating uv venv at $UV_ENV"
    uv venv "$UV_ENV"
else
    echo "Using existing uv venv at $UV_ENV"
fi

# Add to ~/.bashrc if not already present
if ! grep -Fq "$UV_ENV/bin" "$HOME/.bashrc"; then
    echo "export PATH=\"$UV_ENV/bin:\$PATH\"" >> "$HOME/.bashrc"
    echo "Added venv bin to ~/.bashrc (open a new shell or source ~/.bashrc to use)"
fi

# Install projects editable
uv pip install --python "$UV_ENV" -U pip
uv pip install --python "$UV_ENV" -e "$PROJ_A"
uv pip install --python "$UV_ENV" -e "$PROJ_B"

echo "✅ Setup complete."
echo "Restart your shell or run:  source ~/.bashrc"
echo "Console scripts from both projects and dependencies will now be on PATH."
