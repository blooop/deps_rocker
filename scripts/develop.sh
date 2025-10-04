#!/usr/bin/env bash
set -euo pipefail

# Config
UV_ENV="$HOME/.venvs/dev-tools-uv"
PROJ_A="$HOME/projects/rockerc"
PROJ_B="$HOME/projects/deps_rocker"

# Always wipe existing environment
if [ -d "$UV_ENV" ]; then
    echo "🗑️  Removing existing environment at $UV_ENV"
    rm -rf "$UV_ENV"
fi
# Remove specific export line from bashrc if present
if grep -Fq "export PATH=\"$UV_ENV/bin:\$PATH\"" "$HOME/.bashrc" 2>/dev/null; then
    echo "🧹 Removing venv PATH export from ~/.bashrc"
    awk '!/^export PATH="'"$UV_ENV"'\/bin:\$PATH"$/' "$HOME/.bashrc" > "$HOME/.bashrc.tmp" && mv "$HOME/.bashrc.tmp" "$HOME/.bashrc"
fi

# Create fresh venv
echo "Creating uv venv at $UV_ENV"
uv venv "$UV_ENV"

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
