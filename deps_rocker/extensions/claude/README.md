Claude deps_rocker Extension

Goal: Ensure Claude Code inside a rocker-launched container uses the same config/state as the host and never triggers first‑time setup if `~/.claude` already exists on the host.

How it works
- Depends on `user` and `curl` extensions. The `user` extension creates a non‑root user that mirrors the host UID/GID and sets `HOME` accordingly.
- At docker run time, this extension:
  - Bind mounts the host Claude config directory into the container:
    - Prefers `$XDG_CONFIG_HOME/claude` when present on the host
    - Else falls back to `$HOME/.claude` (or `$HOME/.config/claude` if present)
  - Resolves symlinks on the host before mounting to avoid bind‑mount surprises
  - Exports `CLAUDE_CONFIG_DIR` to the target path inside the container to pin Claude’s runtime config resolution
  - Also mounts `$HOME/.cache/claude` and `$HOME/.local/share/claude` when present
- During image build, Claude is installed and symlinked to `/usr/local/bin/claude` so it’s always on `PATH`.

rockerc.yaml example
```
extensions:
  - user
  - curl
  - claude
```
Equivalent docker run args injected by this extension look like:
```
--volume "$HOME/.claude:/home/$USER/.claude" \
--env "CLAUDE_CONFIG_DIR=/home/$USER/.claude"
```
If the host defines `XDG_CONFIG_HOME`, the mount/env use `/home/$USER/.config/claude` instead.

Plain rocker CLI example
```
rocker --user --curl --claude -- \
  bash -lc 'claude --version'
```

Runtime verification
Run these checks inside the container (or use the project’s claude test.sh):
```
whoami; id
echo "HOME=$HOME"
echo "CLAUDE_CONFIG_DIR=${CLAUDE_CONFIG_DIR:-$HOME/.claude}"
ls -al "${CLAUDE_CONFIG_DIR}" | head || true
which claude
claude --version
```
If `${CLAUDE_CONFIG_DIR}` is missing or not writable, fix mounts/permissions and try again.

Troubleshooting
- Wrong HOME/user
  - Ensure the `user` extension is enabled so the container user mirrors your host UID/GID. Without it, mounted files may be read‑only or end up in `/root`.
- Read‑only mount
  - Don’t add `:ro` to the volume. Claude may need to write state files (e.g., tokens or settings).
- Symlinked host config
  - If `~/.claude` is a symlink, this extension resolves it on the host and mounts the real path.
- Missing env
  - `CLAUDE_CONFIG_DIR` is set automatically. If you are overriding entrypoints or run args, ensure you don’t unset it.
- Permissions
  - If the mount path isn’t writable, confirm your host directory ownership matches your UID/GID and that the `user` extension is active.
