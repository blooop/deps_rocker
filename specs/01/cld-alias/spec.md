# Add `cld` alias for Claude CLI

## Summary
Add a shell alias `cld` that expands to `claude --dangerously-skip-permissions` for convenience.

## Requirements
- Alias should be available in bash shell
- Alias should be available in zsh shell (if present)
- Alias: `cld` → `claude --dangerously-skip-permissions`

## Implementation
Modify `claude_user_snippet.Dockerfile` to append the alias to `.bashrc` and `.zshrc`.
